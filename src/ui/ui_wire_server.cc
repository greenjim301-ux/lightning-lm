#include "ui/ui_wire_server.h"

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <glog/logging.h>

#include <deque>
#include <mutex>
#include <thread>

#include "ui/ui_wire_protocol.h"

namespace lightning::ui {

namespace {
namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

// PCL点云到UiPoint的转换：跟pangolin_window_impl.cc里的同名helper一样，故意各自file-local一份，
// 不共享成头文件——两处都只是几行胶水代码，不值得为它单独拉一个公共头。
std::vector<UiPoint> ToUiPoints(const CloudPtr& cloud) {
    std::vector<UiPoint> pts;
    pts.reserve(cloud->size());
    for (const auto& p : *cloud) {
        pts.push_back(UiPoint{p.x, p.y, p.z, p.intensity});
    }
    return pts;
}

constexpr auto kFrontendPoseMinInterval = std::chrono::milliseconds(33);  // ~30Hz上限
constexpr auto kKeyframeSyncInterval = std::chrono::milliseconds(300);    // ~3Hz
// offline回放（run_slam_offline）没有任何限速，实测能跑到真实传感器速率的3~5倍；每个scan消息
// 是全分辨率点云（几千到上万点，几十上百KB），wasm端每条都要重建GL buffer+上传，跟不上时会在
// 浏览器的websocket消息队列里排队积压——因为是单条有序连接，连轨迹/位姿这种廉价消息也会被
// 堵在积压的scan消息后面一起变慢。在这里限速到接近真实传感器速率，跟SendFrontendPose同样的思路。
constexpr auto kScanMinInterval = std::chrono::milliseconds(80);  // ~12.5Hz上限

/// 一个websocket客户端连接。v1只服务单个客户端：新连接进来就顶替旧的。
class Session : public std::enable_shared_from_this<Session> {
   public:
    Session(tcp::socket socket) : ws_(std::move(socket)) {}

    void Start(std::function<void()> on_established) {
        ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
        ws_.binary(true);
        auto self = shared_from_this();
        ws_.async_accept([self, on_established](beast::error_code ec) {
            if (ec) {
                LOG(WARNING) << "UiWireServer: websocket handshake failed: " << ec.message();
                return;
            }
            on_established();
            self->DoRead();
        });
    }

    /// 从任意线程调用都安全：真正的入队+发送通过post搬到ws_的executor上执行。
    void Send(std::shared_ptr<std::vector<uint8_t>> msg) {
        auto self = shared_from_this();
        net::post(ws_.get_executor(), [self, msg]() {
            bool writing = !self->queue_.empty();
            self->queue_.push_back(msg);
            if (!writing) {
                self->DoWrite();
            }
        });
    }

    bool closed() const { return closed_.load(); }

   private:
    void DoWrite() {
        auto self = shared_from_this();
        ws_.async_write(net::buffer(*queue_.front()), [self](beast::error_code ec, size_t) {
            if (ec) {
                self->closed_.store(true);
                return;
            }
            self->queue_.pop_front();
            if (!self->queue_.empty()) {
                self->DoWrite();
            }
        });
    }

    // 目前不处理客户端发来的任何内容，v1协议是纯单向(server->client)推送；
    // 持续读取只是为了让Beast能感知到连接关闭/处理ping-pong。
    void DoRead() {
        auto self = shared_from_this();
        ws_.async_read(buffer_, [self](beast::error_code ec, size_t) {
            if (ec) {
                self->closed_.store(true);
                return;
            }
            self->buffer_.consume(self->buffer_.size());
            self->DoRead();
        });
    }

    websocket::stream<tcp::socket> ws_;
    beast::flat_buffer buffer_;
    std::deque<std::shared_ptr<std::vector<uint8_t>>> queue_;
    std::atomic<bool> closed_{false};
};

}  // namespace

struct UiWireServer::Impl {
    explicit Impl(uint16_t port) : port(port), acceptor(ioc), keyframe_sync_timer(ioc) {}

    uint16_t port;
    net::io_context ioc;
    tcp::acceptor acceptor;
    net::steady_timer keyframe_sync_timer;
    std::thread io_thread;
    std::atomic<bool> stop{false};

    std::mutex session_mtx;
    std::shared_ptr<Session> session;

    // 以下状态受state_mtx保护，因为Update*可能同时被多个原生线程调用（见调研：DR线程+lidar_loc线程
    // +UpdateMapThread都会碰PangolinWindow的接口），而且新客户端连接时(在io线程上)要读一份快照重放。
    std::mutex state_mtx;
    std::map<int, std::vector<UiPoint>> global_chunk_cache;   // 只在第一次见到某id时写入(create-once)
    std::map<int, std::vector<UiPoint>> dynamic_chunk_cache;  // 每次UpdatePointCloudDynamic都整份刷新
    bool have_frontend_pose = false;
    SE3 last_frontend_pose;
    std::chrono::steady_clock::time_point last_frontend_pose_sent{};
    std::chrono::steady_clock::time_point last_scan_sent{};
    // 关键帧的shared_ptr本身留着（不只是拷贝一份pose出来）：闭环优化会通过这些shared_ptr别名
    // 就地改pose，定时器要能重新读到最新值，见ScheduleKeyframeSync的注释。
    std::vector<std::shared_ptr<Keyframe>> keyframes;

    bool Init();
    void DoAccept();
    void SendSnapshotTo(const std::shared_ptr<Session>& s);
    void Broadcast(std::vector<uint8_t> msg);
    void ScheduleKeyframeSync();
    void SendKeyframePoseSync();
    void SendFrontendPose(const SE3& pose, bool is_traj_point);
};

bool UiWireServer::Impl::Init() {
    beast::error_code ec;
    tcp::endpoint endpoint(tcp::v4(), port);

    acceptor.open(endpoint.protocol(), ec);
    if (ec) {
        LOG(ERROR) << "UiWireServer: open failed: " << ec.message();
        return false;
    }
    acceptor.set_option(net::socket_base::reuse_address(true), ec);
    acceptor.bind(endpoint, ec);
    if (ec) {
        LOG(ERROR) << "UiWireServer: bind(" << port << ") failed: " << ec.message();
        return false;
    }
    acceptor.listen(net::socket_base::max_listen_connections, ec);
    if (ec) {
        LOG(ERROR) << "UiWireServer: listen failed: " << ec.message();
        return false;
    }

    DoAccept();
    ScheduleKeyframeSync();

    io_thread = std::thread([this]() { ioc.run(); });
    LOG(INFO) << "UiWireServer: listening on port " << port;
    return true;
}

void UiWireServer::Impl::DoAccept() {
    acceptor.async_accept([this](beast::error_code ec, tcp::socket socket) {
        if (!ec) {
            auto s = std::make_shared<Session>(std::move(socket));
            s->Start([this, s]() {
                {
                    std::lock_guard<std::mutex> lock(session_mtx);
                    session = s;
                }
                LOG(INFO) << "UiWireServer: client connected, sending snapshot";
                SendSnapshotTo(s);
            });
        } else {
            LOG(WARNING) << "UiWireServer: accept failed: " << ec.message();
        }
        if (!stop.load()) {
            DoAccept();
        }
    });
}

void UiWireServer::Impl::SendSnapshotTo(const std::shared_ptr<Session>& s) {
    std::vector<std::pair<int, std::vector<UiPoint>>> global_snapshot, dynamic_snapshot;
    bool have_pose = false;
    SE3 pose;
    std::vector<std::pair<uint64_t, SE3>> kf_poses;
    {
        std::lock_guard<std::mutex> lock(state_mtx);
        global_snapshot.assign(global_chunk_cache.begin(), global_chunk_cache.end());
        dynamic_snapshot.assign(dynamic_chunk_cache.begin(), dynamic_chunk_cache.end());
        have_pose = have_frontend_pose;
        pose = last_frontend_pose;
        kf_poses.reserve(keyframes.size());
        for (auto& kf : keyframes) {
            kf_poses.emplace_back(static_cast<uint64_t>(kf->GetID()), kf->GetOptPose());
        }
    }

    auto send_chunks = [&](const std::vector<std::pair<int, std::vector<UiPoint>>>& chunks, uint8_t is_dynamic) {
        if (chunks.empty()) return;
        for (auto& [id, pts] : chunks) {
            wire::WireWriter w(wire::MsgType::kMapChunk);
            w.WriteI32(id);
            w.WritePod<uint8_t>(is_dynamic);
            w.WritePoints(pts);
            s->Send(std::make_shared<std::vector<uint8_t>>(w.Finish()));
        }
        wire::WireWriter aw(wire::MsgType::kMapChunkActiveSet);
        aw.WritePod<uint8_t>(is_dynamic);
        aw.WriteU32(static_cast<uint32_t>(chunks.size()));
        for (auto& [id, pts] : chunks) aw.WriteI32(id);
        s->Send(std::make_shared<std::vector<uint8_t>>(aw.Finish()));
    };
    send_chunks(global_snapshot, 0);
    send_chunks(dynamic_snapshot, 1);

    if (have_pose) {
        wire::WireWriter w(wire::MsgType::kFrontendPose);
        w.WritePose(pose);
        w.WritePod<uint8_t>(1);
        s->Send(std::make_shared<std::vector<uint8_t>>(w.Finish()));
    }

    // 注意：迟到的客户端只补历史关键帧的位姿(轨迹形状)，不重发它们各自的点云——
    // 长时间运行下关键帧可能有几千个，每个几十到上百KB，全量重放对新连接代价太大。
    // 这是v1的一个已知取舍，见wire协议scope文档里的"迟到者"开放问题。
    if (!kf_poses.empty()) {
        wire::WireWriter w(wire::MsgType::kKeyframePoseSync);
        w.WriteU32(static_cast<uint32_t>(kf_poses.size()));
        for (auto& [id, p] : kf_poses) {
            w.WriteU64(id);
            w.WritePose(p);
        }
        s->Send(std::make_shared<std::vector<uint8_t>>(w.Finish()));
    }
}

void UiWireServer::Impl::Broadcast(std::vector<uint8_t> msg) {
    std::shared_ptr<Session> s;
    {
        std::lock_guard<std::mutex> lock(session_mtx);
        s = session;
    }
    if (s && !s->closed()) {
        s->Send(std::make_shared<std::vector<uint8_t>>(std::move(msg)));
    }
}

void UiWireServer::Impl::ScheduleKeyframeSync() {
    keyframe_sync_timer.expires_after(kKeyframeSyncInterval);
    keyframe_sync_timer.async_wait([this](beast::error_code ec) {
        if (ec || stop.load()) return;
        SendKeyframePoseSync();
        ScheduleKeyframeSync();
    });
}

void UiWireServer::Impl::SendKeyframePoseSync() {
    // 闭环优化通过共享的Keyframe::Ptr别名就地改pose_opt_，不一定伴随UpdateKF调用，
    // 所以要定时重新读取所有已知关键帧的当前pose，而不是只在UpdateKF时读一次就当作永久值。
    std::vector<std::pair<uint64_t, SE3>> poses;
    {
        std::lock_guard<std::mutex> lock(state_mtx);
        poses.reserve(keyframes.size());
        for (auto& kf : keyframes) {
            poses.emplace_back(static_cast<uint64_t>(kf->GetID()), kf->GetOptPose());
        }
    }
    if (poses.empty()) return;

    wire::WireWriter w(wire::MsgType::kKeyframePoseSync);
    w.WriteU32(static_cast<uint32_t>(poses.size()));
    for (auto& [id, pose] : poses) {
        w.WriteU64(id);
        w.WritePose(pose);
    }
    Broadcast(w.Finish());
}

UiWireServer::UiWireServer(uint16_t port) : impl_(std::make_unique<Impl>(port)) {}
UiWireServer::~UiWireServer() { Quit(); }

bool UiWireServer::Init() { return impl_->Init(); }

void UiWireServer::Reset(const std::vector<Keyframe::Ptr>& /*keyframes*/) {
    // 没有外部调用点（见调研），保留只是为了跟PangolinWindow接口对齐。
}

void UiWireServer::UpdatePointCloudGlobal(const std::map<int, CloudPtr>& cloud) {
    std::vector<std::pair<int, std::vector<UiPoint>>> to_send;
    std::vector<int> active_ids;
    {
        std::lock_guard<std::mutex> lock(impl_->state_mtx);
        for (const auto& cp : cloud) {
            active_ids.push_back(cp.first);
            // 跟原生端PangolinWindowImpl::UpdateGlobalMap行为一致：静态地图分片只在第一次
            // 见到某id时创建/发送，之后即使内容变了也不会重发（同一分片理论上不会再变）。
            if (impl_->global_chunk_cache.find(cp.first) == impl_->global_chunk_cache.end()) {
                auto pts = ToUiPoints(cp.second);
                to_send.emplace_back(cp.first, pts);
                impl_->global_chunk_cache.emplace(cp.first, std::move(pts));
            }
        }
        for (auto it = impl_->global_chunk_cache.begin(); it != impl_->global_chunk_cache.end();) {
            if (cloud.find(it->first) == cloud.end()) {
                it = impl_->global_chunk_cache.erase(it);
            } else {
                ++it;
            }
        }
    }

    for (auto& [id, pts] : to_send) {
        wire::WireWriter w(wire::MsgType::kMapChunk);
        w.WriteI32(id);
        w.WritePod<uint8_t>(0);
        w.WritePoints(pts);
        impl_->Broadcast(w.Finish());
    }
    wire::WireWriter aw(wire::MsgType::kMapChunkActiveSet);
    aw.WritePod<uint8_t>(0);
    aw.WriteU32(static_cast<uint32_t>(active_ids.size()));
    for (int id : active_ids) aw.WriteI32(id);
    impl_->Broadcast(aw.Finish());
}

void UiWireServer::UpdatePointCloudDynamic(const std::map<int, CloudPtr>& cloud) {
    std::vector<int> active_ids;
    {
        std::lock_guard<std::mutex> lock(impl_->state_mtx);
        impl_->dynamic_chunk_cache.clear();
        for (const auto& cp : cloud) {
            active_ids.push_back(cp.first);
            impl_->dynamic_chunk_cache.emplace(cp.first, ToUiPoints(cp.second));
        }
    }

    // 跟原生端UpdateDynamicMap行为一致：动态分片"存在也要更新"，每次都整份重发。
    for (const auto& cp : cloud) {
        wire::WireWriter w(wire::MsgType::kMapChunk);
        w.WriteI32(cp.first);
        w.WritePod<uint8_t>(1);
        w.WritePoints(ToUiPoints(cp.second));
        impl_->Broadcast(w.Finish());
    }
    wire::WireWriter aw(wire::MsgType::kMapChunkActiveSet);
    aw.WritePod<uint8_t>(1);
    aw.WriteU32(static_cast<uint32_t>(active_ids.size()));
    for (int id : active_ids) aw.WriteI32(id);
    impl_->Broadcast(aw.Finish());
}

void UiWireServer::Impl::SendFrontendPose(const SE3& pose, bool is_traj_point) {
    auto now = std::chrono::steady_clock::now();
    bool send;
    {
        std::lock_guard<std::mutex> lock(state_mtx);
        have_frontend_pose = true;
        last_frontend_pose = pose;
        // 之前这里对轨迹打点(is_traj_point，来自UpdateNavState)完全不限速，理由是"丢了会让画出来的
        // 轨迹出现缺口"——但实测发现UpdateNavState其实在ProcessIMU()里按IMU频率调用(laser_mapping.cc)，
        // 离线回放(run_slam_offline)没有限速、又跑得比实时快好几倍时，这个"零节流"路径能打到
        // 上千条/秒，把浏览器主线程的websocket消息处理占满，连requestAnimationFrame都排不上号——
        // wasm端一直在正确解码消息(计数器验证过)，但画面因为从来没机会真正合成新的一帧，
        // 表现得像是"卡住不动"，而不是单纯变慢。kFrontendPoseMinInterval(~30Hz)对任何人眼可感的
        // 轨迹平滑度都绰绰有余，不会出现真正意义上的"缺口"，所以两种调用统一走同一个节流。
        send = (now - last_frontend_pose_sent) >= kFrontendPoseMinInterval;
        if (send) last_frontend_pose_sent = now;
    }
    if (!send) return;

    wire::WireWriter w(wire::MsgType::kFrontendPose);
    w.WritePose(pose);
    w.WritePod<uint8_t>(is_traj_point ? 1 : 0);
    Broadcast(w.Finish());
}

void UiWireServer::UpdateNavState(const NavState& state) { impl_->SendFrontendPose(state.GetPose(), true); }

void UiWireServer::UpdateRecentPose(const SE3& pose) { impl_->SendFrontendPose(pose, false); }

void UiWireServer::UpdatePredictPose(const SE3& /*pose*/) {
    // 目前没有实际调用点（见调研），保留只是为了接口对齐。
}

void UiWireServer::UpdateScan(CloudPtr cloud, const SE3& pose) {
    auto now = std::chrono::steady_clock::now();
    bool send;
    {
        std::lock_guard<std::mutex> lock(impl_->state_mtx);
        send = (now - impl_->last_scan_sent) >= kScanMinInterval;
        if (send) impl_->last_scan_sent = now;
    }
    if (!send) return;

    wire::WireWriter w(wire::MsgType::kScan);
    w.WritePose(pose);
    w.WritePoints(ToUiPoints(cloud));
    impl_->Broadcast(w.Finish());
}

void UiWireServer::UpdateKF(std::shared_ptr<Keyframe> kf) {
    {
        std::lock_guard<std::mutex> lock(impl_->state_mtx);
        impl_->keyframes.push_back(kf);
    }

    wire::WireWriter w(wire::MsgType::kKeyframeAppend);
    w.WriteU64(static_cast<uint64_t>(kf->GetID()));
    w.WritePose(kf->GetOptPose());
    w.WritePoints(ToUiPoints(kf->GetCloud()));
    impl_->Broadcast(w.Finish());
}

void UiWireServer::Quit() {
    if (impl_->stop.exchange(true)) return;  // 已经Quit过
    impl_->ioc.stop();
    if (impl_->io_thread.joinable()) {
        impl_->io_thread.join();
    }
}

bool UiWireServer::ShouldQuit() { return impl_->stop.load(); }

void UiWireServer::SetTImuLidar(const SE3& /*T_imu_lidar*/) {
    // 渲染/wire协议不需要这个外参，保留只是为了接口对齐。
}

void UiWireServer::SetCurrentScanSize(int /*current_scan_size*/) {
    // scans_队列的上限是wasm端PangolinUiScene自己的渲染细节，原生端不需要知道。
}

}  // namespace lightning::ui
