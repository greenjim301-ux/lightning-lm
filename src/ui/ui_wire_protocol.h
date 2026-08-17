#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "common/eigen_types.h"
#include "ui/ui_cloud.h"

/// UiWireServer(原生端，src/ui/ui_wire_server.h)和wasm端UI之间的二进制线协议。
/// 故意跟PCL/rclcpp/Boost都不挂钩（只用UiPoint/SE3/std），这样wasm客户端也能直接#include这个头。
/// 只支持小端(little-endian)平台——目前的两端(x86_64原生 + wasm32)都是小端，不做字节序转换。
namespace lightning::ui::wire {

enum class MsgType : uint8_t {
    kFrontendPose = 1,       // 前端(滤波器)位姿：驱动跟随相机 + 前端小车 + 前端轨迹打点
    kScan = 2,                // 当前scan：驱动PushCurrentScan（后端小车+后端轨迹+当前scan点云）
    kMapChunk = 3,             // 全局/动态地图的一个分片
    kMapChunkActiveSet = 4,    // 当前有效的分片id集合（用于客户端按缺席剔除，镜像原生端UpdateGlobalMap的裁剪逻辑）
    kKeyframeAppend = 5,       // 新增一个关键帧（位姿+点云）
    kKeyframePoseSync = 6,     // 周期性重发所有已知关键帧的位姿（闭环优化会就地改已有关键帧的pose，不能只靠append感知）
};

/// SE3在线上的表示：平移(3xdouble) + 四元数xyzw(4xdouble)，共56字节，与Sophus::SE3d的内部表示一一对应。
struct WirePose {
    double tx = 0, ty = 0, tz = 0;
    double qx = 0, qy = 0, qz = 0, qw = 1;
};

inline WirePose ToWirePose(const SE3& pose) {
    WirePose w;
    const auto& t = pose.translation();
    const auto& q = pose.unit_quaternion();
    w.tx = t.x();
    w.ty = t.y();
    w.tz = t.z();
    w.qx = q.x();
    w.qy = q.y();
    w.qz = q.z();
    w.qw = q.w();
    return w;
}

inline SE3 FromWirePose(const WirePose& w) {
    return SE3(Eigen::Quaterniond(w.qw, w.qx, w.qy, w.qz), Eigen::Vector3d(w.tx, w.ty, w.tz));
}

/// 顺序构造一条消息：Finish()之前的所有Write*都会追加到内部buffer，Finish()补上length字段并返回。
class WireWriter {
   public:
    explicit WireWriter(MsgType type) : type_(type) {
        // 预留header：1字节type + 4字节length(payload长度，不含header本身)
        buf_.resize(5);
    }

    template <typename T>
    void WritePod(const T& v) {
        static_assert(std::is_trivially_copyable<T>::value, "WritePod requires a trivially copyable type");
        const auto* p = reinterpret_cast<const uint8_t*>(&v);
        buf_.insert(buf_.end(), p, p + sizeof(T));
    }

    void WriteU32(uint32_t v) { WritePod(v); }
    void WriteI32(int32_t v) { WritePod(v); }
    void WriteU64(uint64_t v) { WritePod(v); }
    void WritePose(const SE3& pose) { WritePod(ToWirePose(pose)); }

    /// count(u32) + 紧凑排列的[x,y,z,intensity]*count（16字节/点，不是PCL那种带padding的布局）
    void WritePoints(const std::vector<UiPoint>& pts) {
        WriteU32(static_cast<uint32_t>(pts.size()));
        if (!pts.empty()) {
            const auto* p = reinterpret_cast<const uint8_t*>(pts.data());
            buf_.insert(buf_.end(), p, p + pts.size() * sizeof(UiPoint));
        }
    }

    std::vector<uint8_t> Finish() {
        buf_[0] = static_cast<uint8_t>(type_);
        uint32_t payload_len = static_cast<uint32_t>(buf_.size() - 5);
        std::memcpy(buf_.data() + 1, &payload_len, sizeof(uint32_t));
        return std::move(buf_);
    }

   private:
    MsgType type_;
    std::vector<uint8_t> buf_;
};

/// 顺序解析一条已经按帧收到的完整消息（一条websocket二进制帧==一条完整消息，不跨帧拼包）。
class WireReader {
   public:
    WireReader(const uint8_t* data, size_t size) : data_(data), size_(size) {
        if (size_ < 5) {
            throw std::runtime_error("wire message too short for header");
        }
        type_ = static_cast<MsgType>(data_[0]);
        std::memcpy(&payload_len_, data_ + 1, sizeof(uint32_t));
        pos_ = 5;
        if (5 + static_cast<size_t>(payload_len_) != size_) {
            throw std::runtime_error("wire message length mismatch");
        }
    }

    MsgType type() const { return type_; }

    template <typename T>
    T ReadPod() {
        static_assert(std::is_trivially_copyable<T>::value, "ReadPod requires a trivially copyable type");
        if (pos_ + sizeof(T) > size_) {
            throw std::runtime_error("wire message truncated");
        }
        T v;
        std::memcpy(&v, data_ + pos_, sizeof(T));
        pos_ += sizeof(T);
        return v;
    }

    uint32_t ReadU32() { return ReadPod<uint32_t>(); }
    int32_t ReadI32() { return ReadPod<int32_t>(); }
    uint64_t ReadU64() { return ReadPod<uint64_t>(); }
    SE3 ReadPose() { return FromWirePose(ReadPod<WirePose>()); }

    std::vector<UiPoint> ReadPoints() {
        uint32_t count = ReadU32();
        size_t bytes = static_cast<size_t>(count) * sizeof(UiPoint);
        if (pos_ + bytes > size_) {
            throw std::runtime_error("wire message truncated (points)");
        }
        std::vector<UiPoint> pts(count);
        if (count > 0) {
            std::memcpy(pts.data(), data_ + pos_, bytes);
        }
        pos_ += bytes;
        return pts;
    }

    bool AtEnd() const { return pos_ == size_; }

   private:
    const uint8_t* data_;
    size_t size_;
    size_t pos_ = 0;
    MsgType type_;
    uint32_t payload_len_ = 0;
};

}  // namespace lightning::ui::wire
