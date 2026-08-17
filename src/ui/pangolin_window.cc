#include "ui/pangolin_window_impl.h"

#ifdef LIGHTNING_ENABLE_RERUN
#include <rerun.hpp>

#include <algorithm>
#include <glog/logging.h>
#endif

namespace lightning::ui {

#ifdef LIGHTNING_ENABLE_RERUN
namespace {

rerun::Position3D ToRerunPos(const Vec3d& p) { return {float(p.x()), float(p.y()), float(p.z())}; }
rerun::Position3D ToRerunPos(const PointType& p) { return {p.x, p.y, p.z}; }
/// LineStrip3D wants Collection<Vec3D>, not Collection<Position3D> — see components/line_strip3d.hpp
rerun::Vec3D ToRerunVec3D(const Vec3d& p) { return {float(p.x()), float(p.y()), float(p.z())}; }

/// 高度到颜色的简易映射，近似复刻 UiCloud::HEIGHT_COLOR 的视觉效果（见 ui_cloud.cc）
rerun::Color HeightToColor(float z) {
    float t = std::clamp(z / 5.0f + 0.5f, 0.0f, 1.0f);
    return rerun::Color(static_cast<uint8_t>(255 * t), static_cast<uint8_t>(255 * (1.0f - std::abs(t - 0.5f) * 2.0f)),
                        static_cast<uint8_t>(255 * (1.0f - t)));
}

// Verified against rerun_cpp_sdk 0.36.0 source (archetypes/transform3d.hpp, datatypes/quaternion.hpp)
rerun::Transform3D ToRerunTransform(const SE3& pose) {
    const Vec3d t = pose.translation();
    const Eigen::Quaterniond q = pose.unit_quaternion();
    return rerun::Transform3D::from_translation_rotation(
        ToRerunVec3D(t), rerun::Quaternion::from_xyzw(float(q.x()), float(q.y()), float(q.z()), float(q.w())));
}

}  // namespace
#endif

PangolinWindow::PangolinWindow() {
    impl_ = std::make_shared<PangolinWindowImpl>();

#ifdef LIGHTNING_ENABLE_RERUN
    rerun_stream_ = std::make_unique<rerun::RecordingStream>("lightning_lm");
    // React app connects via `rrd="rerun+http://<host>:9876/proxy"` (see @rerun-io/web-viewer-react integration).
    // Tighten cors_allow_origins to the actual frontend origin before shipping this beyond local dev.
    auto serve_result = rerun_stream_->serve_grpc("0.0.0.0", 9876, "1GiB", rerun::PlaybackBehavior::OldestFirst,
                                                  {"*"});
    if (serve_result.is_err()) {
        LOG(WARNING) << "rerun serve_grpc failed, web viewer will not be available: "
                    << serve_result.error.description;
    }
    rerun_stream_->log_static("world", rerun::ViewCoordinates::RFU);  // X=Right, Y=Forward, Z=Up
#endif
}
PangolinWindow::~PangolinWindow() { Quit(); }

bool PangolinWindow::Init() {
    impl_->cloud_global_need_update_.store(false);
    impl_->kf_result_need_update_.store(false);
    impl_->lidarloc_need_update_.store(false);
    impl_->current_scan_need_update_.store(false);

    bool inited = impl_->Init();
    // 创建渲染线程
    if (inited) {
        impl_->render_thread_ = std::thread([this]() { impl_->Render(); });
    }
    return inited;
}

void PangolinWindow::Reset(const std::vector<Keyframe::Ptr>& keyframes) {
    impl_->Reset(keyframes);

#ifdef LIGHTNING_ENABLE_RERUN
    // 按关键帧原始顺序重放，使 rerun 播放条可以像 Pangolin 的 Step/Play speed 菜单一样逐帧回放
    std::vector<rerun::Vec3D> loop_strip;
    loop_strip.reserve(keyframes.size());
    for (size_t i = 0; i < keyframes.size(); ++i) {
        rerun_stream_->set_time_sequence("keyframe", static_cast<int64_t>(i));
        const Vec3d t = keyframes[i]->GetOptPose().translation();
        loop_strip.emplace_back(ToRerunVec3D(t));
        rerun_stream_->log("world/backend/trajectory", rerun::Points3D({ToRerunPos(t)}));
    }
    rerun_stream_->log("world/loop/trajectory",
                       rerun::LineStrips3D({loop_strip}).with_colors(rerun::Color(128, 0, 128)));
#endif
}

void PangolinWindow::Quit() {
    if (impl_->render_thread_.joinable()) {
        impl_->exit_flag_.store(true);
        impl_->render_thread_.join();
    }
    impl_->DeInit();
}

void PangolinWindow::UpdatePointCloudGlobal(const std::map<int, CloudPtr>& cloud) {
    std::lock_guard<std::mutex> lock(impl_->mtx_map_cloud_);
    impl_->cloud_global_map_ = cloud;
    impl_->cloud_global_need_update_.store(true);

#ifdef LIGHTNING_ENABLE_RERUN
    // 全局地图点云已经是world系坐标（见 pangolin_window_impl.cc:86 SetCloud(cp.second, SE3())），无需再乘pose
    for (const auto& [submap_id, pc] : cloud) {
        std::vector<rerun::Position3D> pts;
        pts.reserve(pc->size());
        for (const auto& p : *pc) pts.emplace_back(ToRerunPos(p));
        rerun_stream_->log("world/map/" + std::to_string(submap_id),
                           rerun::Points3D(pts).with_colors(rerun::Color(150, 150, 150)));
    }
#endif
}

void PangolinWindow::UpdatePointCloudDynamic(const std::map<int, CloudPtr>& cloud) {
    std::unique_lock<std::mutex> lock(impl_->mtx_map_cloud_);
    impl_->cloud_dynamic_map_.clear();  // need deep copy

    for (auto& cp : cloud) {
        CloudPtr c(new PointCloudType());
        *c = *cp.second;
        impl_->cloud_dynamic_map_.emplace(cp.first, c);
    }

    for (auto iter = impl_->cloud_dynamic_map_.begin(); iter != impl_->cloud_dynamic_map_.end();) {
        if (cloud.find(iter->first) == cloud.end()) {
            iter = impl_->cloud_dynamic_map_.erase(iter);
        } else {
            iter++;
        }
    }

    impl_->cloud_dynamic_need_update_.store(true);

#ifdef LIGHTNING_ENABLE_RERUN
    for (const auto& [submap_id, pc] : cloud) {
        std::vector<rerun::Position3D> pts;
        pts.reserve(pc->size());
        for (const auto& p : *pc) pts.emplace_back(ToRerunPos(p));
        rerun_stream_->log("world/dynamic/" + std::to_string(submap_id),
                           rerun::Points3D(pts).with_colors(rerun::Color(0, 51, 255)));
    }
#endif
}

void PangolinWindow::UpdateNavState(const NavState& state) {
    std::unique_lock<std::mutex> lock_lio_res(impl_->mtx_nav_state_);

    impl_->pose_ = state.GetPose();
    impl_->vel_ = state.GetVel();
    impl_->bias_acc_ = state.Getba();
    impl_->bias_gyr_ = state.Getbg();
    impl_->confidence_ = state.confidence_;

    impl_->kf_result_need_update_.store(true);

#ifdef LIGHTNING_ENABLE_RERUN
    rerun_stream_->log("state/vel/x", rerun::Scalars(state.GetVel().x()));
    rerun_stream_->log("state/vel/y", rerun::Scalars(state.GetVel().y()));
    rerun_stream_->log("state/vel/z", rerun::Scalars(state.GetVel().z()));
    rerun_stream_->log("state/bias_acc/x", rerun::Scalars(state.Getba().x()));
    rerun_stream_->log("state/bias_acc/y", rerun::Scalars(state.Getba().y()));
    rerun_stream_->log("state/bias_acc/z", rerun::Scalars(state.Getba().z()));
    rerun_stream_->log("state/confidence", rerun::Scalars(state.confidence_));
    LogFrontendPose(state.GetPose());
#endif
}

void PangolinWindow::UpdateRecentPose(const SE3& pose) {
    std::lock_guard<std::mutex> lock(impl_->mtx_nav_state_);
    impl_->newest_frontend_pose_ = pose;

#ifdef LIGHTNING_ENABLE_RERUN
    LogFrontendPose(pose);
#endif
}

void PangolinWindow::UpdatePredictPose(const SE3& pose) {
    UL lock(impl_->mtx_nav_state_);
    impl_->predicted_pose_ = pose;
}

void PangolinWindow::UpdateScan(CloudPtr cloud, const SE3& pose) {
    std::lock_guard<std::mutex> lock(impl_->mtx_current_scan_);
    std::lock_guard<std::mutex> lock2(impl_->mtx_nav_state_);

    *impl_->current_scan_ = *cloud;  // need deep copy
    impl_->current_scan_pose_ = pose;
    impl_->current_scan_need_update_.store(true);

#ifdef LIGHTNING_ENABLE_RERUN
    // current_scan_ 是lidar系(lP)坐标，用Transform3D承载pose，让rerun viewer去做世界系变换
    // （对应 UiCloud::SetCloud 中 pose_l * cloud->points[id] 的手动变换）
    rerun_stream_->log("world/scan/current", ToRerunTransform(pose));

    std::vector<rerun::Position3D> pts;
    std::vector<rerun::Color> colors;
    pts.reserve(cloud->size());
    colors.reserve(cloud->size());
    for (const auto& p : *cloud) {
        pts.emplace_back(ToRerunPos(p));
        colors.emplace_back(HeightToColor(p.z));
    }
    rerun_stream_->log("world/scan/current/points", rerun::Points3D(pts).with_colors(colors));

    rerun_stream_->log("world/backend/car", ToRerunTransform(pose));
    rerun_stream_->log("world/backend/trajectory", rerun::Points3D({ToRerunPos(pose.translation())}));
#endif
}

void PangolinWindow::UpdateKF(std::shared_ptr<Keyframe> kf) {
    UL lock(impl_->mtx_current_scan_);
    impl_->all_keyframes_.emplace_back(kf);

#ifdef LIGHTNING_ENABLE_RERUN
    // 闭环轨迹（对应 pangolin_window_impl.cc DrawAll() 中 all_keyframes_ 连线的紫色轨迹）
    std::vector<rerun::Vec3D> strip;
    strip.reserve(impl_->all_keyframes_.size());
    for (const auto& k : impl_->all_keyframes_) strip.emplace_back(ToRerunVec3D(k->GetOptPose().translation()));
    rerun_stream_->log("world/loop/trajectory", rerun::LineStrips3D({strip}).with_colors(rerun::Color(128, 0, 128)));
#endif
}

void PangolinWindow::SetCurrentScanSize(int current_scan_size) { impl_->max_size_of_current_scan_ = current_scan_size; }

void PangolinWindow::SetTImuLidar(const SE3& T_imu_lidar) { impl_->T_imu_lidar_ = T_imu_lidar; }

bool PangolinWindow::ShouldQuit() { return pangolin::ShouldQuit(); }

#ifdef LIGHTNING_ENABLE_RERUN
void PangolinWindow::LogFrontendPose(const SE3& pose) {
    rerun_stream_->log("world/frontend/car", ToRerunTransform(pose));
    rerun_stream_->log("world/frontend/trajectory", rerun::Points3D({ToRerunPos(pose.translation())}));
}
#endif

}  // namespace lightning::ui
