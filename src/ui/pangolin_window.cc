#include "ui/pangolin_window.h"

#include <rerun.hpp>

#include <algorithm>
#include <glog/logging.h>

#include "core/lightning_math.hpp"

namespace lightning::ui {

namespace {

rerun::Position3D ToRerunPos(const Vec3d& p) { return {float(p.x()), float(p.y()), float(p.z())}; }
rerun::Position3D ToRerunPos(const PointType& p) { return {p.x, p.y, p.z}; }
/// LineStrip3D wants Collection<Vec3D>, not Collection<Position3D> — see components/line_strip3d.hpp
rerun::Vec3D ToRerunVec3D(const Vec3d& p) { return {float(p.x()), float(p.y()), float(p.z())}; }

/// 高度到颜色的简易映射，近似复刻原 UiCloud::HEIGHT_COLOR 的视觉效果
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

PangolinWindow::PangolinWindow() { rerun_stream_ = std::make_unique<rerun::RecordingStream>("lightning_lm"); }

PangolinWindow::~PangolinWindow() { Quit(); }

bool PangolinWindow::Init() {
    // React app connects via `rrd="rerun+http://<host>:9876/proxy"` (@rerun-io/web-viewer-react).
    // Tighten cors_allow_origins to the actual frontend origin before shipping this beyond local dev.
    auto serve_result = rerun_stream_->serve_grpc("0.0.0.0", 9876, "1GiB", rerun::PlaybackBehavior::OldestFirst,
                                                  {"*"});
    if (serve_result.is_err()) {
        LOG(ERROR) << "rerun serve_grpc failed, web viewer will not be available: " << serve_result.error.description;
        return false;
    }

    rerun_stream_->log_static("world", rerun::ViewCoordinates::RFU);  // X=Right, Y=Forward, Z=Up
    return true;
}

void PangolinWindow::Reset(const std::vector<Keyframe::Ptr>& keyframes) {
    if (keyframes.empty()) {
        return;
    }

    // 按关键帧原始顺序重放，使 rerun 播放条可以像回放录像一样逐帧查看建图过程
    std::vector<rerun::Vec3D> loop_strip;
    loop_strip.reserve(keyframes.size());
    for (size_t i = 0; i < keyframes.size(); ++i) {
        rerun_stream_->set_time_sequence("keyframe", static_cast<int64_t>(i));
        const SE3 pose = keyframes[i]->GetOptPose();
        loop_strip.emplace_back(ToRerunVec3D(pose.translation()));
        rerun_stream_->log("world/backend/trajectory", rerun::Points3D({ToRerunPos(pose.translation())}));
    }
    rerun_stream_->log("world/loop/trajectory",
                       rerun::LineStrips3D({loop_strip}).with_colors(rerun::Color(128, 0, 128)));

    // 仅重放最近 max_size_of_current_scan_ 个关键帧的点云，避免一次性推送过多历史点云
    const size_t begin = keyframes.size() > static_cast<size_t>(max_size_of_current_scan_)
                             ? keyframes.size() - static_cast<size_t>(max_size_of_current_scan_)
                             : 0;
    for (size_t i = begin; i < keyframes.size(); ++i) {
        const auto& kf = keyframes[i];
        rerun_stream_->set_time_sequence("keyframe", static_cast<int64_t>(i));

        CloudPtr voxeled = math::VoxelGrid(std::make_shared<PointCloudType>(*kf->GetCloud()), 0.5);
        std::vector<rerun::Position3D> pts;
        std::vector<rerun::Color> colors;
        pts.reserve(voxeled->size());
        colors.reserve(voxeled->size());
        for (const auto& p : *voxeled) {
            pts.emplace_back(ToRerunPos(p));
            colors.emplace_back(HeightToColor(p.z));
        }

        rerun_stream_->log("world/scan/current", ToRerunTransform(kf->GetOptPose()));
        rerun_stream_->log("world/scan/current/points", rerun::Points3D(pts).with_colors(colors));
    }

    std::lock_guard<std::mutex> lock(mtx_keyframes_);
    all_keyframes_.assign(keyframes.begin(), keyframes.end());
}

void PangolinWindow::UpdatePointCloudGlobal(const std::map<int, CloudPtr>& cloud) {
    // 全局地图点云已经是world系坐标，无需再乘pose
    for (const auto& [submap_id, pc] : cloud) {
        std::vector<rerun::Position3D> pts;
        pts.reserve(pc->size());
        for (const auto& p : *pc) pts.emplace_back(ToRerunPos(p));
        rerun_stream_->log("world/map/" + std::to_string(submap_id),
                           rerun::Points3D(pts).with_colors(rerun::Color(150, 150, 150)));
    }
}

void PangolinWindow::UpdatePointCloudDynamic(const std::map<int, CloudPtr>& cloud) {
    for (const auto& [submap_id, pc] : cloud) {
        std::vector<rerun::Position3D> pts;
        pts.reserve(pc->size());
        for (const auto& p : *pc) pts.emplace_back(ToRerunPos(p));
        rerun_stream_->log("world/dynamic/" + std::to_string(submap_id),
                           rerun::Points3D(pts).with_colors(rerun::Color(0, 51, 255)));
    }
}

void PangolinWindow::UpdateNavState(const NavState& state) {
    rerun_stream_->log("state/vel/x", rerun::Scalars(state.GetVel().x()));
    rerun_stream_->log("state/vel/y", rerun::Scalars(state.GetVel().y()));
    rerun_stream_->log("state/vel/z", rerun::Scalars(state.GetVel().z()));
    rerun_stream_->log("state/bias_acc/x", rerun::Scalars(state.Getba().x()));
    rerun_stream_->log("state/bias_acc/y", rerun::Scalars(state.Getba().y()));
    rerun_stream_->log("state/bias_acc/z", rerun::Scalars(state.Getba().z()));
    rerun_stream_->log("state/confidence", rerun::Scalars(state.confidence_));
    LogFrontendPose(state.GetPose());
}

void PangolinWindow::UpdateRecentPose(const SE3& pose) { LogFrontendPose(pose); }

void PangolinWindow::UpdatePredictPose(const SE3& pose) {
    rerun_stream_->log("world/predicted/car", ToRerunTransform(pose));
}

void PangolinWindow::UpdateScan(CloudPtr cloud, const SE3& pose) {
    // cloud 是lidar系(lP)坐标，用Transform3D承载pose，让rerun viewer去做世界系变换
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
}

void PangolinWindow::UpdateKF(std::shared_ptr<Keyframe> kf) {
    std::lock_guard<std::mutex> lock(mtx_keyframes_);
    all_keyframes_.emplace_back(kf);

    // 闭环轨迹：连接所有关键帧优化后的位置
    std::vector<rerun::Vec3D> strip;
    strip.reserve(all_keyframes_.size());
    for (const auto& k : all_keyframes_) strip.emplace_back(ToRerunVec3D(k->GetOptPose().translation()));
    rerun_stream_->log("world/loop/trajectory", rerun::LineStrips3D({strip}).with_colors(rerun::Color(128, 0, 128)));
}

void PangolinWindow::Quit() {}

bool PangolinWindow::ShouldQuit() { return false; }

void PangolinWindow::SetTImuLidar(const SE3& /*T_imu_lidar*/) {}

void PangolinWindow::SetCurrentScanSize(int current_scan_size) { max_size_of_current_scan_ = current_scan_size; }

void PangolinWindow::LogFrontendPose(const SE3& pose) {
    rerun_stream_->log("world/frontend/car", ToRerunTransform(pose));
    rerun_stream_->log("world/frontend/trajectory", rerun::Points3D({ToRerunPos(pose.translation())}));

    // 平移跟随锚点：只取XY，忽略Z与朝向，对应原Pangolin Follow()的行为（不随车辆转向旋转视角）。
    // 供blueprint的chase-cam视图(origin="world/follow_anchor")使用。
    const Vec3d t = pose.translation();
    rerun_stream_->log("world/follow_anchor",
                       rerun::Transform3D::from_translation(rerun::Vec3D{float(t.x()), float(t.y()), 0.f}));
}

}  // namespace lightning::ui
