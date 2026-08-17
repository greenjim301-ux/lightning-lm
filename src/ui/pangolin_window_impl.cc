#include <string>
#include <thread>

#include "common/options.h"
#include "common/std_types.h"
#include "core/lightning_math.hpp"
#include "ui/pangolin_window_impl.h"

namespace lightning::ui {

namespace {
// UiCloud不依赖PCL（方便以后给Emscripten/web交叉编译），所以PCL点云到UiPoint的转换放在这里——
// 原生端调用点，PCL在这里是现成可用的。等做wire protocol时，wasm那一侧的等价物会是"从网络反序列化"。
std::vector<UiPoint> ToUiPoints(const CloudPtr &cloud) {
    std::vector<UiPoint> pts;
    pts.reserve(cloud->size());
    for (const auto &p : *cloud) {
        pts.push_back(UiPoint{p.x, p.y, p.z, p.intensity});
    }
    return pts;
}
}  // namespace

bool PangolinWindowImpl::Init() {
    current_scan_.reset(new PointCloudType);  // 重置pcl点云指针
    scene_.Init();
    return true;
}

void PangolinWindowImpl::Reset(const std::vector<Keyframe::Ptr> &keyframes) {
    UL lock(mtx_reset_);

    std::vector<SE3> backend_traj;
    backend_traj.reserve(keyframes.size());
    for (const auto &keyframe : keyframes) {
        backend_traj.push_back(keyframe->GetOptPose());
    }

    std::size_t i = keyframes.size() > max_size_of_current_scan_ ? keyframes.size() - max_size_of_current_scan_ : 0;
    std::vector<std::pair<std::vector<UiPoint>, SE3>> recent_scans;
    recent_scans.reserve(keyframes.size() - i);
    for (; i < keyframes.size(); ++i) {
        const auto &keyframe = keyframes.at(i);
        CloudPtr tmp_cloud = std::make_shared<PointCloudType>(*(keyframe->GetCloud()));
        recent_scans.emplace_back(ToUiPoints(math::VoxelGrid(tmp_cloud, 0.5)), keyframe->GetOptPose());
    }

    scene_.Reset(backend_traj, recent_scans);
}

bool PangolinWindowImpl::DeInit() { return true; }

bool PangolinWindowImpl::UpdateGlobalMap() {
    if (!cloud_global_need_update_.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx_map_cloud_);
    std::map<int, std::vector<UiPoint>> clouds_by_id;
    for (const auto &cp : cloud_global_map_) {
        clouds_by_id.emplace(cp.first, ToUiPoints(cp.second));
    }
    scene_.SyncGlobalMap(clouds_by_id);

    cloud_global_need_update_.store(false);
    return true;
}

bool PangolinWindowImpl::UpdateDynamicMap() {
    if (!cloud_dynamic_need_update_.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx_map_cloud_);
    std::map<int, std::vector<UiPoint>> clouds_by_id;
    for (const auto &cp : cloud_dynamic_map_) {
        clouds_by_id.emplace(cp.first, ToUiPoints(cp.second));
    }
    scene_.SyncDynamicMap(clouds_by_id);

    cloud_dynamic_need_update_.store(false);
    return true;
}

bool PangolinWindowImpl::UpdateCurrentScan() {
    UL lock(mtx_current_scan_);
    if (current_scan_ != nullptr && !current_scan_->empty() && current_scan_need_update_) {
        scene_.PushCurrentScan(ToUiPoints(current_scan_), current_scan_pose_);
        current_scan_need_update_.store(false);
    }

    scene_.TrimScanQueue();

    return true;
}

bool PangolinWindowImpl::UpdateState() {
    if (!kf_result_need_update_.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx_nav_state_);
    Vec3d vel_baselink = pose_.so3().inverse() * vel_;

    // 滤波器状态作曲线图
    scene_.LogFilterState(vel_, vel_baselink, bias_acc_, confidence_);

    newest_frontend_pose_ = pose_;
    scene_.AddFrontendTrajPt(newest_frontend_pose_);

    kf_result_need_update_.store(false);
    return false;
}

void PangolinWindowImpl::RenderClouds() {
    UL lock(mtx_reset_);

    // 更新各种推送过来的状态
    UpdateGlobalMap();
    UpdateDynamicMap();
    UpdateState();
    UpdateCurrentScan();

    // newest_frontend_pose_可能被外部（PangolinWindow::UpdateRecentPose）直接改写，每帧都同步给scene
    scene_.SetFrontendPose(newest_frontend_pose_);

    // 闭环后的轨迹：每帧都要重新从all_keyframes_取位姿再传给scene——闭环优化可能就地更新已有关键帧的
    // pose，不一定伴随新增关键帧，所以不能靠"是否有新关键帧"这种脏标记来判断是否需要重新给出
    {
        UL lock2(mtx_current_scan_);
        if (all_keyframes_.size() > 1) {
            std::vector<Vec3f> loop_line_pts;
            loop_line_pts.reserve(all_keyframes_.size());
            for (const auto &kf : all_keyframes_) {
                loop_line_pts.emplace_back(kf->GetOptPose().translation().cast<float>());
            }
            scene_.SetLoopLinePoints(loop_line_pts);
        } else {
            scene_.SetLoopLinePoints({});
        }
    }

    // 绘制
    scene_.RenderFrame();
}

void PangolinWindowImpl::Render() {
    scene_.BindContext();

    // Issue specific OpenGl we might need
    // 启用OpenGL深度测试和混合功能，以支持透明度等效果。
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    scene_.CreateMenu();
    scene_.CreateDisplayLayout();

    exit_flag_.store(false);
    while (!scene_.ShouldQuit() && !exit_flag_) {
        scene_.BeginFrame();
        scene_.PollMenu();

        debug::flg_next = scene_.StepRequested();
        debug::play_speed = scene_.PlaySpeed();

        // Render pointcloud
        RenderClouds();

        // Swap frames and Process Events
        // 完成当前帧的渲染并处理与窗口交互相关的事件
        scene_.FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    scene_.Shutdown();
}

std::string PangolinWindowImpl::GetWindowName() const { return scene_.GetWindowName(); }

}  // namespace lightning::ui
