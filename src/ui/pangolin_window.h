#pragma once

#include <map>
#include <memory>
#include <queue>

#include "common/eigen_types.h"
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "common/nav_state.h"
#include "common/point_def.h"

#ifdef LIGHTNING_ENABLE_RERUN
namespace rerun {
class RecordingStream;
}
#endif

namespace lightning::ui {

class PangolinWindowImpl;

/**
 * @note 此类本身不直接涉及任何opengl和pangolin操作，应当放到PangolinWindowImpl中
 */
class PangolinWindow {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PangolinWindow();
    ~PangolinWindow();

    /// @brief 初始化窗口，后台启动render线程。
    /// @note 与opengl/pangolin无关的初始化，尽量放到此函数体中;
    ///       opengl/pangolin相关的内容，尽量放到PangolinWindowImpl::Init中。
    bool Init();

    void Reset(const std::vector<Keyframe::Ptr>& keyframes);

    /// 更新激光地图点云，在激光定位中的地图发生改变时，由fusion调用
    void UpdatePointCloudGlobal(const std::map<int, CloudPtr>& cloud);

    /// 更新动态图层点云
    void UpdatePointCloudDynamic(const std::map<int, CloudPtr>& cloud);

    /// 更新kalman滤波器状态
    void UpdateNavState(const NavState& state);

    /// 更新最新的pose
    void UpdateRecentPose(const SE3& pose);

    void UpdatePredictPose(const SE3& pose);

    /// 更新一次scan和它对应的Pose
    void UpdateScan(CloudPtr cloud, const SE3& pose);

    void UpdateKF(std::shared_ptr<Keyframe> kf);

    /// 等待显示线程结束，并释放资源
    void Quit();

    /// 用户是否已经退出UI
    bool ShouldQuit();

    /// 设置IMU到雷达的外参
    void SetTImuLidar(const SE3& T_imu_lidar);

    /// 设置需要保留多少个扫描数据
    void SetCurrentScanSize(int current_scan_size);

   private:
    std::shared_ptr<PangolinWindowImpl> impl_ = nullptr;

#ifdef LIGHTNING_ENABLE_RERUN
    /// 将各 Update* 调用同时投递给 rerun，供网页端 web viewer 实时查看
    /// @note 前向声明以避免把 rerun.hpp 传递给所有包含本头文件的调用方
    std::unique_ptr<rerun::RecordingStream> rerun_stream_;

    /// UpdateNavState/UpdateRecentPose 共用：更新前端车辆位姿与红色轨迹
    void LogFrontendPose(const SE3& pose);
#endif
};
}  // namespace lightning::ui
