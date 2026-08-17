#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "common/keyframe.h"
#include "common/loop_candidate.h"

#include "ui/pangolin_window.h"
#include "ui/pangolin_ui_scene.h"

namespace lightning::ui {

/**
 * PCL/ROS2数据接入端：负责从fusion/lio/lc/g2p5等原生模块接收数据（锁+PCL点云+Keyframe），
 * 转换成PangolinUiScene认识的轻量类型（UiPoint/SE3/Vec3f）后交给scene_渲染。
 * @note 渲染/展示相关的实现都在PangolinUiScene里（不依赖PCL/ROS2，方便以后给Emscripten/web交叉编译）。
 */
class PangolinWindowImpl {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PangolinWindowImpl() = default;
    ~PangolinWindowImpl() = default;

    PangolinWindowImpl(const PangolinWindowImpl &) = delete;
    PangolinWindowImpl &operator=(const PangolinWindowImpl &) = delete;
    PangolinWindowImpl(PangolinWindowImpl &&) = delete;
    PangolinWindowImpl &operator=(PangolinWindowImpl &&) = delete;

    /// 初始化，创建可用于渲染的点云和轨迹
    bool Init();

    void Reset(const std::vector<Keyframe::Ptr> &keyframes);

    /// 注销
    bool DeInit();

    /// 渲染所有信息
    void Render();

    /// 获取窗口名称
    std::string GetWindowName() const;

   public:
    /// 后台渲染线程
    std::thread render_thread_;

    /// 一些辅助的锁和原子变量
    std::mutex mtx_map_cloud_;
    std::mutex mtx_current_scan_;
    std::mutex mtx_nav_state_;
    std::mutex mtx_gps_pose_;
    std::mutex mtx_loop_info_;

    std::mutex mtx_reset_;

    std::atomic<bool> exit_flag_;

    std::atomic<bool> cloud_global_need_update_;   // 全局点云是否需要更新
    std::atomic<bool> cloud_dynamic_need_update_;  // 动态点云是否需要更新
    std::atomic<bool> kf_result_need_update_;      // 卡尔曼滤波结果
    std::atomic<bool> current_scan_need_update_;   // 更新当前扫描
    std::atomic<bool> lidarloc_need_update_;       // 雷达位置？

    pcl::PointCloud<PointType>::Ptr current_scan_ = nullptr;  // 当前scan
    SE3 newest_frontend_pose_;                                // 最新pose，也可能被PangolinWindow::UpdateRecentPose直接写
    SE3 predicted_pose_;
    SE3 current_scan_pose_;  // 当前scan对应的pose or Twb/Twi
    std::deque<std::pair<int, int>> loop_info_;
    std::vector<LoopCandidate> new_loop_candidate_;

    // 地图点云
    std::map<int, CloudPtr> cloud_global_map_;
    std::map<int, CloudPtr> cloud_dynamic_map_;

    /// 滤波器状态
    Sophus::SE3d pose_;
    double confidence_;
    Vec3d vel_;
    Vec3d bias_acc_;
    Vec3d bias_gyr_;
    Vec3d grav_;

    Sophus::SE3d T_imu_lidar_;
    int max_size_of_current_scan_ = 200;  // 当前扫描数据保留多少个
    std::vector<std::shared_ptr<Keyframe>> all_keyframes_;

    //////////////////////////////// 以下和render相关 ///////////////////////////
   private:
    /// 渲染点云，调用各种Update函数
    void RenderClouds();
    bool UpdateGlobalMap();
    bool UpdateDynamicMap();
    bool UpdateState();
    bool UpdateCurrentScan();

   private:
    ui::PangolinUiScene scene_;
};

}  // namespace lightning::ui
