#pragma once

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/widgets.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/handler/handler.h>
#include <pangolin/plot/plotter.h>
#include <pangolin/var/var.h>

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/eigen_types.h"
#include "ui/ui_car.h"
#include "ui/ui_cloud.h"
#include "ui/ui_trajectory.h"

namespace lightning::ui {

/// 纯展示/渲染层：只依赖Pangolin窗口/显示/绘图部件 + UiCar/UiCloud/UiTrajectory + Eigen/Sophus，
/// 不引入PCL、rclcpp、Keyframe等重依赖，方便以后给Emscripten/web交叉编译。
/// PangolinWindowImpl（原生端，PCL/ROS2数据接入都在那）负责把数据转换成这里认识的类型（UiPoint/SE3/Vec3f）
/// 之后再调用本类；以后做wire protocol的话，wasm那一侧的数据来源会是网络反序列化，同样调用本类。
class PangolinUiScene {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PangolinUiScene() = default;

    /// 创建窗口、重置轨迹/当前scan等opengl资源
    void Init();
    /// 注销窗口
    void Shutdown();

    /// 把窗口上下文绑定到当前线程
    void BindContext();
    bool ShouldQuit() const;

    /// 创建菜单控件（须在CreateDisplayLayout前调用一次）
    void CreateMenu();
    /// 创建3D视窗+曲线图的布局（须在渲染循环开始前调用一次）
    void CreateDisplayLayout();

    /// 每帧开始：清屏
    void BeginFrame();
    /// 读取本帧菜单状态：更新跟随/轨迹显示开关，处理视角复位按钮，同步点云透明度
    void PollMenu();
    /// 交换缓冲区、处理窗口事件
    void FinishFrame();

    bool StepRequested() const { return menu_step_; }
    float PlaySpeed() const { return menu_play_speed_; }

    /// 重置：整段后端轨迹 + 最近若干scan（点云数据已由调用方转换好）
    void Reset(const std::vector<SE3>& backend_traj,
              const std::vector<std::pair<std::vector<UiPoint>, SE3>>& recent_scans);

    /// 全局/动态地图整份同步（按id增删+更新），世界系点云，pose固定为原点
    void SyncGlobalMap(const std::map<int, std::vector<UiPoint>>& clouds_by_id);
    void SyncDynamicMap(const std::map<int, std::vector<UiPoint>>& clouds_by_id);

    /// 把当前显示的current scan归档进scans_队列，换上新的一帧scan；同步更新后端轨迹/最新后端pose
    void PushCurrentScan(const std::vector<UiPoint>& points, const SE3& pose);
    /// 无论本帧是否有新scan，都维持scans_队列不超过上限
    void TrimScanQueue();
    void SetMaxScanQueueSize(int n) { max_size_of_current_scan_ = n; }

    /// 前端(滤波器)最新位姿：驱动相机跟随与前端小车位置，每帧都应传入当前值（不一定每帧变化）
    void SetFrontendPose(const SE3& pose) { newest_frontend_pose_ = pose; }
    /// 前端轨迹新增一个点（滤波器状态更新时调用，与SetFrontendPose是两回事：后者只影响当前显示位置）
    void AddFrontendTrajPt(const SE3& pose) { traj_newest_state_->AddPt(pose); }

    /// 闭环线：每帧由调用方重新给出世界系折线点（少于2个点则不画）
    void SetLoopLinePoints(const std::vector<Vec3f>& pts) { loop_line_pts_ = pts; }

    /// 滤波器状态曲线打点
    void LogFilterState(const Vec3d& vel, const Vec3d& vel_baselink, const Vec3d& bias_acc, double confidence);

    /// 渲染这一帧：计算跟随MVP + 画所有内容。调用方需先做完所有数据同步（Sync*/Push*/Set*）。
    void RenderFrame();

    std::string GetWindowName() const { return win_name_; }

   private:
    void DrawAll();

    int win_width_ = 1920;
    int win_height_ = 1080;
    static constexpr float cam_focus_ = 5000;
    static constexpr float cam_z_near_ = 1.0;
    static constexpr float cam_z_far_ = 1e10;
    static constexpr int menu_width_ = 210;
    const std::string win_name_ = "UI";
    const std::string dis_main_name_ = "main";
    const std::string dis_3d_name_ = "Cam 3D";
    const std::string dis_3d_main_name_ = "Cam 3D Main";  // main
    const std::string dis_plot_name_ = "Plot";

    bool following_loc_ = true;       // 相机是否追踪定位结果
    bool draw_frontend_traj_ = true;  // 可视化前端轨迹
    bool draw_backend_traj_ = true;   // 可视化后端轨迹

    pangolin::OpenGlRenderState s_cam_main_;

    /// 每帧在DrawAll()之前算好的投影*视图矩阵（含跟随偏移），供所有shader绘制调用共用。
    /// @note 不能用s_cam_main_.GetProjectionModelViewMatrix()：Pangolin的OpenGlRenderState::Follow()
    ///       只在legacy矩阵栈的Apply()里才会叠加跟随偏移(modelview*T_cw)，GetModelViewMatrix()本身不含它，
    ///       shader绘制不走矩阵栈，所以要自己在这里把跟随偏移叠加进去。
    Eigen::Matrix4f current_mvp_ = Eigen::Matrix4f::Identity();

    SE3 newest_frontend_pose_;
    SE3 newest_backend_pose_;

    ui::UiCar backend_car_{Vec3f(0.2, 0.2, 0.8)};
    ui::UiCar frontend_car_{Vec3f(0.2, 0.2, 0.8)};

    std::map<int, std::shared_ptr<ui::UiCloud>> cloud_map_ui_;  // 用来渲染的全局点云地图
    std::map<int, std::shared_ptr<ui::UiCloud>> cloud_dyn_ui_;  // 用来渲染的动态点云地图
    std::shared_ptr<ui::UiCloud> current_scan_ui_;              // current scan
    std::deque<std::shared_ptr<ui::UiCloud>> scans_;            // current scan 保留的队列
    int max_size_of_current_scan_ = 200;                        // 当前扫描数据保留多少个

    // 闭环后的轨迹（紫色，连接all_keyframes_的优化后位置），GLES3/WebGL安全绘制用
    std::vector<Vec3f> loop_line_pts_;
    pangolin::GlBuffer loop_line_vbo_;

    // trajectory
    std::shared_ptr<ui::UiTrajectory> traj_scans_;         // 激光扫描的轨迹
    std::shared_ptr<ui::UiTrajectory> traj_newest_state_;  // 最新state的轨迹

    // 滤波器状态相关 Data logger object
    pangolin::DataLog log_vel_;           // odom frame下的速度
    pangolin::DataLog log_vel_baselink_;  // baselink frame下的速度
    pangolin::DataLog log_bias_acc_;      // accelerometer bias
    pangolin::DataLog log_confidence_;    // confidence
    pangolin::DataLog log_error_;         // 误差

    std::unique_ptr<pangolin::Plotter> plotter_vel_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_vel_baselink_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_bias_acc_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_confidence_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_err_ = nullptr;

    // menu vars（须在CreateMenu()中构造：pangolin::Var不可默认构造）
    std::unique_ptr<pangolin::Var<bool>> menu_follow_loc_var_;
    std::unique_ptr<pangolin::Var<bool>> menu_draw_frontend_traj_var_;
    std::unique_ptr<pangolin::Var<bool>> menu_draw_backend_traj_var_;
    std::unique_ptr<pangolin::Var<bool>> menu_reset_3d_view_var_;
    std::unique_ptr<pangolin::Var<bool>> menu_reset_front_view_var_;
    std::unique_ptr<pangolin::Var<bool>> menu_step_var_;
    std::unique_ptr<pangolin::Var<float>> menu_play_speed_var_;
    std::unique_ptr<pangolin::Var<float>> menu_intensity_var_;

    bool menu_step_ = false;
    float menu_play_speed_ = 10.0f;
};

}  // namespace lightning::ui
