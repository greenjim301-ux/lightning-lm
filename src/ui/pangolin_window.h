#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "common/eigen_types.h"
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "common/nav_state.h"
#include "common/point_def.h"

namespace rerun {
class RecordingStream;
}

namespace lightning::ui {

/**
 * @note 基于 rerun_cpp 的可视化后端：数据通过 gRPC 推送给 rerun 的 web viewer，
 *       在浏览器中渲染（见 @rerun-io/web-viewer-react），取代了原先的 Pangolin/OpenGL 原生窗口。
 *       类名/公开接口保持不变，以避免改动所有调用方。
 */
class PangolinWindow {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PangolinWindow();
    ~PangolinWindow();

    /// @brief 启动 rerun gRPC server（供web viewer前端连接），并写入静态坐标系设定
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

    /// 空操作：rerun server 持续运行，没有可被用户关闭的窗口需要等待
    void Quit();

    /// 恒为false，理由同Quit()
    bool ShouldQuit();

    /// 设置IMU到雷达的外参（当前渲染逻辑未使用，仅保留接口）
    void SetTImuLidar(const SE3& T_imu_lidar);

    /// 设置Reset()时保留最近多少个关键帧的点云（避免一次性推送过多历史数据）
    void SetCurrentScanSize(int current_scan_size);

   private:
    /// UpdateNavState/UpdateRecentPose 共用：更新前端车辆位姿与红色轨迹
    void LogFrontendPose(const SE3& pose);

    /// UpdatePointCloudGlobal/UpdatePointCloudDynamic 共用：静态点云地图作为log_static写入
    /// （不受serve_grpc的server_memory_limit淘汰），因此子图被卸载时需要显式Clear，否则会永久残留。
    /// @param entity_prefix 例如 "world/map/" 或 "world/dynamic/"
    /// @param known_ids 该图层已知的子图id集合，函数会原地更新为cloud的id集合
    void SyncStaticSubmapCloud(const std::string& entity_prefix, const std::map<int, CloudPtr>& cloud,
                               std::set<int>& known_ids, uint8_t r, uint8_t g, uint8_t b);

    /// @note 前向声明以避免把 rerun.hpp 传递给所有包含本头文件的调用方
    std::unique_ptr<rerun::RecordingStream> rerun_stream_;

    int max_size_of_current_scan_ = 200;

    std::mutex mtx_keyframes_;
    std::vector<std::shared_ptr<Keyframe>> all_keyframes_;  // 用于重绘闭环轨迹

    std::set<int> global_map_submap_ids_;   // world/map/* 当前已知的子图id，用于检测被卸载的子图
    std::set<int> dynamic_map_submap_ids_;  // world/dynamic/* 同上
};
}  // namespace lightning::ui
