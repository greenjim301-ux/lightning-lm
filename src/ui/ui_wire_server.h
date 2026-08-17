#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include "common/keyframe.h"
#include "common/nav_state.h"
#include "common/point_def.h"

namespace lightning::ui {

/// PangolinWindow的线协议版本：实现完全相同的公开接口（见ui/pangolin_window.h），
/// 所以slam.cc/localization.cpp等~14处调用点不用改一行，只需要换一下实例化哪个UI后端。
/// 内部把每个Update*调用转换成ui_wire_protocol.h定义的二进制消息，通过websocket推给
/// 浏览器里跑PangolinUiScene的wasm端。Boost.Beast相关类型全部藏在.cc里（pimpl），
/// 这个头文件本身仍然是PCL/rclcpp都能碰的原生端头（跟PangolinWindowImpl一个级别）。
class UiWireServer {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit UiWireServer(uint16_t port = 9877);
    ~UiWireServer();

    UiWireServer(const UiWireServer&) = delete;
    UiWireServer& operator=(const UiWireServer&) = delete;

    bool Init();

    /// 目前没有外部调用点（见调研），保留只是为了跟PangolinWindow接口对齐；不发送任何消息。
    void Reset(const std::vector<Keyframe::Ptr>& keyframes);

    void UpdatePointCloudGlobal(const std::map<int, CloudPtr>& cloud);
    void UpdatePointCloudDynamic(const std::map<int, CloudPtr>& cloud);
    void UpdateNavState(const NavState& state);
    void UpdateRecentPose(const SE3& pose);
    /// 目前没有实际调用点（见调研），保留只是为了接口对齐；不发送任何消息。
    void UpdatePredictPose(const SE3& pose);
    void UpdateScan(CloudPtr cloud, const SE3& pose);
    void UpdateKF(std::shared_ptr<Keyframe> kf);

    void Quit();
    bool ShouldQuit();

    void SetTImuLidar(const SE3& T_imu_lidar);
    void SetCurrentScanSize(int current_scan_size);

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lightning::ui
