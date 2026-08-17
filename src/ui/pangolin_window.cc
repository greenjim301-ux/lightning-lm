#include "ui/pangolin_window.h"
#include "ui/ui_wire_server.h"

namespace lightning::ui {

PangolinWindow::PangolinWindow() { impl_ = std::make_shared<UiWireServer>(); }
PangolinWindow::~PangolinWindow() { Quit(); }

bool PangolinWindow::Init() { return impl_->Init(); }

void PangolinWindow::Reset(const std::vector<Keyframe::Ptr>& keyframes) { impl_->Reset(keyframes); }

void PangolinWindow::Quit() { impl_->Quit(); }

void PangolinWindow::UpdatePointCloudGlobal(const std::map<int, CloudPtr>& cloud) {
    impl_->UpdatePointCloudGlobal(cloud);
}

void PangolinWindow::UpdatePointCloudDynamic(const std::map<int, CloudPtr>& cloud) {
    impl_->UpdatePointCloudDynamic(cloud);
}

void PangolinWindow::UpdateNavState(const NavState& state) { impl_->UpdateNavState(state); }

void PangolinWindow::UpdateRecentPose(const SE3& pose) { impl_->UpdateRecentPose(pose); }

void PangolinWindow::UpdatePredictPose(const SE3& pose) { impl_->UpdatePredictPose(pose); }

void PangolinWindow::UpdateScan(CloudPtr cloud, const SE3& pose) { impl_->UpdateScan(cloud, pose); }

void PangolinWindow::UpdateKF(std::shared_ptr<Keyframe> kf) { impl_->UpdateKF(kf); }

void PangolinWindow::SetCurrentScanSize(int current_scan_size) { impl_->SetCurrentScanSize(current_scan_size); }

void PangolinWindow::SetTImuLidar(const SE3& T_imu_lidar) { impl_->SetTImuLidar(T_imu_lidar); }

bool PangolinWindow::ShouldQuit() { return impl_->ShouldQuit(); }

}  // namespace lightning::ui
