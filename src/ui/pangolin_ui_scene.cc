#include "ui/pangolin_ui_scene.h"

#include "ui/gl_colored_shader.h"

namespace lightning::ui {

void PangolinUiScene::Init() {
    // create a window and bind its context to the main thread
    pangolin::CreateWindowAndBind(win_name_, win_width_, win_height_);

    // 3D mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();

    traj_newest_state_ = std::make_shared<ui::UiTrajectory>(Vec3f(1.0, 0.0, 0.0));  // 红色
    traj_scans_ = std::make_shared<ui::UiTrajectory>(Vec3f(0.0, 1.0, 0.0));         // 绿色
    current_scan_ui_ = std::make_shared<ui::UiCloud>();

    /// data log
    log_vel_.SetLabels(std::vector<std::string>{"vel_x", "vel_y", "vel_z"});
    log_vel_baselink_.SetLabels(std::vector<std::string>{"baselink_vel_x", "baselink_vel_y", "baselink_vel_z"});
    log_bias_acc_.SetLabels(std::vector<std::string>{"ba_x", "ba_y", "ba_z"});
    log_confidence_.SetLabels(std::vector<std::string>{"lidar loc confidence"});
    log_error_.SetLabels(std::vector<std::string>{"err v", "err h", "err eval v", "err eval h"});
}

void PangolinUiScene::Shutdown() {
    pangolin::GetBoundWindow()->RemoveCurrent();
    pangolin::DestroyWindow(win_name_);
}

void PangolinUiScene::BindContext() { pangolin::BindToContext(win_name_); }

bool PangolinUiScene::ShouldQuit() const { return pangolin::ShouldQuit(); }

void PangolinUiScene::CreateMenu() {
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(menu_width_));
    menu_follow_loc_var_ = std::make_unique<pangolin::Var<bool>>("menu.Follow", false, true);
    menu_draw_frontend_traj_var_ = std::make_unique<pangolin::Var<bool>>("menu.Draw Frontend Traj", true, true);
    menu_draw_backend_traj_var_ = std::make_unique<pangolin::Var<bool>>("menu.Draw Backend Traj", true, true);
    menu_reset_3d_view_var_ = std::make_unique<pangolin::Var<bool>>("menu.Reset 3D View", false, false);
    menu_reset_front_view_var_ = std::make_unique<pangolin::Var<bool>>("menu.Set to front View", false, false);
    menu_step_var_ = std::make_unique<pangolin::Var<bool>>("menu.Step", false, false);
    menu_play_speed_var_ = std::make_unique<pangolin::Var<float>>("menu.Play speed", 10.0, 0.1, 10.0);
    menu_intensity_var_ = std::make_unique<pangolin::Var<float>>("menu.intensity", 0.5, 0.0, 1.0);
}

void PangolinUiScene::CreateDisplayLayout() {
    // define camera render object (for view / scene browsing)
    // 定义视点的透视投影方式
    auto proj_mat_main = pangolin::ProjectionMatrix(win_width_, win_width_, cam_focus_, cam_focus_, win_width_ / 2,
                                                    win_width_ / 2, cam_z_near_, cam_z_far_);
    // 模型视图矩阵定义了视点的位置和朝向
    auto model_view_main = pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, pangolin::AxisY);
    s_cam_main_ = pangolin::OpenGlRenderState(std::move(proj_mat_main), std::move(model_view_main));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View &d_cam3d_main = pangolin::Display(dis_3d_main_name_)
                                       .SetBounds(0.0, 1.0, 0.0, 1.0)
                                       .SetHandler(new pangolin::Handler3D(s_cam_main_));

    pangolin::View &d_cam3d = pangolin::Display(dis_3d_name_)
                                  .SetBounds(0.0, 1.0, 0.0, 0.75)
                                  .SetLayout(pangolin::LayoutOverlay)
                                  .AddDisplay(d_cam3d_main);

    // OpenGL 'view' of data. We might have many views of the same data.
    plotter_vel_ = std::make_unique<pangolin::Plotter>(&log_vel_, -10, 600, -11, 11, 75, 2);
    plotter_vel_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_vel_->Track("$i");
    plotter_vel_baselink_ = std::make_unique<pangolin::Plotter>(&log_vel_baselink_, -10, 600, -11, 11, 75, 2);
    plotter_vel_baselink_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_vel_baselink_->Track("$i");
    plotter_bias_acc_ = std::make_unique<pangolin::Plotter>(&log_bias_acc_, -10, 600, -1, 1, 75, 0.1);
    plotter_bias_acc_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_bias_acc_->Track("$i");
    plotter_confidence_ = std::make_unique<pangolin::Plotter>(&log_confidence_, -10, 600, 0, 5.0, 100, 0.5);
    plotter_confidence_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_confidence_->Track("$i");
    plotter_err_ = std::make_unique<pangolin::Plotter>(&log_error_, -10, 600, 0, 1.0, 100, 0.1);
    plotter_err_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_err_->Track("$i");

    pangolin::View &d_plot = pangolin::Display(dis_plot_name_)
                                 .SetBounds(0.0, 1.0, 0.75, 1.0)
                                 .SetLayout(pangolin::LayoutEqualVertical)
                                 .AddDisplay(*plotter_confidence_)
                                 .AddDisplay(*plotter_err_)
                                 .AddDisplay(*plotter_bias_acc_)
                                 .AddDisplay(*plotter_vel_)
                                 .AddDisplay(*plotter_vel_baselink_);
    pangolin::Display(dis_main_name_)
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(menu_width_), 1.0)
        .AddDisplay(d_cam3d)
        .AddDisplay(d_plot);
}

void PangolinUiScene::BeginFrame() {
    // Issue specific OpenGl we might need
    // 清除了颜色缓冲区（GL_COLOR_BUFFER_BIT）和深度缓冲区（GL_DEPTH_BUFFER_BIT）。
    glClearColor(20.0 / 255.0, 20.0 / 255.0, 20.0 / 255.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void PangolinUiScene::PollMenu() {
    following_loc_ = *menu_follow_loc_var_;
    draw_frontend_traj_ = *menu_draw_frontend_traj_var_;
    draw_backend_traj_ = *menu_draw_backend_traj_var_;

    if (*menu_reset_3d_view_var_) {
        s_cam_main_.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0, 1000, 0, 0, 0, pangolin::AxisY));
        *menu_reset_3d_view_var_ = false;
    }

    if (*menu_reset_front_view_var_) {
        s_cam_main_.SetModelViewMatrix(pangolin::ModelViewLookAt(-50, 0, 10, 50, 0, 10, pangolin::AxisZ));
        *menu_reset_front_view_var_ = false;
    }

    menu_step_ = *menu_step_var_;
    menu_play_speed_ = *menu_play_speed_var_;
    ui::opacity = *menu_intensity_var_;
}

void PangolinUiScene::FinishFrame() { pangolin::FinishFrame(); }

void PangolinUiScene::Reset(const std::vector<SE3> &backend_traj,
                            const std::vector<std::pair<std::vector<UiPoint>, SE3>> &recent_scans) {
    cloud_map_ui_.clear();
    scans_.clear();
    current_scan_ui_ = nullptr;
    traj_scans_->Clear();

    for (const auto &pose : backend_traj) {
        traj_scans_->AddPt(pose);
    }

    for (const auto &scan : recent_scans) {
        current_scan_ui_ = std::make_shared<ui::UiCloud>();
        current_scan_ui_->SetCloud(scan.first, scan.second);
        current_scan_ui_->SetRenderColor(ui::UiCloud::UseColor::HEIGHT_COLOR);
        scans_.emplace_back(current_scan_ui_);
    }

    newest_backend_pose_ = backend_traj.back();
}

void PangolinUiScene::SyncGlobalMap(const std::map<int, std::vector<UiPoint>> &clouds_by_id) {
    for (const auto &cp : clouds_by_id) {
        if (cloud_map_ui_.find(cp.first) != cloud_map_ui_.end()) {
            continue;
        }

        auto ui_cloud = std::make_shared<ui::UiCloud>();
        ui_cloud->SetCloud(cp.second, SE3());
        ui_cloud->SetRenderColor(ui::UiCloud::UseColor::GRAY_COLOR);
        cloud_map_ui_.emplace(cp.first, ui_cloud);
    }

    for (auto iter = cloud_map_ui_.begin(); iter != cloud_map_ui_.end();) {
        if (clouds_by_id.find(iter->first) == clouds_by_id.end()) {
            iter = cloud_map_ui_.erase(iter);
        } else {
            ++iter;
        }
    }
}

void PangolinUiScene::SyncDynamicMap(const std::map<int, std::vector<UiPoint>> &clouds_by_id) {
    for (const auto &cp : clouds_by_id) {
        auto it = cloud_dyn_ui_.find(cp.first);
        if (it != cloud_dyn_ui_.end()) {
            // 存在也要更新
            it->second = std::make_shared<ui::UiCloud>();
            it->second->SetCustomColor(Vec4f(0.0, 0.2, 1.0, 1.0));
            it->second->SetCloud(cp.second, SE3());
            it->second->SetRenderColor(ui::UiCloud::UseColor::CUSTOM_COLOR);
            continue;
        }

        /// 不存在则创建一个
        auto ui_cloud = std::make_shared<ui::UiCloud>();
        ui_cloud->SetCustomColor(Vec4f(0.0, 0.2, 1.0, 1.0));
        ui_cloud->SetCloud(cp.second, SE3());
        ui_cloud->SetRenderColor(ui::UiCloud::UseColor::CUSTOM_COLOR);
        cloud_dyn_ui_.emplace(cp.first, ui_cloud);
    }

    for (auto iter = cloud_dyn_ui_.begin(); iter != cloud_dyn_ui_.end();) {
        if (clouds_by_id.find(iter->first) == clouds_by_id.end()) {
            iter = cloud_dyn_ui_.erase(iter);
        } else {
            ++iter;
        }
    }
}

void PangolinUiScene::PushCurrentScan(const std::vector<UiPoint> &points, const SE3 &pose) {
    if (current_scan_ui_) {
        current_scan_ui_->SetRenderColor(ui::UiCloud::UseColor::HEIGHT_COLOR);
        scans_.emplace_back(current_scan_ui_);
    }

    current_scan_ui_ = std::make_shared<ui::UiCloud>();
    current_scan_ui_->SetCloud(points, pose);
    current_scan_ui_->SetRenderColor(ui::UiCloud::UseColor::HEIGHT_COLOR);

    traj_scans_->AddPt(pose);
    newest_backend_pose_ = pose;
}

void PangolinUiScene::TrimScanQueue() {
    while (static_cast<int>(scans_.size()) >= max_size_of_current_scan_) {
        scans_.pop_front();
    }
}

void PangolinUiScene::LogFilterState(const Vec3d &vel, const Vec3d &vel_baselink, const Vec3d &bias_acc,
                                     double confidence) {
    log_vel_.Log(vel(0), vel(1), vel(2));
    log_vel_baselink_.Log(vel_baselink(0), vel_baselink(1), vel_baselink(2));
    log_bias_acc_.Log(bias_acc(0), bias_acc(1), bias_acc(2));
    log_confidence_.Log(confidence);
}

void PangolinUiScene::DrawAll() {
    /// 地图
    for (const auto &pc : cloud_map_ui_) {
        pc.second->Render(current_mvp_);
    }

    /// 动态地图
    for (const auto &pc : cloud_dyn_ui_) {
        pc.second->Render(current_mvp_);
    }

    /// 缓存的scans
    for (const auto &s : scans_) {
        s->Render(current_mvp_);
    }

    if (current_scan_ui_) {
        current_scan_ui_->Render(current_mvp_);
    }

    if (draw_frontend_traj_) {
        traj_newest_state_->Render(current_mvp_);
        // 车
        frontend_car_.SetPose(newest_frontend_pose_);  // 车在current pose上
        frontend_car_.Render(current_mvp_);
    }

    if (draw_backend_traj_) {
        traj_scans_->Render(current_mvp_);
        // 车
        backend_car_.SetPose(newest_backend_pose_);
        backend_car_.Render(current_mvp_);
    }

    /// 闭环后的轨迹：每帧都由调用方重新给出（闭环优化可能就地更新已有关键帧的pose，
    /// 不一定伴随新增关键帧，所以调用方不能靠"是否有新关键帧"这种脏标记来判断是否需要重新给出）
    if (loop_line_pts_.size() > 1) {
        loop_line_vbo_.Reinitialise(pangolin::GlArrayBuffer, static_cast<GLuint>(loop_line_pts_.size()), GL_FLOAT, 3,
                                    GL_DYNAMIC_DRAW);
        loop_line_vbo_.Upload(loop_line_pts_);

        ui::GlColoredShader::Instance().DrawUniform(current_mvp_, loop_line_vbo_, loop_line_pts_.size(),
                                                     GL_LINE_STRIP, Vec4f(0.5f, 0.0f, 0.5f, 1.0f), 5.0f);
    }
}

void PangolinUiScene::RenderFrame() {
    /// 处理相机跟随问题：手动叠加跟随偏移，不依赖只在legacy矩阵栈里生效的OpenGlRenderState::Follow()
    /// （原理同Follow()：把当前跟踪目标的世界系XY平移的逆，叠加在用户鼠标操作出的base modelview之上，
    /// 让目标始终位于视野中心；Z置0、旋转为单位阵，对应原来"只平移不跟随朝向"的效果）
    Eigen::Matrix4d modelview = s_cam_main_.GetModelViewMatrix();
    if (following_loc_) {
        Eigen::Vector3d translation = newest_frontend_pose_.translation();
        Sophus::SE3d T_wc_now(Eigen::Quaterniond::Identity(), Eigen::Vector3d(translation.x(), translation.y(), 0.0));
        modelview = modelview * T_wc_now.matrix().inverse();
    }
    current_mvp_ = (Eigen::Matrix4d(s_cam_main_.GetProjectionMatrix()) * modelview).cast<float>();

    pangolin::Display(dis_3d_main_name_).Activate(s_cam_main_);
    DrawAll();
}

}  // namespace lightning::ui
