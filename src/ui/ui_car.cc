#include "ui/ui_car.h"
#include "ui/gl_colored_shader.h"

namespace lightning::ui {

std::vector<Vec3f> UiCar::car_vertices_ = {
    // clang-format off
     { 0, 0, 0}, { 3.0, 0, 0},
     { 0, 0, 0}, { 0, 3.0, 0},
     { 0, 0, 0}, { 0, 0, 3.0},
    // clang-format on
};

void UiCar::SetPose(const SE3& pose) {
    pts_.clear();
    for (auto& p : car_vertices_) {
        pts_.emplace_back(p);
    }

    // 转换到世界系
    auto pose_f = pose.cast<float>();
    for (auto& pt : pts_) {
        pt = pose_f * pt;
    }

    vbo_dirty_ = true;
}

void UiCar::Render(const Eigen::Matrix4f& mvp) {
    if (pts_.empty()) {
        return;
    }

    if (vbo_dirty_) {
        vbo_.Reinitialise(pangolin::GlArrayBuffer, static_cast<GLuint>(pts_.size()), GL_FLOAT, 3, GL_DYNAMIC_DRAW);
        vbo_.Upload(pts_);
        vbo_dirty_ = false;
    }

    /// x -红, y-绿 z-蓝 （目前三段共用一个颜色，见color_）
    GlColoredShader::Instance().DrawUniform(mvp, vbo_, pts_.size(), GL_LINES, Vec4f(color_[0], color_[1], color_[2], 1.0f),
                                            5.0f);
}

}  // namespace lightning::ui
