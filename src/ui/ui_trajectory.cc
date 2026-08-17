#include "ui/ui_trajectory.h"
#include "ui/gl_colored_shader.h"

namespace lightning::ui {

void UiTrajectory::AddPt(const SE3& pose) {
    // 如果轨迹点超出阈值 直接删除前一半点
    pos_.emplace_back(pose.translation().cast<float>());
    if (pos_.size() > max_size_) {
        pos_.erase(pos_.begin(), pos_.begin() + pos_.size() / 2);
    }
    vbo_dirty_ = true;
}

void UiTrajectory::Render(const Eigen::Matrix4f& mvp) {
    if (pos_.empty()) {
        return;
    }

    if (vbo_dirty_) {
        vbo_.Reinitialise(pangolin::GlArrayBuffer, static_cast<GLuint>(pos_.size()), GL_FLOAT, 3, GL_DYNAMIC_DRAW);
        vbo_.Upload(pos_);
        vbo_dirty_ = false;
    }

    // 点线形式
    GlColoredShader::Instance().DrawUniform(mvp, vbo_, pos_.size(), GL_LINE_STRIP,
                                            Vec4f(color_[0], color_[1], color_[2], 1.0f), 5.0f);
}

}  // namespace lightning::ui
