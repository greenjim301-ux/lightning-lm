#include "ui/ui_cloud.h"
#include "common/options.h"
#include "ui/gl_colored_shader.h"

#include <numeric>

namespace lightning::ui {

std::vector<Vec4f> UiCloud::intensity_color_table_pcl_;

UiCloud::UiCloud(CloudPtr cloud) { SetCloud(cloud, SE3()); }

void UiCloud::SetCustomColor(Vec4f custom_color) {
    custom_color_ = custom_color;
    color_dirty_ = true;
}

// 把输入的点云映射为opengl可以渲染的点云
void UiCloud::SetCloud(CloudPtr cloud, const SE3& pose) {
    if (intensity_color_table_pcl_.empty()) {
        BuildIntensityTable();
    }

    // assert(cloud != nullptr && cloud->empty() == false);
    xyz_data_.resize(cloud->size());
    color_data_pcl_.resize(cloud->size());
    color_data_intensity_.resize(cloud->size());
    color_data_height_.resize(cloud->size());
    color_data_gray_.resize(cloud->size());

    std::vector<int> idx(cloud->size());
    std::iota(idx.begin(), idx.end(), 0);  // 使用从0开始递增的整数填充idx

    SE3f pose_l = (pose).cast<float>();

    // 遍历所有点
    for (auto iter = idx.begin(); iter != idx.end(); iter++) {
        const int& id = *iter;
        const auto& pt = cloud->points[id];
        // 计算点的世界坐标
        auto pt_world = pose_l * cloud->points[id].getVector3fMap();
        xyz_data_[id] = Vec3f(pt_world.x(), pt_world.y(), pt_world.z());
        // 把intensity映射为颜色
        color_data_pcl_[id] = IntensityToRgbPCL(pt.intensity);
        color_data_gray_[id] = Vec4f(0.5, 0.5, 0.5, 1.0);
        // 根据高度映射颜色
        color_data_height_[id] = IntensityToRgbPCL(pt.z * 10);
        color_data_intensity_[id] =
            Vec4f(pt.intensity / 255.0 * 3.0, pt.intensity / 255.0 * 3.0, pt.intensity / 255.0 * 3.0, 1.0);
    }

    vbo_pos_.Reinitialise(pangolin::GlArrayBuffer, static_cast<GLuint>(xyz_data_.size()), GL_FLOAT, 3,
                          GL_DYNAMIC_DRAW);
    if (!xyz_data_.empty()) {
        vbo_pos_.Upload(xyz_data_);
    }
    color_dirty_ = true;
}

void UiCloud::UploadColorIfDirty() {
    if (!color_dirty_) {
        return;
    }

    const std::vector<Vec4f>* colors = &color_data_pcl_;
    std::vector<Vec4f> custom_expanded;

    switch (use_color_) {
        case UseColor::PCL_COLOR:
            colors = &color_data_pcl_;
            break;
        case UseColor::INTENSITY_COLOR:
            colors = &color_data_intensity_;
            break;
        case UseColor::HEIGHT_COLOR:
            colors = &color_data_height_;
            break;
        case UseColor::GRAY_COLOR:
            colors = &color_data_gray_;
            break;
        case UseColor::CUSTOM_COLOR:
            custom_expanded.assign(xyz_data_.size(), custom_color_);
            colors = &custom_expanded;
            break;
    }

    vbo_color_.Reinitialise(pangolin::GlArrayBuffer, static_cast<GLuint>(colors->size()), GL_FLOAT, 4,
                            GL_DYNAMIC_DRAW);
    if (!colors->empty()) {
        vbo_color_.Upload(*colors);
    }
    color_dirty_ = false;
}

void UiCloud::Render(const Eigen::Matrix4f& mvp) {
    if (xyz_data_.empty()) {
        return;
    }

    UploadColorIfDirty();
    // ui::opacity每帧都可能被菜单滑块改变，所以每次Render都要重新传，不能只在颜色脏的时候传一次
    GlColoredShader::Instance().DrawVarying(mvp, vbo_pos_, vbo_color_, xyz_data_.size(), GL_POINTS, point_size_,
                                            ui::opacity);
}

void UiCloud::BuildIntensityTable() {
    intensity_color_table_pcl_.reserve(255 * 3);
    // 接受rgb三个值，将它们归一化到范围 [0, 1]，同时设置 alpha 通道为0.2。
    auto make_color = [](int r, int g, int b) -> Vec4f { return Vec4f(r / 255.0f, g / 255.0f, b / 255.0f, 0.2f); };

    for (int i = 0; i < 256; i++) {
        intensity_color_table_pcl_.emplace_back(make_color(255, i, 0));
    }
    for (int i = 0; i < 256; i++) {
        intensity_color_table_pcl_.emplace_back(make_color(i, 0, 255));
    }

    for (int i = 0; i < 256; i++) {
        intensity_color_table_pcl_.emplace_back(make_color(0, 255, i));
    }
}

void UiCloud::SetRenderColor(UiCloud::UseColor use_color) {
    use_color_ = use_color;
    color_dirty_ = true;
}

}  // namespace lightning::ui
