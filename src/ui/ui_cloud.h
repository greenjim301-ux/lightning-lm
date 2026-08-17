#pragma once

#include <pangolin/gl/gl.h>

#include <vector>

#include "common/eigen_types.h"

namespace lightning::ui {

/// UiCloud用的点，故意跟pcl::PointCloud完全解耦（不引入PCL），因为PCL拉Boost/FLANN一整套依赖，
/// 没法给Emscripten/web交叉编译。把PCL点云转换成这个的责任在调用方——目前是pangolin_window_impl.cc
/// 里的ToUiPoints()（原生端，PCL可用）；以后走wire protocol的话，会是从网络反序列化出来的代码。
struct UiPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float intensity = 0.0f;
};

/// 点云透明度，由菜单的intensity滑块控制。声明在这里而不是common/options.h：
/// 那个头文件拉了rclcpp（ROS2），ui_cloud.cc要保持ROS2/PCL无关，方便以后给web交叉编译。
extern float opacity;

/// 在UI中使用的点云
/// 固定不变的点云都可以用这个来渲染
class UiCloud {
   public:
    /// 使用哪种颜色渲染本点云
    enum UseColor {
        PCL_COLOR,        // PCL颜色，偏红
        INTENSITY_COLOR,  // 亮度
        HEIGHT_COLOR,     // 高度
        GRAY_COLOR,       // 显示为灰色
        CUSTOM_COLOR,     // 显示为自定颜色
    };

    UiCloud() {}

    /**
     * 设置UI点云
     * @param points            点云: lP, coordinates are in Lidar frame
     * @param pose              点云位姿: Twi，
     */
    void SetCloud(const std::vector<UiPoint>& points, const SE3& pose);

    /// 渲染这个点云
    /// @param mvp 调用方算好的投影*视图矩阵（含相机跟随偏移，见PangolinWindowImpl::current_mvp_）
    void Render(const Eigen::Matrix4f& mvp);

    /// 指定内置颜色
    void SetRenderColor(UseColor use_color);

    /// 指定自选颜色，RGBA
    void SetCustomColor(Vec4f custom_color);

    void SetPointSize(float point_size) { point_size_ = point_size; }

   private:
    Vec4f IntensityToRgbPCL(const float& intensity) const {
        int index = int(intensity * 3);
        index = index % intensity_color_table_pcl_.size();
        return intensity_color_table_pcl_[index];
    }

    /// 根据use_color_，把对应的color_data_*重新上传到vbo_color_
    /// （惰性：只在SetCloud/SetRenderColor/SetCustomColor之后、下次Render()时才会真正重新上传一次）
    void UploadColorIfDirty();

    UseColor use_color_ = UseColor::PCL_COLOR;
    Vec4f custom_color_ = Vec4f::Zero();

    float point_size_ = 1.0f;
    std::vector<Vec3f> xyz_data_;              // 点的世界坐标
    std::vector<Vec4f> color_data_pcl_;        // 根据intensity映射得到的颜色
    std::vector<Vec4f> color_data_intensity_;  // 点的intensity不映射直接做颜色
    std::vector<Vec4f> color_data_height_;     // 根据height映射得到的颜色
    std::vector<Vec4f> color_data_gray_;       // 全部点都为灰色

    /// PCL中intensity table
    void BuildIntensityTable();
    // 颜色映射表
    static std::vector<Vec4f> intensity_color_table_pcl_;

    // GLES3/WebGL安全绘制所需的显存数据，见gl_colored_shader.h
    pangolin::GlBuffer vbo_pos_;
    pangolin::GlBuffer vbo_color_;
    bool color_dirty_ = true;
};

}  // namespace lightning::ui
