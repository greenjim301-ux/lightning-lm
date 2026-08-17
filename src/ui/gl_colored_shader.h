#pragma once

#include <pangolin/gl/gl.h>
#include <pangolin/gl/glsl.h>

#include "common/eigen_types.h"

namespace lightning::ui {

/**
 * 共享的、GLES3/WebGL安全的着色器。
 *
 * @note UiCloud/UiCar/UiTrajectory原先用glBegin/glVertex3f/glColor4f这类立即绘制模式
 *       (immediate mode)，这在桌面OpenGL上能用，但在Pangolin的Emscripten(web)构建目标
 *       (-sFULL_ES3=1，不带-sLEGACY_GL_EMULATION)下不存在，无法编译。这里换成
 *       glVertexAttribPointer+GlBuffer+shader的写法，与Pangolin自己的Plotter内部实现
 *       一致(见pango_plot/src/plotter.cpp)，已经在其官方web demo中验证可用。
 *
 *       调用方(UiCloud等)各自持有自己的GlBuffer，这里只负责编译一次共享shader程序、
 *       绑定属性、发出绘制调用。
 */
class GlColoredShader {
   public:
    static GlColoredShader& Instance();

    GlColoredShader(const GlColoredShader&) = delete;
    GlColoredShader& operator=(const GlColoredShader&) = delete;

    /// 逐顶点变色绘制（点云用）：vbo_pos每顶点3个float，vbo_color每顶点4个float(rgba)
    /// @param mvp 调用方自己算好的投影*视图矩阵（不要用OpenGlRenderState::GetProjectionModelViewMatrix()：
    ///        它不包含Follow()跟随偏移，那个偏移只在legacy矩阵栈的Apply()里才会叠加，见pangolin_window_impl.cc
    ///        里对current_mvp_的计算）
    /// @param alpha_override 覆盖掉vbo_color里每个顶点自带的alpha，对应原ui::opacity滑块每帧生效的行为
    ///        （原实现是glColor4f(rgb, ui::opacity)，rgb来自颜色表，alpha永远读自滑块，与颜色表自带的alpha无关）
    void DrawVarying(const Eigen::Matrix4f& mvp, pangolin::GlBuffer& vbo_pos, pangolin::GlBuffer& vbo_color,
                     size_t num_pts, GLenum mode, float point_size, float alpha_override);

    /// 单一颜色绘制（车、轨迹、闭环连线用）：只需要位置buffer + 一个uniform颜色
    void DrawUniform(const Eigen::Matrix4f& mvp, pangolin::GlBuffer& vbo_pos, size_t num_pts, GLenum mode,
                     const Vec4f& color, float line_width = 1.0f);

   private:
    GlColoredShader();

    pangolin::GlSlProgram prog_;
    GLint loc_position_ = -1;
    GLint loc_color_ = -1;
};

}  // namespace lightning::ui
