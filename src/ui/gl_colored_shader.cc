#include "ui/gl_colored_shader.h"

namespace lightning::ui {

namespace {

// GLSL ES 1.00风格 (attribute/varying，无#version)：与Pangolin自身Plotter的写法一致
// (pango_plot/src/plotter.cpp)，已在Pangolin官方Emscripten web demo (SimplePlot)中验证可用。
const char* kVertexShader = R"(
attribute vec3 a_position;
attribute vec4 a_color;
uniform mat4 u_mvp;
uniform vec4 u_uniform_color;
uniform float u_use_uniform_color;
uniform float u_point_size;
uniform float u_alpha_override;
varying vec4 v_color;
void main() {
    gl_Position = u_mvp * vec4(a_position, 1.0);
    gl_PointSize = u_point_size;
    // use_uniform_color==1时，a_color读到的是禁用属性的规范默认值(0,0,0,1)，
    // mix()结果仍精确等于u_uniform_color，不需要额外绑定一个占位的颜色buffer。
    vec4 base_color = mix(a_color, u_uniform_color, u_use_uniform_color);
    // alpha_override<0表示不覆盖，直接用base_color自带的alpha
    v_color = vec4(base_color.rgb, u_alpha_override < 0.0 ? base_color.a : u_alpha_override);
}
)";

// precision是GLES专属指令，桌面GL上会导致语法错误（fragment shader编译失败->
// 链接器认为只喂给v_color的uniform/attribute全部"未使用"而丢弃->退回固定管线的默认白色）。
// 做法与Pangolin自己的Plotter一致(pango_plot/src/plotter.cpp)：只在GLES目标下才加这行。
const char* kFragmentShader =
#ifdef HAVE_GLES_2
    "precision mediump float;\n"
#endif  // HAVE_GLES_2
    R"(
varying vec4 v_color;
void main() {
    gl_FragColor = v_color;
}
)";

}  // namespace

GlColoredShader& GlColoredShader::Instance() {
    static GlColoredShader instance;
    return instance;
}

GlColoredShader::GlColoredShader() {
    prog_.AddShader(pangolin::GlSlVertexShader, kVertexShader);
    prog_.AddShader(pangolin::GlSlFragmentShader, kFragmentShader);
    prog_.Link();

    loc_position_ = prog_.GetAttributeHandle("a_position");
    loc_color_ = prog_.GetAttributeHandle("a_color");
}

void GlColoredShader::DrawVarying(const Eigen::Matrix4f& mvp, pangolin::GlBuffer& vbo_pos,
                                  pangolin::GlBuffer& vbo_color, size_t num_pts, GLenum mode, float point_size,
                                  float alpha_override) {
    if (num_pts == 0) {
        return;
    }

    prog_.SaveBind();
    prog_.SetUniform("u_mvp", mvp);
    prog_.SetUniform("u_use_uniform_color", 0.0f);
    prog_.SetUniform("u_point_size", point_size);
    prog_.SetUniform("u_alpha_override", alpha_override);

    vbo_pos.Bind();
    glVertexAttribPointer(loc_position_, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(loc_position_);
    vbo_pos.Unbind();

    vbo_color.Bind();
    glVertexAttribPointer(loc_color_, 4, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(loc_color_);
    vbo_color.Unbind();

    glDrawArrays(mode, 0, static_cast<GLsizei>(num_pts));

    glDisableVertexAttribArray(loc_position_);
    glDisableVertexAttribArray(loc_color_);
    prog_.Unbind();
}

void GlColoredShader::DrawUniform(const Eigen::Matrix4f& mvp, pangolin::GlBuffer& vbo_pos, size_t num_pts,
                                  GLenum mode, const Vec4f& color, float line_width) {
    if (num_pts == 0) {
        return;
    }

    glLineWidth(line_width);

    prog_.SaveBind();
    prog_.SetUniform("u_mvp", mvp);
    prog_.SetUniform("u_use_uniform_color", 1.0f);
    prog_.SetUniform("u_uniform_color", color[0], color[1], color[2], color[3]);
    prog_.SetUniform("u_point_size", line_width);
    prog_.SetUniform("u_alpha_override", -1.0f);

    vbo_pos.Bind();
    glVertexAttribPointer(loc_position_, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(loc_position_);
    vbo_pos.Unbind();

    glDrawArrays(mode, 0, static_cast<GLsizei>(num_pts));

    glDisableVertexAttribArray(loc_position_);
    prog_.Unbind();
}

}  // namespace lightning::ui
