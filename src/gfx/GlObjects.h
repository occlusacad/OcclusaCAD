#pragma once

#include "gfx/GL.h"

#include <glm/glm.hpp>

#include <string>
#include <unordered_map>

namespace occlusa::gfx {

class ShaderProgram {
public:
    ShaderProgram() = default;
    ShaderProgram(const char* vertexSrc, const char* fragmentSrc, const char* name);
    ~ShaderProgram();
    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;
    ShaderProgram(ShaderProgram&& o) noexcept;
    ShaderProgram& operator=(ShaderProgram&& o) noexcept;

    bool valid() const { return program_ != 0; }
    void use() const { glUseProgram(program_); }
    GLint location(const char* name);

    void set(const char* name, int v) { glUniform1i(location(name), v); }
    void set(const char* name, float v) { glUniform1f(location(name), v); }
    void set(const char* name, const glm::vec2& v) { glUniform2fv(location(name), 1, &v.x); }
    void set(const char* name, const glm::vec3& v) { glUniform3fv(location(name), 1, &v.x); }
    void set(const char* name, const glm::vec4& v) { glUniform4fv(location(name), 1, &v.x); }
    void set(const char* name, const glm::mat3& m) { glUniformMatrix3fv(location(name), 1, GL_FALSE, &m[0][0]); }
    void set(const char* name, const glm::mat4& m) { glUniformMatrix4fv(location(name), 1, GL_FALSE, &m[0][0]); }

private:
    GLuint program_ = 0;
    std::unordered_map<std::string, GLint> locations_;
};

// Off-screen render target: multisampled colour+depth for rendering, resolved into a
// single-sample colour texture that ImGui displays, plus a single-sample depth copy that
// the volume ray caster reads to stop rays at opaque surfaces.
class RenderTarget {
public:
    RenderTarget() = default;
    ~RenderTarget();
    RenderTarget(const RenderTarget&) = delete;
    RenderTarget& operator=(const RenderTarget&) = delete;

    void resize(int width, int height, int samples = 4);
    int width() const { return width_; }
    int height() const { return height_; }
    bool valid() const { return msaaFbo_ != 0; }

    void bindForRendering() const;
    // Copy the multisampled depth buffer into depthTexture().
    void copyDepth() const;
    // Resolve colour into colorTexture().
    void resolve() const;

    GLuint colorTexture() const { return resolveColor_; }
    GLuint depthTexture() const { return depthCopy_; }
    GLuint resolveFbo() const { return resolveFbo_; }

private:
    void release();
    int width_ = 0, height_ = 0, samples_ = 0;
    GLuint msaaFbo_ = 0, msaaColor_ = 0, msaaDepth_ = 0;
    GLuint resolveFbo_ = 0, resolveColor_ = 0;
    GLuint depthFbo_ = 0, depthCopy_ = 0;
};

// Empty VAO used for full-screen passes (vertices generated from gl_VertexID).
GLuint emptyVao();

} // namespace occlusa::gfx
