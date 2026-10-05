#include "gfx/GlObjects.h"

#include "core/Log.h"

#include <stdexcept>
#include <vector>

namespace occlusa::gfx {

namespace {

GLuint compile(GLenum type, const char* src, const char* name)
{
    const GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetShaderiv(s, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> logBuf(static_cast<std::size_t>(std::max(len, 1)));
        glGetShaderInfoLog(s, len, nullptr, logBuf.data());
        glDeleteShader(s);
        throw std::runtime_error(std::string("Shader compile failed (") + name + "): " + logBuf.data());
    }
    return s;
}

} // namespace

ShaderProgram::ShaderProgram(const char* vs, const char* fs, const char* name)
{
    const GLuint v = compile(GL_VERTEX_SHADER, vs, name);
    const GLuint f = compile(GL_FRAGMENT_SHADER, fs, name);
    program_ = glCreateProgram();
    glAttachShader(program_, v);
    glAttachShader(program_, f);
    glLinkProgram(program_);
    glDeleteShader(v);
    glDeleteShader(f);
    GLint ok = 0;
    glGetProgramiv(program_, GL_LINK_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetProgramiv(program_, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> logBuf(static_cast<std::size_t>(std::max(len, 1)));
        glGetProgramInfoLog(program_, len, nullptr, logBuf.data());
        glDeleteProgram(program_);
        program_ = 0;
        throw std::runtime_error(std::string("Shader link failed (") + name + "): " + logBuf.data());
    }
}

ShaderProgram::~ShaderProgram()
{
    if (program_)
        glDeleteProgram(program_);
}

ShaderProgram::ShaderProgram(ShaderProgram&& o) noexcept : program_(o.program_), locations_(std::move(o.locations_))
{
    o.program_ = 0;
}

ShaderProgram& ShaderProgram::operator=(ShaderProgram&& o) noexcept
{
    if (this != &o) {
        if (program_)
            glDeleteProgram(program_);
        program_ = o.program_;
        locations_ = std::move(o.locations_);
        o.program_ = 0;
    }
    return *this;
}

GLint ShaderProgram::location(const char* name)
{
    auto it = locations_.find(name);
    if (it != locations_.end())
        return it->second;
    const GLint loc = glGetUniformLocation(program_, name);
    locations_.emplace(name, loc);
    return loc;
}

RenderTarget::~RenderTarget()
{
    release();
}

void RenderTarget::release()
{
    const GLuint fbos[] = {msaaFbo_, resolveFbo_, depthFbo_};
    for (GLuint f : fbos)
        if (f)
            glDeleteFramebuffers(1, &f);
    if (msaaColor_)
        glDeleteRenderbuffers(1, &msaaColor_);
    if (msaaDepth_)
        glDeleteRenderbuffers(1, &msaaDepth_);
    if (resolveColor_)
        glDeleteTextures(1, &resolveColor_);
    if (depthCopy_)
        glDeleteTextures(1, &depthCopy_);
    msaaFbo_ = resolveFbo_ = depthFbo_ = msaaColor_ = msaaDepth_ = resolveColor_ = depthCopy_ = 0;
    width_ = height_ = 0;
}

void RenderTarget::resize(int w, int h, int samples)
{
    w = std::max(w, 1);
    h = std::max(h, 1);
    GLint maxSamples = 1;
    glGetIntegerv(GL_MAX_SAMPLES, &maxSamples);
    samples = std::clamp(samples, 1, std::max(maxSamples, 1));
    if (w == width_ && h == height_ && samples == samples_ && msaaFbo_)
        return;
    release();
    width_ = w;
    height_ = h;
    samples_ = samples;

    glGenRenderbuffers(1, &msaaColor_);
    glBindRenderbuffer(GL_RENDERBUFFER, msaaColor_);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_RGBA8, w, h);
    glGenRenderbuffers(1, &msaaDepth_);
    glBindRenderbuffer(GL_RENDERBUFFER, msaaDepth_);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_DEPTH_COMPONENT24, w, h);
    glGenFramebuffers(1, &msaaFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, msaaFbo_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, msaaColor_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, msaaDepth_);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        log::error("Multisample framebuffer incomplete");

    glGenTextures(1, &resolveColor_);
    glBindTexture(GL_TEXTURE_2D, resolveColor_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glGenFramebuffers(1, &resolveFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, resolveFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, resolveColor_, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        log::error("Resolve framebuffer incomplete");

    glGenTextures(1, &depthCopy_);
    glBindTexture(GL_TEXTURE_2D, depthCopy_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, w, h, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glGenFramebuffers(1, &depthFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, depthFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthCopy_, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        log::error("Depth copy framebuffer incomplete");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void RenderTarget::bindForRendering() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, msaaFbo_);
    glViewport(0, 0, width_, height_);
}

void RenderTarget::copyDepth() const
{
    glBindFramebuffer(GL_READ_FRAMEBUFFER, msaaFbo_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, depthFbo_);
    glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_DEPTH_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, msaaFbo_);
}

void RenderTarget::resolve() const
{
    glBindFramebuffer(GL_READ_FRAMEBUFFER, msaaFbo_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, resolveFbo_);
    glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

GLuint emptyVao()
{
    static GLuint vao = 0;
    if (!vao)
        glGenVertexArrays(1, &vao);
    return vao;
}

} // namespace occlusa::gfx
