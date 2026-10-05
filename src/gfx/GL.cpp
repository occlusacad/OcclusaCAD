#define GLAD_GL_IMPLEMENTATION
#include "gfx/GL.h"

#include "core/Log.h"

namespace occlusa::gfx {

namespace {
GLLoadProc gLoader = nullptr;
GLADapiproc trampoline(const char* name)
{
    return reinterpret_cast<GLADapiproc>(gLoader(name));
}
} // namespace

bool loadGL(GLLoadProc loader)
{
    gLoader = loader;
    const int version = gladLoadGL(trampoline);
    if (version == 0)
        return false;
    log::info("OpenGL {}.{} - {} ({})", GLAD_VERSION_MAJOR(version), GLAD_VERSION_MINOR(version), glRendererString(), glVersionString());
    return GLAD_VERSION_MAJOR(version) > 3 || (GLAD_VERSION_MAJOR(version) == 3 && GLAD_VERSION_MINOR(version) >= 3);
}

const char* glRendererString()
{
    const auto* s = glGetString(GL_RENDERER);
    return s ? reinterpret_cast<const char*>(s) : "unknown";
}

const char* glVersionString()
{
    const auto* s = glGetString(GL_VERSION);
    return s ? reinterpret_cast<const char*>(s) : "unknown";
}

void checkGLError(const char* where)
{
    for (GLenum err = glGetError(); err != GL_NO_ERROR; err = glGetError())
        log::error("OpenGL error 0x{:04x} at {}", static_cast<unsigned>(err), where);
}

} // namespace occlusa::gfx
