#include "ui/HeadlessContext.h"

#include "core/Log.h"

#if defined(OCCLUSACAD_HAVE_EGL)
#include <EGL/egl.h>
#include <EGL/eglext.h>
#endif

namespace occlusa::ui::headless {

#if defined(OCCLUSACAD_HAVE_EGL)
namespace {
EGLDisplay gDisplay = EGL_NO_DISPLAY;
EGLContext gContext = EGL_NO_CONTEXT;
EGLSurface gSurface = EGL_NO_SURFACE;
} // namespace

bool createContext()
{
    auto getPlatformDisplay = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
    if (getPlatformDisplay)
        gDisplay = getPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
    if (gDisplay == EGL_NO_DISPLAY)
        gDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (gDisplay == EGL_NO_DISPLAY || !eglInitialize(gDisplay, nullptr, nullptr)) {
        log::error("Headless: no EGL display");
        return false;
    }
    eglBindAPI(EGL_OPENGL_API);
    const EGLint cfgAttribs[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8,
                                 EGL_BLUE_SIZE, 8, EGL_DEPTH_SIZE, 24, EGL_NONE};
    EGLConfig config = nullptr;
    EGLint count = 0;
    if (!eglChooseConfig(gDisplay, cfgAttribs, &config, 1, &count) || count == 0) {
        log::error("Headless: no suitable EGL config");
        return false;
    }
    const EGLint ctxAttribs[] = {EGL_CONTEXT_MAJOR_VERSION, 3, EGL_CONTEXT_MINOR_VERSION, 3, EGL_CONTEXT_OPENGL_PROFILE_MASK,
                                 EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT, EGL_NONE};
    gContext = eglCreateContext(gDisplay, config, EGL_NO_CONTEXT, ctxAttribs);
    const EGLint pbAttribs[] = {EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE};
    gSurface = eglCreatePbufferSurface(gDisplay, config, pbAttribs);
    if (gContext == EGL_NO_CONTEXT || !eglMakeCurrent(gDisplay, gSurface, gSurface, gContext)) {
        log::error("Headless: cannot create an OpenGL 3.3 core EGL context");
        return false;
    }
    return true;
}

void* getProcAddress(const char* name)
{
    return reinterpret_cast<void*>(eglGetProcAddress(name));
}

void destroyContext()
{
    if (gDisplay == EGL_NO_DISPLAY)
        return;
    eglMakeCurrent(gDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (gContext != EGL_NO_CONTEXT)
        eglDestroyContext(gDisplay, gContext);
    if (gSurface != EGL_NO_SURFACE)
        eglDestroySurface(gDisplay, gSurface);
    eglTerminate(gDisplay);
    gDisplay = EGL_NO_DISPLAY;
}
#else
bool createContext()
{
    log::error("Headless rendering requires EGL (Linux builds only)");
    return false;
}
void* getProcAddress(const char*)
{
    return nullptr;
}
void destroyContext() {}
#endif

} // namespace occlusa::ui::headless
