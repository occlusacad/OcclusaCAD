#include "ui/App.h"

#include "core/Log.h"
#include "core/Platform.h"
#include "gfx/GL.h"
#include "ui/Fonts.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#if defined(_WIN32)
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#include <dwmapi.h>
#endif

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <ImGuizmo.h>

#include "ui/HeadlessContext.h"

#include <stb_image_write.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <vector>

namespace occlusa::ui {

namespace {
std::atomic<int> gActivity{0};
std::atomic<bool> gFocusChanged{false};

void onGlfwError(int code, const char* description)
{
    log::error("GLFW error {}: {}", code, description ? description : "");
}

void activityCursor(GLFWwindow*, double, double) { notifyActivity(); }
void activityButton(GLFWwindow*, int, int, int) { notifyActivity(); }
void activityScroll(GLFWwindow*, double, double) { notifyActivity(); }
void activityKey(GLFWwindow*, int, int, int, int) { notifyActivity(); }
void activityChar(GLFWwindow*, unsigned int) { notifyActivity(); }
void activitySize(GLFWwindow*, int, int) { notifyActivity(); }
void activityFocus(GLFWwindow*, int)
{
    gFocusChanged = true;
    notifyActivity();
}
void activityDrop(GLFWwindow*, int, const char**) { notifyActivity(); }

#if defined(_WIN32)
void applyWindowsTitleBar(GLFWwindow* window, bool dark)
{
    HWND hwnd = glfwGetWin32Window(window);
    BOOL value = dark ? TRUE : FALSE;
    // DWMWA_USE_IMMERSIVE_DARK_MODE = 20 (Windows 10 20H1+); harmless on older versions.
    DwmSetWindowAttribute(hwnd, 20, &value, sizeof(value));
}
#endif

} // namespace

void notifyActivity()
{
    gActivity.fetch_add(1);
    glfwPostEmptyEvent();
}

struct GuiApp::HeadlessState {
    GLuint fbo = 0, color = 0, depth = 0;
    int width = 0, height = 0;
};

GuiApp::GuiApp(AppOptions options) : options_(std::move(options))
{
    themeMode_ = options_.theme;
}

GuiApp::~GuiApp()
{
    delete headlessState_;
}

float GuiApp::detectScale() const
{
    if (options_.uiScale > 0.0f)
        return options_.uiScale;
    if (!window_)
        return 1.0f;
#if defined(__APPLE__)
    // macOS uses points; the framebuffer scale handles Retina.
    return 1.0f;
#else
    float xs = 1.0f, ys = 1.0f;
    glfwGetWindowContentScale(window_, &xs, &ys);
    return std::clamp(std::max(xs, ys), 1.0f, 4.0f);
#endif
}

bool GuiApp::initWindowed()
{
    glfwSetErrorCallback(onGlfwError);
    if (!glfwInit()) {
        log::error("Failed to initialise GLFW");
        return false;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE); // required on macOS
    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 0);
    window_ = glfwCreateWindow(options_.width, options_.height, options_.title.c_str(), nullptr, nullptr);
    if (!window_) {
        log::error("Failed to create a window with an OpenGL 3.3 core context");
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window_);
    // On Wayland, a vsynced eglSwapBuffers waits for the compositor's frame callback, and
    // compositors send none while the window is hidden (e.g. on another workspace). The main
    // thread would block and stop answering pings ("not responding"). FramePacer limits the
    // frame rate instead.
    glfwSwapInterval(glfwGetPlatform() == GLFW_PLATFORM_WAYLAND ? 0 : 1);
    if (GLFWmonitor* monitor = glfwGetPrimaryMonitor()) {
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);
        if (mode && mode->refreshRate > 0)
            pacer_.setMinFrameInterval(1.0 / mode->refreshRate);
    }
    if (!gfx::loadGL(reinterpret_cast<gfx::GLLoadProc>(glfwGetProcAddress))) {
        log::error("OpenGL 3.3 is required");
        return false;
    }
    // Activity callbacks first; ImGui chains to them.
    glfwSetCursorPosCallback(window_, activityCursor);
    glfwSetMouseButtonCallback(window_, activityButton);
    glfwSetScrollCallback(window_, activityScroll);
    glfwSetKeyCallback(window_, activityKey);
    glfwSetCharCallback(window_, activityChar);
    glfwSetFramebufferSizeCallback(window_, activitySize);
    glfwSetWindowFocusCallback(window_, activityFocus);
    glfwSetDropCallback(window_, activityDrop);
    return true;
}

bool GuiApp::initHeadless()
{
    if (!headless::createContext())
        return false;
    if (!gfx::loadGL(reinterpret_cast<gfx::GLLoadProc>(headless::getProcAddress)))
        return false;
    auto* hs = headlessState_ = new HeadlessState();
    hs->width = options_.width;
    hs->height = options_.height;
    glGenTextures(1, &hs->color);
    glBindTexture(GL_TEXTURE_2D, hs->color);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, hs->width, hs->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glGenRenderbuffers(1, &hs->depth);
    glBindRenderbuffer(GL_RENDERBUFFER, hs->depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, hs->width, hs->height);
    glGenFramebuffers(1, &hs->fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, hs->fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, hs->color, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, hs->depth);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return true;
}

void GuiApp::initImGui()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_DockingEnable;
    io.ConfigWindowsMoveFromTitleBarOnly = true;
    if (headless_) {
        io.IniFilename = nullptr;
    } else {
        std::error_code ec;
        std::filesystem::create_directories(platform::configDir(), ec);
        iniPath_ = platform::pathToUtf8(platform::configDir() / options_.iniFileName);
        io.IniFilename = iniPath_.c_str();
    }
    fonts::load();
    scale_ = detectScale();
    dark_ = resolveDark(themeMode_);
    applyTheme(dark_, scale_);
#if defined(_WIN32)
    if (window_)
        applyWindowsTitleBar(window_, dark_);
#endif

    if (window_)
        ImGui_ImplGlfw_InitForOpenGL(window_, true);
    else {
        io.DisplaySize = ImVec2(static_cast<float>(options_.width), static_cast<float>(options_.height));
        io.BackendPlatformName = "occlusacad_headless";
    }
    ImGui_ImplOpenGL3_Init("#version 330 core");
}

void GuiApp::setTitle(const std::string& title)
{
    if (window_)
        glfwSetWindowTitle(window_, title.c_str());
}

void GuiApp::setThemeMode(ThemeMode mode)
{
    themeMode_ = mode;
    const bool dark = resolveDark(mode, true);
    dark_ = dark;
    applyTheme(dark_, scale_);
#if defined(_WIN32)
    if (window_)
        applyWindowsTitleBar(window_, dark_);
#endif
    onThemeChanged();
    requestRedraw();
}

void GuiApp::requestRedraw(int frames)
{
    redrawFrames_ = std::max(redrawFrames_, frames);
    if (!headless_)
        glfwPostEmptyEvent();
}

void GuiApp::frame()
{
    if (window_) {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
    } else {
        ImGuiIO& io = ImGui::GetIO();
        io.DeltaTime = 1.0f / 60.0f;
        io.DisplaySize = ImVec2(static_cast<float>(options_.width), static_cast<float>(options_.height));
        ImGui_ImplOpenGL3_NewFrame();
    }
    ImGui::NewFrame();
    ImGuizmo::BeginFrame();
    onFrame();
    ImGui::Render();

    if (window_) {
        int w = 0, h = 0;
        glfwGetFramebufferSize(window_, &w, &h);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, w, h);
    } else {
        glBindFramebuffer(GL_FRAMEBUFFER, headlessState_->fbo);
        glViewport(0, 0, headlessState_->width, headlessState_->height);
    }
    const ImVec4 bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
    glClearColor(bg.x, bg.y, bg.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    if (window_)
        glfwSwapBuffers(window_);
    ++frameIndex_;
}

int GuiApp::run()
{
    headless_ = options_.screenshotPath.has_value();
    if (!(headless_ ? initHeadless() : initWindowed()))
        return 1;
    initImGui();
    try {
        onStart();
    } catch (const std::exception& e) {
        log::error("Startup failed: {}", e.what());
    }

    if (headless_) {
        // Render the requested number of frames, then keep going (bounded) until background work settles.
        int settle = 0;
        for (int i = 0; !quitRequested_; ++i) {
            frame();
            if (i < options_.screenshotFrames)
                continue;
            if (headlessBusy() && i < options_.screenshotFrames + 20000) {
                settle = 0;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            if (++settle > 10)
                break;
        }
        std::vector<unsigned char> pixels(static_cast<std::size_t>(headlessState_->width) * headlessState_->height * 4);
        glBindFramebuffer(GL_FRAMEBUFFER, headlessState_->fbo);
        glReadPixels(0, 0, headlessState_->width, headlessState_->height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
        stbi_flip_vertically_on_write(1);
        const std::string out = platform::pathToUtf8(*options_.screenshotPath);
        if (!stbi_write_png(out.c_str(), headlessState_->width, headlessState_->height, 4, pixels.data(), headlessState_->width * 4))
            log::error("Failed to write screenshot {}", out);
        else
            log::info("Screenshot written to {}", out);
        shutdown();
        return exitCode_;
    }

    int lastActivity = -1;
    while (!quitRequested_) {
        // Sleep until input arrives or the next frame is due.
        const double timeout = pacer_.waitTimeout(glfwGetTime(), redrawFrames_ > 0);
        if (timeout > 0.0)
            glfwWaitEventsTimeout(timeout);
        else
            glfwPollEvents();
        const int activity = gActivity.load();
        if (activity != lastActivity) {
            lastActivity = activity;
            redrawFrames_ = std::max(redrawFrames_, 3);
        }
        if (gFocusChanged.exchange(false) && themeMode_ == ThemeMode::System) {
            const bool dark = resolveDark(ThemeMode::System, true);
            if (dark != dark_)
                setThemeMode(ThemeMode::System);
        }
        if (glfwWindowShouldClose(window_)) {
            if (onCloseRequested())
                break;
            glfwSetWindowShouldClose(window_, GLFW_FALSE);
        }
        if (glfwGetWindowAttrib(window_, GLFW_ICONIFIED)) {
            glfwWaitEventsTimeout(0.2);
            continue;
        }
        // Waits can return without a reason to draw (Wayland wakes the loop after every frame).
        const double now = glfwGetTime();
        if (!pacer_.frameDue(now, redrawFrames_ > 0))
            continue;
        frame();
        pacer_.frameDrawn(now);
        if (redrawFrames_ > 0)
            --redrawFrames_;
    }
    shutdown();
    return exitCode_;
}

void GuiApp::shutdown()
{
    try {
        onShutdown();
    } catch (const std::exception& e) {
        log::error("Shutdown error: {}", e.what());
    }
    ImGui_ImplOpenGL3_Shutdown();
    if (window_)
        ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
    }
    if (headless_)
        headless::destroyContext();
}

} // namespace occlusa::ui
