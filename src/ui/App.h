#pragma once

#include "ui/FramePacer.h"
#include "ui/Theme.h"

#include <filesystem>
#include <optional>
#include <string>

struct GLFWwindow;

namespace occlusa::ui {

struct AppOptions {
    std::string title = "OcclusaCAD";
    int width = 1600;
    int height = 1000;
    std::string iniFileName = "imgui.ini";  // stored in the user config directory
    ThemeMode theme = ThemeMode::System;
    float uiScale = 0.0f;                    // 0 = automatic
    // Headless capture: render `screenshotFrames` frames off-screen and save a PNG (Linux/EGL only).
    std::optional<std::filesystem::path> screenshotPath;
    int screenshotFrames = 40;
};

// Base class for OcclusaCAD GUI applications: owns the window, OpenGL context and Dear ImGui.
class GuiApp {
public:
    explicit GuiApp(AppOptions options);
    virtual ~GuiApp();
    GuiApp(const GuiApp&) = delete;
    GuiApp& operator=(const GuiApp&) = delete;

    int run();

    void quit() { quitRequested_ = true; }
    void setExitCode(int code) { exitCode_ = code; }
    void setTitle(const std::string& title);
    void setThemeMode(ThemeMode mode);
    ThemeMode themeMode() const { return themeMode_; }
    bool darkTheme() const { return dark_; }
    float uiScale() const { return scale_; }
    // Keep rendering continuously (animations, progress) for the next frames.
    void requestRedraw(int frames = 2);
    bool headless() const { return headless_; }
    int frameIndex() const { return frameIndex_; }
    GLFWwindow* window() const { return window_; }

protected:
    virtual void onStart() {}
    virtual void onFrame() = 0;
    // Return false to keep the window open (e.g. to ask about unsaved changes first).
    virtual bool onCloseRequested() { return true; }
    virtual void onShutdown() {}
    // Called when the theme changes (palette() already updated).
    virtual void onThemeChanged() {}
    // Headless capture keeps rendering frames while this returns true (background work pending).
    virtual bool headlessBusy() const { return false; }

    AppOptions options_;

private:
    bool initWindowed();
    bool initHeadless();
    void initImGui();
    void frame();
    void shutdown();
    float detectScale() const;

    GLFWwindow* window_ = nullptr;
    bool headless_ = false;
    bool quitRequested_ = false;
    ThemeMode themeMode_ = ThemeMode::System;
    bool dark_ = true;
    float scale_ = 1.0f;
    int redrawFrames_ = 3;
    FramePacer pacer_{1.0 / 60.0};
    int frameIndex_ = 0;
    int exitCode_ = 0;
    std::string iniPath_;
    struct HeadlessState;
    HeadlessState* headlessState_ = nullptr;
};

// Called from GLFW callbacks; wakes the render loop.
void notifyActivity();

} // namespace occlusa::ui
