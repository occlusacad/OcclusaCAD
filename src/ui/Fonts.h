#pragma once

#include <imgui.h>

namespace occlusa::ui::fonts {

constexpr float kBaseSize = 15.0f;
constexpr float kHeadingSize = 19.0f;
constexpr float kTitleSize = 24.0f;

// Load the embedded Inter fonts into the current ImGui context.
void load();

ImFont* regular();
ImFont* semibold();

// RAII font push helpers.
struct Scope {
    Scope(ImFont* font, float size) { ImGui::PushFont(font, size); }
    ~Scope() { ImGui::PopFont(); }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
};

} // namespace occlusa::ui::fonts
