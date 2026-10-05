#pragma once

#include <imgui.h>

#include <glm/glm.hpp>
#include <string>

namespace occlusa::ui {

enum class ThemeMode { System, Light, Dark };

ThemeMode themeModeFromString(const std::string& s);
std::string toString(ThemeMode m);

// Semantic colours for custom drawing; switch with the theme.
struct Palette {
    bool dark = true;
    ImVec4 accent;
    ImVec4 accentHover;
    ImVec4 accentActive;
    ImVec4 onAccent;      // text on accent fills
    ImVec4 success;
    ImVec4 warning;
    ImVec4 danger;
    ImVec4 text;
    ImVec4 textMuted;
    ImVec4 surface;       // cards
    ImVec4 surfaceAlt;
    ImVec4 border;
    glm::vec3 viewportTop;
    glm::vec3 viewportBottom;
    glm::vec3 sliceBackground;
    glm::vec3 scanColors[4];
    glm::vec3 boneColor;
};

// Resolve System to Light/Dark using the OS preference (cached; refresh() re-queries).
bool resolveDark(ThemeMode mode, bool refresh = false);

// Apply the full ImGui style for light or dark mode, scaled for the given DPI factor.
void applyTheme(bool dark, float scale);
const Palette& palette();

ImU32 toU32(const ImVec4& c, float alphaMul = 1.0f);

} // namespace occlusa::ui
