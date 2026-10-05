#include "ui/Theme.h"

#include "core/Platform.h"

#include <optional>

namespace occlusa::ui {

namespace {
Palette gPalette;

constexpr ImVec4 rgb(int hex, float a = 1.0f)
{
    return ImVec4(((hex >> 16) & 0xFF) / 255.0f, ((hex >> 8) & 0xFF) / 255.0f, (hex & 0xFF) / 255.0f, a);
}
glm::vec3 rgb3(int hex)
{
    return {((hex >> 16) & 0xFF) / 255.0f, ((hex >> 8) & 0xFF) / 255.0f, (hex & 0xFF) / 255.0f};
}
} // namespace

ThemeMode themeModeFromString(const std::string& s)
{
    if (s == "light")
        return ThemeMode::Light;
    if (s == "dark")
        return ThemeMode::Dark;
    return ThemeMode::System;
}

std::string toString(ThemeMode m)
{
    switch (m) {
    case ThemeMode::Light: return "light";
    case ThemeMode::Dark: return "dark";
    default: return "system";
    }
}

bool resolveDark(ThemeMode mode, bool refresh)
{
    static std::optional<bool> cached;
    if (mode == ThemeMode::Light)
        return false;
    if (mode == ThemeMode::Dark)
        return true;
    if (!cached || refresh)
        cached = platform::systemPrefersDark().value_or(true);
    return *cached;
}

const Palette& palette()
{
    return gPalette;
}

ImU32 toU32(const ImVec4& c, float alphaMul)
{
    return ImGui::ColorConvertFloat4ToU32(ImVec4(c.x, c.y, c.z, c.w * alphaMul));
}

void applyTheme(bool dark, float scale)
{
    ImGuiStyle style;
    style.WindowRounding = 6.0f;
    style.ChildRounding = 6.0f;
    style.FrameRounding = 5.0f;
    style.PopupRounding = 6.0f;
    style.ScrollbarRounding = 8.0f;
    style.GrabRounding = 5.0f;
    style.TabRounding = 5.0f;
    style.WindowPadding = ImVec2(12, 12);
    style.FramePadding = ImVec2(9, 5);
    style.ItemSpacing = ImVec2(8, 7);
    style.ItemInnerSpacing = ImVec2(6, 5);
    style.CellPadding = ImVec2(8, 5);
    style.IndentSpacing = 18.0f;
    style.ScrollbarSize = 13.0f;
    style.GrabMinSize = 10.0f;
    style.WindowBorderSize = 1.0f;
    style.ChildBorderSize = 1.0f;
    style.PopupBorderSize = 1.0f;
    style.FrameBorderSize = dark ? 0.0f : 1.0f;
    style.TabBorderSize = 0.0f;
    style.WindowTitleAlign = ImVec2(0.0f, 0.5f);
    style.SeparatorTextBorderSize = 1.0f;
    style.DockingSeparatorSize = 2.0f;

    if (dark)
        ImGui::StyleColorsDark(&style);
    else
        ImGui::StyleColorsLight(&style);

    Palette& p = gPalette;
    p.dark = dark;
    ImVec4* c = style.Colors;
    if (dark) {
        p.accent = rgb(0x3B9EFF);
        p.accentHover = rgb(0x5BB0FF);
        p.accentActive = rgb(0x2A86E0);
        p.onAccent = rgb(0xFFFFFF);
        p.success = rgb(0x3FB97A);
        p.warning = rgb(0xE8A93A);
        p.danger = rgb(0xE5534B);
        p.text = rgb(0xE6E9ED);
        p.textMuted = rgb(0x8A939E);
        p.surface = rgb(0x23272D);
        p.surfaceAlt = rgb(0x2A2F36);
        p.border = rgb(0x363C44);
        p.viewportTop = rgb3(0x30363E);
        p.viewportBottom = rgb3(0x111316);
        p.sliceBackground = rgb3(0x0B0C0E);

        c[ImGuiCol_Text] = p.text;
        c[ImGuiCol_TextDisabled] = p.textMuted;
        c[ImGuiCol_WindowBg] = rgb(0x1C1F24);
        c[ImGuiCol_ChildBg] = rgb(0x1C1F24, 0.0f);
        c[ImGuiCol_PopupBg] = rgb(0x24282E);
        c[ImGuiCol_Border] = p.border;
        c[ImGuiCol_BorderShadow] = rgb(0, 0.0f);
        c[ImGuiCol_FrameBg] = rgb(0x2A2F36);
        c[ImGuiCol_FrameBgHovered] = rgb(0x333942);
        c[ImGuiCol_FrameBgActive] = rgb(0x3B424C);
        c[ImGuiCol_TitleBg] = rgb(0x17191D);
        c[ImGuiCol_TitleBgActive] = rgb(0x1B1E23);
        c[ImGuiCol_TitleBgCollapsed] = rgb(0x17191D);
        c[ImGuiCol_MenuBarBg] = rgb(0x17191D);
        c[ImGuiCol_ScrollbarBg] = rgb(0x1C1F24, 0.0f);
        c[ImGuiCol_ScrollbarGrab] = rgb(0x3A4049);
        c[ImGuiCol_ScrollbarGrabHovered] = rgb(0x4A515C);
        c[ImGuiCol_ScrollbarGrabActive] = rgb(0x58606C);
        c[ImGuiCol_CheckMark] = p.accent;
        c[ImGuiCol_SliderGrab] = p.accent;
        c[ImGuiCol_SliderGrabActive] = p.accentHover;
        c[ImGuiCol_Button] = rgb(0x2D333B);
        c[ImGuiCol_ButtonHovered] = rgb(0x3A424D);
        c[ImGuiCol_ButtonActive] = rgb(0x46505D);
        c[ImGuiCol_Header] = rgb(0x2B3A4C);
        c[ImGuiCol_HeaderHovered] = rgb(0x324A66);
        c[ImGuiCol_HeaderActive] = rgb(0x3A5A80);
        c[ImGuiCol_Separator] = p.border;
        c[ImGuiCol_SeparatorHovered] = p.accent;
        c[ImGuiCol_SeparatorActive] = p.accentHover;
        c[ImGuiCol_ResizeGrip] = rgb(0x3B9EFF, 0.15f);
        c[ImGuiCol_ResizeGripHovered] = rgb(0x3B9EFF, 0.5f);
        c[ImGuiCol_ResizeGripActive] = rgb(0x3B9EFF, 0.8f);
        c[ImGuiCol_Tab] = rgb(0x1F2328);
        c[ImGuiCol_TabHovered] = rgb(0x324A66);
        c[ImGuiCol_TabSelected] = rgb(0x2A3F57);
        c[ImGuiCol_TabSelectedOverline] = p.accent;
        c[ImGuiCol_TabDimmed] = rgb(0x1A1D21);
        c[ImGuiCol_TabDimmedSelected] = rgb(0x252B33);
        c[ImGuiCol_DockingPreview] = rgb(0x3B9EFF, 0.5f);
        c[ImGuiCol_DockingEmptyBg] = rgb(0x141619);
        c[ImGuiCol_PlotHistogram] = p.accent;
        c[ImGuiCol_TableHeaderBg] = rgb(0x23272D);
        c[ImGuiCol_TableBorderStrong] = p.border;
        c[ImGuiCol_TableBorderLight] = rgb(0x2C3138);
        c[ImGuiCol_TableRowBg] = rgb(0, 0.0f);
        c[ImGuiCol_TableRowBgAlt] = rgb(0xFFFFFF, 0.025f);
        c[ImGuiCol_TextSelectedBg] = rgb(0x3B9EFF, 0.35f);
        c[ImGuiCol_NavCursor] = p.accent;
        c[ImGuiCol_ModalWindowDimBg] = rgb(0x000000, 0.55f);
    } else {
        p.accent = rgb(0x1A73E8);
        p.accentHover = rgb(0x3B87EE);
        p.accentActive = rgb(0x135CBC);
        p.onAccent = rgb(0xFFFFFF);
        p.success = rgb(0x1E8E4E);
        p.warning = rgb(0xB7791F);
        p.danger = rgb(0xC9372C);
        p.text = rgb(0x1F2328);
        p.textMuted = rgb(0x6B7480);
        p.surface = rgb(0xFFFFFF);
        p.surfaceAlt = rgb(0xF0F3F6);
        p.border = rgb(0xD4D9DF);
        p.viewportTop = rgb3(0xF2F5F8);
        p.viewportBottom = rgb3(0xB4C0CC);
        p.sliceBackground = rgb3(0x14161A);

        c[ImGuiCol_Text] = p.text;
        c[ImGuiCol_TextDisabled] = p.textMuted;
        c[ImGuiCol_WindowBg] = rgb(0xF5F7F9);
        c[ImGuiCol_ChildBg] = rgb(0xFFFFFF, 0.0f);
        c[ImGuiCol_PopupBg] = rgb(0xFFFFFF);
        c[ImGuiCol_Border] = p.border;
        c[ImGuiCol_BorderShadow] = rgb(0, 0.0f);
        c[ImGuiCol_FrameBg] = rgb(0xFFFFFF);
        c[ImGuiCol_FrameBgHovered] = rgb(0xEEF3F8);
        c[ImGuiCol_FrameBgActive] = rgb(0xE2EAF3);
        c[ImGuiCol_TitleBg] = rgb(0xE9EDF1);
        c[ImGuiCol_TitleBgActive] = rgb(0xE3E8ED);
        c[ImGuiCol_TitleBgCollapsed] = rgb(0xE9EDF1);
        c[ImGuiCol_MenuBarBg] = rgb(0xEBEFF3);
        c[ImGuiCol_ScrollbarBg] = rgb(0xF5F7F9, 0.0f);
        c[ImGuiCol_ScrollbarGrab] = rgb(0xC5CCD4);
        c[ImGuiCol_ScrollbarGrabHovered] = rgb(0xAEB7C1);
        c[ImGuiCol_ScrollbarGrabActive] = rgb(0x97A2AE);
        c[ImGuiCol_CheckMark] = p.accent;
        c[ImGuiCol_SliderGrab] = p.accent;
        c[ImGuiCol_SliderGrabActive] = p.accentActive;
        c[ImGuiCol_Button] = rgb(0xE8ECF1);
        c[ImGuiCol_ButtonHovered] = rgb(0xDCE3EA);
        c[ImGuiCol_ButtonActive] = rgb(0xCFD8E2);
        c[ImGuiCol_Header] = rgb(0xDCE9FA);
        c[ImGuiCol_HeaderHovered] = rgb(0xCFE1F8);
        c[ImGuiCol_HeaderActive] = rgb(0xBDD6F5);
        c[ImGuiCol_Separator] = p.border;
        c[ImGuiCol_SeparatorHovered] = p.accent;
        c[ImGuiCol_SeparatorActive] = p.accentActive;
        c[ImGuiCol_ResizeGrip] = rgb(0x1A73E8, 0.12f);
        c[ImGuiCol_ResizeGripHovered] = rgb(0x1A73E8, 0.45f);
        c[ImGuiCol_ResizeGripActive] = rgb(0x1A73E8, 0.75f);
        c[ImGuiCol_Tab] = rgb(0xE6EAEF);
        c[ImGuiCol_TabHovered] = rgb(0xD4E4F8);
        c[ImGuiCol_TabSelected] = rgb(0xFFFFFF);
        c[ImGuiCol_TabSelectedOverline] = p.accent;
        c[ImGuiCol_TabDimmed] = rgb(0xE9EDF1);
        c[ImGuiCol_TabDimmedSelected] = rgb(0xF7F9FB);
        c[ImGuiCol_DockingPreview] = rgb(0x1A73E8, 0.4f);
        c[ImGuiCol_DockingEmptyBg] = rgb(0xE3E7EC);
        c[ImGuiCol_PlotHistogram] = p.accent;
        c[ImGuiCol_TableHeaderBg] = rgb(0xEDF1F5);
        c[ImGuiCol_TableBorderStrong] = p.border;
        c[ImGuiCol_TableBorderLight] = rgb(0xE3E7EC);
        c[ImGuiCol_TableRowBg] = rgb(0, 0.0f);
        c[ImGuiCol_TableRowBgAlt] = rgb(0x000000, 0.025f);
        c[ImGuiCol_TextSelectedBg] = rgb(0x1A73E8, 0.25f);
        c[ImGuiCol_NavCursor] = p.accent;
        c[ImGuiCol_ModalWindowDimBg] = rgb(0x20262E, 0.35f);
    }
    c[ImGuiCol_DragDropTarget] = p.accent;
    c[ImGuiCol_TextLink] = p.accent;
    c[ImGuiCol_InputTextCursor] = p.text;
    c[ImGuiCol_CheckboxSelectedBg] = c[ImGuiCol_FrameBg];
    c[ImGuiCol_TabDimmedSelectedOverline] = ImVec4(p.accent.x, p.accent.y, p.accent.z, 0.5f);
    c[ImGuiCol_PlotLines] = p.accent;
    c[ImGuiCol_PlotLinesHovered] = p.accentHover;
    c[ImGuiCol_PlotHistogramHovered] = p.accentHover;
    c[ImGuiCol_SeparatorActive] = p.accentActive;

    // Scan colours: stone/gingiva-like tones that stay readable over bone.
    p.scanColors[0] = rgb3(0x8DBBE6);
    p.scanColors[1] = rgb3(0xE9B98F);
    p.scanColors[2] = rgb3(0xB8E0B0);
    p.scanColors[3] = rgb3(0xE6B3D3);
    p.boneColor = rgb3(0xEDE3CF);

    style.ScaleAllSizes(scale);
    style.FontScaleDpi = scale;
    ImGui::GetStyle() = style;
}

} // namespace occlusa::ui
