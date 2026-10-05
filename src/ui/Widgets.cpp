#include "ui/Widgets.h"

#include "ui/Fonts.h"
#include "ui/Theme.h"

#include <imgui_internal.h>
#include <imgui_stdlib.h>

#include <chrono>
#include <cstdarg>
#include <deque>

namespace occlusa::ui {

bool primaryButton(const char* label, const ImVec2& size, bool enabled)
{
    const Palette& p = palette();
    ImGui::BeginDisabled(!enabled);
    ImGui::PushStyleColor(ImGuiCol_Button, p.accent);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, p.accentHover);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, p.accentActive);
    ImGui::PushStyleColor(ImGuiCol_Text, p.onAccent);
    ImGui::PushStyleColor(ImGuiCol_Border, p.accentActive);
    ImGui::PushFont(fonts::semibold(), 0.0f);
    const bool pressed = ImGui::Button(label, size);
    ImGui::PopFont();
    ImGui::PopStyleColor(5);
    ImGui::EndDisabled();
    return pressed && enabled;
}

bool dangerButton(const char* label, const ImVec2& size)
{
    const Palette& p = palette();
    ImGui::PushStyleColor(ImGuiCol_Button, p.danger);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(p.danger.x * 1.1f, p.danger.y * 1.1f, p.danger.z * 1.1f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(p.danger.x * 0.85f, p.danger.y * 0.85f, p.danger.z * 0.85f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Text, p.onAccent);
    const bool pressed = ImGui::Button(label, size);
    ImGui::PopStyleColor(4);
    return pressed;
}

bool button(const char* label, const ImVec2& size, bool enabled)
{
    ImGui::BeginDisabled(!enabled);
    const bool pressed = ImGui::Button(label, size);
    ImGui::EndDisabled();
    return pressed && enabled;
}

void heading(const char* text)
{
    fonts::Scope f(fonts::semibold(), fonts::kHeadingSize);
    ImGui::TextUnformatted(text);
}

void subheading(const char* text)
{
    fonts::Scope f(fonts::semibold(), 0.0f);
    ImGui::TextUnformatted(text);
}

void mutedText(const char* fmt, ...)
{
    ImGui::PushStyleColor(ImGuiCol_Text, palette().textMuted);
    va_list args;
    va_start(args, fmt);
    ImGui::TextV(fmt, args);
    va_end(args);
    ImGui::PopStyleColor();
}

void wrappedMutedText(const char* text)
{
    ImGui::PushStyleColor(ImGuiCol_Text, palette().textMuted);
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
}

void helpMarker(const char* text)
{
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void pill(const char* text, const ImVec4& color)
{
    const ImVec2 textSize = ImGui::CalcTextSize(text);
    const ImVec2 pad(8.0f * ImGui::GetStyle().FontScaleDpi, 2.0f * ImGui::GetStyle().FontScaleDpi);
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const ImVec2 size(textSize.x + pad.x * 2, textSize.y + pad.y * 2);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), toU32(color, 0.18f), size.y * 0.5f);
    dl->AddRect(pos, ImVec2(pos.x + size.x, pos.y + size.y), toU32(color, 0.55f), size.y * 0.5f);
    dl->AddText(ImVec2(pos.x + pad.x, pos.y + pad.y), toU32(color), text);
    ImGui::Dummy(size);
}

bool segmented(const char* id, const std::vector<const char*>& labels, int& current, float itemWidth)
{
    const Palette& p = palette();
    bool changed = false;
    ImGui::PushID(id);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));
    for (int i = 0; i < static_cast<int>(labels.size()); ++i) {
        if (i > 0)
            ImGui::SameLine();
        const bool selected = i == current;
        if (selected) {
            ImGui::PushStyleColor(ImGuiCol_Button, p.accent);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, p.accentHover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, p.accentActive);
            ImGui::PushStyleColor(ImGuiCol_Text, p.onAccent);
        }
        float rounding = ImGui::GetStyle().FrameRounding;
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, rounding);
        if (ImGui::Button(labels[i], ImVec2(itemWidth, 0)) && !selected) {
            current = i;
            changed = true;
        }
        ImGui::PopStyleVar();
        if (selected)
            ImGui::PopStyleColor(4);
    }
    ImGui::PopStyleVar();
    ImGui::PopID();
    return changed;
}

bool beginCard(const char* id, const ImVec2& size, bool autoHeight)
{
    const Palette& p = palette();
    ImGui::PushStyleColor(ImGuiCol_ChildBg, p.surface);
    ImGui::PushStyleColor(ImGuiCol_Border, p.border);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f * ImGui::GetStyle().FontScaleDpi);
    ImGuiChildFlags flags = ImGuiChildFlags_Borders | ImGuiChildFlags_AlwaysUseWindowPadding;
    if (autoHeight)
        flags |= ImGuiChildFlags_AutoResizeY;
    const bool open = ImGui::BeginChild(id, size, flags);
    return open;
}

void endCard()
{
    ImGui::EndChild();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(2);
}

void keyHint(const char* text)
{
    const Palette& p = palette();
    ImGui::PushStyleColor(ImGuiCol_Text, p.textMuted);
    const ImVec2 size = ImGui::CalcTextSize(text);
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const float pad = 4.0f * ImGui::GetStyle().FontScaleDpi;
    ImGui::GetWindowDrawList()->AddRect(pos, ImVec2(pos.x + size.x + 2 * pad, pos.y + size.y + pad), toU32(p.border), 3.0f);
    ImGui::SetCursorScreenPos(ImVec2(pos.x + pad, pos.y + pad * 0.5f));
    ImGui::TextUnformatted(text);
    ImGui::PopStyleColor();
}

bool labeledInput(const char* label, std::string& value, const char* hint, float width)
{
    ImGui::PushStyleColor(ImGuiCol_Text, palette().textMuted);
    ImGui::TextUnformatted(label);
    ImGui::PopStyleColor();
    ImGui::SetNextItemWidth(width);
    const std::string id = std::string("##") + label;
    return hint ? ImGui::InputTextWithHint(id.c_str(), hint, &value) : ImGui::InputText(id.c_str(), &value);
}

bool beginForm(const char* id, int columns)
{
    const ImVec2 spacing = ImGui::GetStyle().ItemSpacing;
    ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(spacing.x * 0.5f, spacing.y * 0.5f));
    const bool open = ImGui::BeginTable(id, columns, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_NoPadOuterX);
    if (!open)
        ImGui::PopStyleVar();
    return open;
}

void endForm()
{
    ImGui::EndTable();
    ImGui::PopStyleVar();
}

void formLabel(const char* label)
{
    ImGui::TableNextColumn();
    // Cells in a row share a text baseline; reset it so labels line up across columns.
    ImGui::GetCurrentWindow()->DC.CurrLineTextBaseOffset = 0.0f;
    ImGui::PushStyleColor(ImGuiCol_Text, palette().textMuted);
    ImGui::TextUnformatted(label);
    ImGui::PopStyleColor();
    ImGui::SetNextItemWidth(-FLT_MIN);
}

bool formField(const char* label, std::string& value, const char* hint)
{
    formLabel(label);
    const std::string id = std::string("##") + label;
    return hint ? ImGui::InputTextWithHint(id.c_str(), hint, &value) : ImGui::InputText(id.c_str(), &value);
}

// ---------------------------------------------------------------------------
// Modals
// ---------------------------------------------------------------------------

namespace {
struct ModalMessage {
    std::string title;
    std::string message;
    bool error;
};
std::deque<ModalMessage> gModals;

struct Toast {
    ToastKind kind;
    std::string message;
    std::chrono::steady_clock::time_point created;
};
std::deque<Toast> gToasts;
} // namespace

void showError(const std::string& title, const std::string& message)
{
    gModals.push_back({title, message, true});
}

void showInfo(const std::string& title, const std::string& message)
{
    gModals.push_back({title, message, false});
}

void drawModals()
{
    if (gModals.empty())
        return;
    const ModalMessage& m = gModals.front();
    const std::string id = m.title + "##occlusa_modal";
    if (!ImGui::IsPopupOpen(id.c_str()))
        ImGui::OpenPopup(id.c_str());
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSizeConstraints(ImVec2(ImGui::GetFontSize() * 22, 0), ImVec2(ImGui::GetFontSize() * 40, FLT_MAX));
    if (ImGui::BeginPopupModal(id.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (m.error) {
            ImGui::PushStyleColor(ImGuiCol_Text, palette().danger);
            subheading("Error");
            ImGui::PopStyleColor();
        }
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 38);
        ImGui::TextUnformatted(m.message.c_str());
        ImGui::PopTextWrapPos();
        ImGui::Spacing();
        if (primaryButton("OK", ImVec2(ImGui::GetFontSize() * 6, 0)) || ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            ImGui::CloseCurrentPopup();
            gModals.pop_front();
        }
        ImGui::EndPopup();
    }
}

void toast(ToastKind kind, const std::string& message)
{
    gToasts.push_back({kind, message, std::chrono::steady_clock::now()});
    while (gToasts.size() > 5)
        gToasts.pop_front();
}

void drawToasts()
{
    const auto now = std::chrono::steady_clock::now();
    while (!gToasts.empty() && now - gToasts.front().created > std::chrono::seconds(5))
        gToasts.pop_front();
    if (gToasts.empty())
        return;
    const Palette& p = palette();
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float margin = ImGui::GetFontSize() * 1.2f;
    float y = vp->WorkPos.y + vp->WorkSize.y - margin;
    int index = 0;
    for (auto it = gToasts.rbegin(); it != gToasts.rend(); ++it, ++index) {
        ImVec4 color = it->kind == ToastKind::Error ? p.danger : it->kind == ToastKind::Warning ? p.warning : it->kind == ToastKind::Success ? p.success : p.accent;
        const std::string id = "##toast" + std::to_string(index);
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x - margin, y), ImGuiCond_Always, ImVec2(1.0f, 1.0f));
        ImGui::SetNextWindowBgAlpha(0.97f);
        ImGui::PushStyleColor(ImGuiCol_Border, color);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.5f);
        ImGui::Begin(id.c_str(), nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoDocking);
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 26);
        ImGui::TextUnformatted(it->message.c_str());
        ImGui::PopTextWrapPos();
        y -= ImGui::GetWindowHeight() + margin * 0.4f;
        ImGui::End();
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
    }
}

} // namespace occlusa::ui
