#include "ui/ToothChart.h"

#include "ui/Fonts.h"
#include "ui/Theme.h"

#include <cmath>

namespace occlusa::ui {

namespace {
// Relative crown widths (central incisor .. third molar), roughly anatomical.
constexpr float kToothScale[9] = {0.0f, 0.86f, 0.74f, 0.80f, 0.78f, 0.78f, 1.0f, 0.95f, 0.88f};

ImVec4 typeColor(const std::string& key)
{
    if (const auto* t = dental::findRestorationType(key))
        return ImVec4(t->color[0], t->color[1], t->color[2], 1.0f);
    return palette().accent;
}

bool isImplantType(const std::string& key)
{
    const auto* t = dental::findRestorationType(key);
    return t && (std::string_view(t->category) == "Implant planning" || std::string_view(t->category) == "Implant restoration");
}
} // namespace

int toothChart(const char* id, const std::map<int, std::string>& assignments, const ToothChartStyle& style)
{
    const Palette& p = palette();
    const float width = style.width > 0.0f ? style.width : ImGui::GetContentRegionAvail().x;
    const float height = width * 0.82f;
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton(id, ImVec2(width, height));
    const bool hovered = ImGui::IsItemHovered();
    const bool clicked = style.interactive && ImGui::IsItemClicked(ImGuiMouseButton_Left);
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    ImDrawList* dl = ImGui::GetWindowDrawList();

    const float cx = origin.x + width * 0.5f;
    const float rx = width * 0.40f;
    const float ry = height * 0.38f;
    const float upperCy = origin.y + height * 0.455f;
    const float lowerCy = origin.y + height * 0.545f;
    const float baseR = width * 0.034f;
    int hoveredTooth = 0;
    int clickedTooth = 0;

    // Midline and jaw labels.
    dl->AddLine(ImVec2(cx, origin.y + height * 0.06f), ImVec2(cx, origin.y + height * 0.94f), toU32(p.border), 1.0f);
    {
        fonts::Scope f(fonts::semibold(), fonts::kBaseSize * 0.85f);
        const char* r = "R";
        const char* l = "L";
        dl->AddText(ImVec2(origin.x + 4, upperCy - ImGui::GetFontSize() * 0.5f), toU32(p.textMuted), r);
        dl->AddText(ImVec2(origin.x + width - ImGui::CalcTextSize(l).x - 4, upperCy - ImGui::GetFontSize() * 0.5f), toU32(p.textMuted), l);
    }

    auto drawArch = [&](const std::vector<int>& teeth, bool upper) {
        const int n = static_cast<int>(teeth.size());
        for (int i = 0; i < n; ++i) {
            const int fdi = teeth[i];
            // i = 0 is the patient's right third molar (left side of the chart).
            const float t = (static_cast<float>(i) + 0.5f) / static_cast<float>(n);
            const float theta = 3.14159265f * (1.0f - t);
            const float x = cx + rx * std::cos(theta);
            const float y = upper ? upperCy - ry * std::sin(theta) : lowerCy + ry * std::sin(theta);
            const float r = baseR * kToothScale[dental::positionInQuadrant(fdi)] * 1.25f;
            const ImVec2 c(x, y);
            const float dx = mouse.x - x, dy = mouse.y - y;
            const bool over = hovered && dx * dx + dy * dy <= r * r * 1.3f;
            if (over)
                hoveredTooth = fdi;

            auto it = assignments.find(fdi);
            const bool assigned = it != assignments.end();
            const ImVec4 fill = assigned ? typeColor(it->second) : p.surfaceAlt;
            dl->AddCircleFilled(c, r, toU32(fill, assigned ? 0.9f : 1.0f), 32);
            dl->AddCircle(c, r, over ? toU32(p.accent) : toU32(p.border), 32, over ? 2.5f : 1.2f);
            if (assigned && isImplantType(it->second)) {
                // Implant glyph: a small screw below the label.
                const float w = r * 0.35f;
                const ImU32 col = toU32(p.onAccent, 0.9f);
                for (int k = 0; k < 3; ++k) {
                    const float yy = y + r * 0.15f + k * r * 0.18f;
                    dl->AddLine(ImVec2(x - w, yy), ImVec2(x + w, yy + r * 0.08f), col, 1.2f);
                }
            }
            const std::string label = dental::toothLabel(fdi, style.numbering);
            fonts::Scope f(fonts::semibold(), fonts::kBaseSize * 0.85f);
            const ImVec2 ts = ImGui::CalcTextSize(label.c_str());
            const ImU32 textCol = assigned ? toU32(p.onAccent) : toU32(p.text);
            const float ty = (assigned && isImplantType(it->second)) ? y - ts.y * 0.85f : y - ts.y * 0.5f;
            dl->AddText(ImVec2(x - ts.x * 0.5f, ty), textCol, label.c_str());
        }
    };
    drawArch(dental::upperArch(), true);
    drawArch(dental::lowerArch(), false);

    if (hoveredTooth != 0) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
        if (ImGui::BeginTooltip()) {
            ImGui::Text("%s  (%s)", dental::toothName(hoveredTooth).c_str(), dental::toothLabel(hoveredTooth, style.numbering).c_str());
            auto it = assignments.find(hoveredTooth);
            if (it != assignments.end())
                if (const auto* t = dental::findRestorationType(it->second))
                    ImGui::TextColored(typeColor(it->second), "%s", t->label);
            ImGui::EndTooltip();
        }
        if (clicked)
            clickedTooth = hoveredTooth;
    }
    return clickedTooth;
}

} // namespace occlusa::ui
