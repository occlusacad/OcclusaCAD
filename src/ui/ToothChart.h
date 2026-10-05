#pragma once

#include "core/Dental.h"

#include <imgui.h>

#include <functional>
#include <map>
#include <string>

namespace occlusa::ui {

struct ToothChartStyle {
    float width = 0.0f; // 0 = available width
    dental::Numbering numbering = dental::Numbering::FDI;
    bool interactive = true;
};

// Arch-shaped FDI tooth chart. `assignments` maps FDI tooth -> restoration type key.
// Returns the tooth that was clicked this frame (0 if none).
int toothChart(const char* id, const std::map<int, std::string>& assignments, const ToothChartStyle& style = {});

} // namespace occlusa::ui
