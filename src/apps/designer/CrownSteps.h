#pragma once

#include "apps/designer/Steps.h"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace occlusa::designer {

class DesignerApp;

// Crown & bridge tools: margin line, insertion axis and crown design.
std::unique_ptr<Step> makeMarginLineStep();
std::unique_ptr<Step> makeInsertionAxisStep();
std::unique_ptr<Step> makeCrownDesignStep();

// Keeps the document's restorations in step with the case and the loaded scans
// (creates entries for the case's crowns, restores saved designs once their scans are loaded).
void syncRestorations(DesignerApp& app);
bool restorationsPending(const DesignerApp& app);

std::vector<SavedRestoration> captureRestorations(const DesignerApp& app);
std::vector<SavedBridge> captureBridges(const DesignerApp& app);

struct CrownExport {
    int tooth = 0;
    std::string label;                 // e.g. "Crown 46", "Bridge 47-46-45"
    std::string fileStem;              // e.g. "crown_46" / "crown_30" (named in the configured tooth numbering)
    std::shared_ptr<const Mesh> mesh;  // in the coordinates of the preparation scan file
    std::string header;                // STL header: names the tooth in both numbering systems
};
// Finished restorations: single crowns, and each complete bridge merged into one solid (its units
// are not exported separately). `merge` computes bridge unions that are out of date.
std::vector<CrownExport> crownExports(DesignerApp& app, bool merge);
// True when every designed restoration is part of crownExports() (all geometry is rebuilt).
bool allRestorationsExported(const DesignerApp& app);
// Case files written by OcclusaCAD for restorations (design/crown_*.stl, design/bridge_*.stl).
bool isGeneratedRestorationFile(const std::string& relativePath);

// Headless demo / end-to-end check: detect the margin at the ground-truth preparation point,
// set the axis, design the crown automatically and compare the margin with the ground truth.
// Called every frame; returns true while still running.
struct CrownDemo {
    std::filesystem::path truthFile;
    double maxMarginError = 0.0; // > 0: exit code 3 if exceeded (mm)
    bool save = false;
    int state = 0;
    int waitFrames = 0;
    std::size_t index = 0; // preparation being processed
    std::string library;   // tooth library for the designed restorations (empty = default)
};
bool runCrownDemo(DesignerApp& app, CrownDemo& demo);

} // namespace occlusa::designer
