#pragma once

#include "apps/designer/CrownSteps.h"
#include "apps/designer/Steps.h"

#include <filesystem>
#include <memory>
#include <vector>

namespace occlusa::designer {

class DesignerApp;

// Implant restorations: scan body alignment (with the implant library picker) and abutment design.
std::unique_ptr<Step> makeScanBodyAlignmentStep();
std::unique_ptr<Step> makeAbutmentDesignStep();

// Keeps the document's implant restorations in step with the case and the loaded scans.
void syncImplants(DesignerApp& app);
std::vector<SavedImplant> captureImplants(const DesignerApp& app);

// Finished abutments (designed part united with the library interface), in the coordinates of
// the scan holding the scan body.
std::vector<CrownExport> abutmentExports(DesignerApp& app);
// True when every designed abutment is part of abutmentExports().
bool allAbutmentsExported(const DesignerApp& app);

// Headless demo / end-to-end check: choose the truth file's implant connection, match the scan
// body from a click on its top, compare the implant position with the ground truth, design the
// default abutment and check that it is a closed solid.
struct AbutmentDemo {
    std::filesystem::path truthFile;
    double maxError = 0.0; // > 0: exit code 3 if the implant position is off by more (mm)
    bool save = false;
    int state = 0;
    int waitFrames = 0;
};
bool runAbutmentDemo(DesignerApp& app, AbutmentDemo& demo);

} // namespace occlusa::designer
