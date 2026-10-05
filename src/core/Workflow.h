#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace occlusa::workflow {

// Every design step that exists in any OcclusaCAD workflow. Wizard mode walks the
// steps of the case's workflow in order; expert mode can jump to any step, including
// steps from other workflows.
enum class StepId {
    LoadData,
    VolumeSetup,
    ScanAlignment,
    PanoramicCurve,
    NerveCanal,
    VirtualTeeth,
    ImplantPlacement,
    SleeveSetup,
    GuideDesign,
    ScanBodyAlignment,
    MarginLine,
    InsertionAxis,
    CrownDesign,
    AbutmentDesign,
    Review,
};

struct StepInfo {
    StepId id;
    const char* key;
    const char* title;
    const char* group;
    const char* summary;   // one line shown in the step list
    const char* guidance;  // wizard instructions
    bool implemented;      // false = placeholder in this MVP
};

struct WorkflowDef {
    std::string key;
    std::string title;
    std::string description;
    std::vector<StepId> steps;
};

const std::vector<StepInfo>& allSteps();
const StepInfo& stepInfo(StepId id);
std::optional<StepId> stepFromKey(std::string_view key);

const std::vector<WorkflowDef>& allWorkflows();
const WorkflowDef* findWorkflow(std::string_view key);
const WorkflowDef& defaultWorkflow();

// Expert mode grouping (group title -> steps), in display order.
std::vector<std::pair<std::string, std::vector<StepId>>> stepGroups();

// Pick the workflow for a set of restoration type keys (most comprehensive wins).
std::string workflowForRestorations(const std::vector<std::string>& restorationTypeKeys);

} // namespace occlusa::workflow
