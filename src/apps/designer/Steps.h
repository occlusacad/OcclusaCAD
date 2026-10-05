#pragma once

#include "apps/designer/Views.h"
#include "core/Workflow.h"

#include <memory>
#include <optional>
#include <string>

namespace occlusa::designer {

class DesignerApp;

enum class ViewId { Main3D, ScanPick, VolumePick, Axial, Coronal, Sagittal };

// Arrangement of the viewport area. Steps pick a default; the user can switch.
enum class ViewLayout { Standard, Alignment, Single3D, Slices };

// One tool/stage of a design workflow. Wizard mode walks steps in workflow order;
// expert mode can open any step directly.
class Step {
public:
    explicit Step(workflow::StepId id) : id_(id) {}
    virtual ~Step() = default;

    workflow::StepId id() const { return id_; }
    const workflow::StepInfo& info() const { return workflow::stepInfo(id_); }

    virtual void onEnter(DesignerApp&) {}
    virtual void onLeave(DesignerApp&) {}
    virtual void drawPanel(DesignerApp& app) = 0;
    // Reason the wizard cannot advance yet (nullopt = ready).
    virtual std::optional<std::string> blocker(const DesignerApp&) const { return std::nullopt; }
    virtual ViewLayout preferredLayout() const { return ViewLayout::Standard; }

    // Viewport integration.
    virtual OverlayFn overlay(DesignerApp&, ViewId) { return {}; }
    virtual void onViewEvent(DesignerApp&, ViewId, const ViewEvents&) {}

private:
    workflow::StepId id_;
};

std::unique_ptr<Step> makeLoadDataStep();
std::unique_ptr<Step> makeVolumeSetupStep();
std::unique_ptr<Step> makeScanAlignmentStep();
std::unique_ptr<Step> makeReviewStep();
std::unique_ptr<Step> makePlaceholderStep(workflow::StepId id);

} // namespace occlusa::designer
