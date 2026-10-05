#pragma once

#include "apps/designer/Steps.h"
#include "core/Registration.h"

#include <optional>
#include <vector>

namespace occlusa::designer {

// Registers surface scans to the CBCT. The CBCT is the fixed reference: only the scan's
// transform is changed. Methods: landmark point pairs (Horn) with automatic ICP refinement,
// stand-alone ICP refinement, and manual adjustment (gizmo / nudge).
class ScanAlignmentStep final : public Step {
public:
    ScanAlignmentStep();

    void onEnter(DesignerApp& app) override;
    void onLeave(DesignerApp& app) override;
    void drawPanel(DesignerApp& app) override;
    std::optional<std::string> blocker(const DesignerApp& app) const override;
    ViewLayout preferredLayout() const override { return ViewLayout::Alignment; }
    OverlayFn overlay(DesignerApp& app, ViewId view) override;
    void onViewEvent(DesignerApp& app, ViewId view, const ViewEvents& events) override;

    // Programmatic access (used by automated demos/tests).
    void setLandmarks(DesignerApp& app, std::vector<glm::dvec3> scanLocal, std::vector<glm::dvec3> cbctWorld);
    void alignFromLandmarks(DesignerApp& app, bool refine);
    void refine(DesignerApp& app);

private:
    struct ScanSelection;
    ScanObject* activeScan(DesignerApp& app);
    void pushUndo(const ScanObject& scan);
    void applyTransform(DesignerApp& app, ScanObject& scan, const glm::dmat4& t, const std::string& method);
    void updateDeviation(DesignerApp& app);
    void drawLandmarkSection(DesignerApp& app, ScanObject& scan);
    void drawRefineSection(DesignerApp& app, ScanObject& scan);
    void drawManualSection(DesignerApp& app, ScanObject& scan);
    void drawVerification(DesignerApp& app, ScanObject& scan);
    std::optional<glm::dvec3> pickVolume(const DesignerApp& app, const Ray& ray) const;

    int scanId_ = 0;
    std::vector<glm::dvec3> scanPts_;
    std::vector<glm::dvec3> cbctPts_;
    std::vector<double> residuals_;
    std::vector<std::pair<int, glm::dmat4>> undo_;
    bool autoRefine_ = true;
    bool manual_ = false;
    int gizmoOp_ = 0;
    float nudgeMm_ = 0.25f;
    float nudgeDeg_ = 0.5f;
    bool gizmoWasUsing_ = false;
    IcpOptions icp_;
    double icpIso_ = 0.0;     // 0 = use the volume display threshold
    std::optional<DeviationStats> deviation_;
    std::optional<IcpResult> lastIcp_;
    int picking_ = 0; // 0 = scan point next, 1 = CBCT point next
};

} // namespace occlusa::designer
