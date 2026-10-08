#pragma once

#include "apps/designer/CrownSteps.h"
#include "apps/designer/Document.h"
#include "apps/designer/Steps.h"
#include "apps/designer/Views.h"
#include "core/AppConfig.h"
#include "core/Dental.h"
#include "core/dicom/DicomSeries.h"
#include "db/CaseRepository.h"
#include "ui/App.h"
#include "ui/Task.h"

#include <deque>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace occlusa::designer {

struct DesignerOptions {
    std::optional<std::string> caseUuid;
    std::vector<std::filesystem::path> dicomPaths; // ad-hoc data from the command line
    std::vector<std::filesystem::path> scanPaths;
    std::optional<std::string> startStep;
    std::optional<gfx::ViewPreset> viewPreset; // initial 3D view direction (--view)
    std::optional<bool> showLeftPanel;  // --panels both|left|right|none (else the remembered state)
    std::optional<bool> showRightPanel;
    bool expandSteps = false;   // --expand-steps: open with the full step list shown
    bool expertMode = false;
    bool demoAutoAlign = false; // headless demo: run point-pair + ICP alignment automatically
    std::optional<std::filesystem::path> demoGroundTruth;
    bool demoSave = false;      // headless demo: save the design to the case after aligning
    double demoMaxError = 0.0;  // > 0: exit with code 3 if the registration error exceeds this (mm)
    std::optional<std::filesystem::path> demoCrownTruth; // headless demo: automatic crown design checked against this
    double demoMaxMarginError = 0.0;
    std::string demoLibrary;
};

// OcclusaCAD: the design application (Implant Studio style workflows).
class DesignerApp final : public ui::GuiApp {
public:
    DesignerApp(ui::AppOptions options, AppConfig config, DesignerOptions designerOptions);
    ~DesignerApp() override;

    // ---- Services for steps -------------------------------------------------
    Document& doc() { return doc_; }
    const Document& doc() const { return doc_; }
    ui::TaskRunner& tasks() { return tasks_; }
    gfx::SceneRenderer& renderer() { return *renderer_; }
    const AppConfig& config() const { return config_; }
    // Tooth numbering for display (the setting shared with OcclusaCAD DB); data stays FDI.
    dental::Numbering numbering() const { return dental::numberingFromString(config_.toothNumbering); }
    std::string toothText(int fdi) const { return dental::toothLabel(fdi, numbering()); }
    const DesignerOptions& designerOptions() const { return designerOptions_; }

    bool caseMode() const { return record_.has_value(); }
    db::CaseRecord* caseRecord() { return record_ ? &*record_ : nullptr; }
    db::ICaseRepository* repository() { return repo_.get(); }
    bool readOnly() const { return readOnly_; }
    std::filesystem::path resolveSource(const std::string& source) const;

    // Loading (asynchronous; queued while another task runs).
    void loadDicom(const std::filesystem::path& path, const std::string& source, std::optional<VolumeDisplay> display = {});
    void loadScan(const std::filesystem::path& path, const std::string& source, db::FileRole role, const std::string& label,
                  std::optional<SavedScan> saved = {});
    void enqueue(std::function<void()> action);
    bool loading() const { return tasks_.busy() || !queue_.empty(); }

    View3D& view3D(ViewId id);
    SliceView& sliceView(ViewId id);
    void setLayout(ViewLayout layout) { layout_ = layout; }
    ViewLayout layout() const { return layout_; }
    void fitAllViews();
    void centerCursorOn(const glm::dvec3& world);

    int selectedScan() const { return selectedScan_; }
    void selectScan(int id) { selectedScan_ = id; }

    void markModified() { doc_.touch(); }
    void saveDesign(bool finish);
    void exportDesign(); // registered scans and designed crowns to a folder (no case open)

    // Workflow
    const workflow::WorkflowDef& workflowDef() const { return *workflow_; }
    workflow::StepId currentStep() const { return current_; }
    void goToStep(workflow::StepId id);
    void markStepComplete(workflow::StepId id) { completed_.insert(id); }
    bool isStepComplete(workflow::StepId id) const { return completed_.count(id) != 0; }
    bool expertMode() const { return expert_; }

protected:
    void onStart() override;
    void onFrame() override;
    bool onCloseRequested() override;
    void onShutdown() override;
    bool headlessBusy() const override
    {
        return tasks_.busy() || !queue_.empty() || (designerOptions_.demoAutoAlign && demoState_ >= 0) || (crownDemo_ && crownDemo_->state < 100);
    }

private:
    void openCase();
    void restoreOrLoadCaseData();
    void placeUnregisteredScans();
    void processQueue();
    Step& step(workflow::StepId id);
    bool inWorkflow(workflow::StepId id) const;
    int workflowIndex(workflow::StepId id) const;
    void next();
    void back();
    DesignState captureState() const;

    // UI
    void setupDockLayout(unsigned int dockspaceId, ImVec2 size);
    void drawMenuBar();
    void drawToolbar(float height);
    void drawWorkflowPanel();
    void drawStepList();
    void drawStepHeader();
    void drawObjectsPanel();
    void drawViewports();
    void drawStatusBar(float height);
    void drawSeriesChooser();
    void drawLockPrompt();
    void drawClosePrompt();
    void drawHelp();
    void handleShortcuts();
    void runDemoAutoAlign();

    AppConfig config_;
    DesignerOptions designerOptions_;
    std::unique_ptr<db::ICaseRepository> repo_;
    std::optional<db::CaseRecord> record_;
    std::optional<DesignState> savedState_;
    bool readOnly_ = false;
    bool lockHeld_ = false;
    std::string lockHolder_;
    bool showLockPrompt_ = false;

    Document doc_;
    std::unique_ptr<gfx::SceneRenderer> renderer_;
    ui::TaskRunner tasks_;
    std::deque<std::function<void()>> queue_;

    // Views
    View3D main3D_{"3D"};
    View3D scanPick_{"Scan", ViewContent::ScanOnly};
    View3D volumePick_{"CBCT", ViewContent::VolumeOnly};
    SliceView axial_{SliceOrientation::Axial};
    SliceView coronal_{SliceOrientation::Coronal};
    SliceView sagittal_{SliceOrientation::Sagittal};
    ViewLayout layout_ = ViewLayout::Standard;
    SliceOrientation alignmentSlice_ = SliceOrientation::Axial;
    int selectedScan_ = 0;

    // Workflow
    const workflow::WorkflowDef* workflow_ = nullptr;
    workflow::StepId current_ = workflow::StepId::LoadData;
    std::set<workflow::StepId> completed_;
    std::map<workflow::StepId, std::unique_ptr<Step>> steps_;
    bool expert_ = false;
    bool firstFrame_ = true;
    bool showLeftPanel_ = true;   // workflow / step panel
    bool showRightPanel_ = true;  // objects and log
    bool stepsExpanded_ = false;  // full step list shown under the current-step header
    bool resetLayout_ = false;
    bool savedLeftPanel_ = true, savedRightPanel_ = true; // last state written to designer.ini

    // Pending UI state
    std::vector<dicom::SeriesInfo> seriesChoice_;
    std::string seriesChoiceSource_;
    std::optional<VolumeDisplay> seriesChoiceDisplay_;
    bool showClosePrompt_ = false;
    bool closeAfterSave_ = false;
    bool showHelp_ = false;
    int demoState_ = 0;
    std::optional<CrownDemo> crownDemo_;
};

} // namespace occlusa::designer
