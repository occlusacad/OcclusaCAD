#include "apps/designer/DesignerApp.h"

#include "apps/designer/StepScanAlignment.h"
#include "core/Log.h"
#include "core/Platform.h"
#include "core/StlIO.h"
#include "core/Time.h"
#include "db/LocalFileStore.h"
#include "gfx/GL.h"
#include "ui/FileDialog.h"
#include "ui/Fonts.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"

#include <imgui.h>
#include <imgui_internal.h>

#include <json.hpp>

#include <format>
#include <fstream>

namespace fs = std::filesystem;

namespace occlusa::designer {

using workflow::StepId;

namespace {
constexpr const char* kWorkflowWindow = "Workflow";
constexpr const char* kObjectsWindow = "Objects";
constexpr const char* kLogWindow = "Log";
constexpr const char* kViewportWindow = "Viewports";

std::string scanRoleLabel(db::FileRole r)
{
    return std::string(db::displayName(r));
}
} // namespace

namespace {

// Sidebar toggle drawn as a small window icon with the left or right pane filled when shown.
bool panelToggle(const char* id, bool left, bool shown, const char* what, const char* shortcut)
{
    const ui::Palette& pal = ui::palette();
    const float h = ImGui::GetFrameHeight();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton(id, ImVec2(h, h));
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    if (hovered)
        dl->AddRectFilled(p, ImVec2(p.x + h, p.y + h), ui::toU32(pal.surfaceAlt), ImGui::GetStyle().FrameRounding);
    const float s = ImGui::GetStyle().FontScaleDpi;
    const ImVec2 a(p.x + h * 0.2f, p.y + h * 0.25f), b(p.x + h * 0.8f, p.y + h * 0.75f);
    const ImU32 col = ui::toU32(hovered ? pal.text : pal.textMuted);
    const float pane = (b.x - a.x) * 0.36f;
    if (shown)
        dl->AddRectFilled(left ? a : ImVec2(b.x - pane, a.y), left ? ImVec2(a.x + pane, b.y) : b, ui::toU32(pal.accent), 2.0f * s);
    dl->AddRect(a, b, col, 2.0f * s, 1.5f * s);
    dl->AddLine(ImVec2(left ? a.x + pane : b.x - pane, a.y), ImVec2(left ? a.x + pane : b.x - pane, b.y), col, 1.5f * s);
    if (hovered)
        ImGui::SetTooltip("%s the %s (%s)", shown ? "Hide" : "Show", what, shortcut);
    return clicked;
}

} // namespace

DesignerApp::DesignerApp(ui::AppOptions options, AppConfig config, DesignerOptions designerOptions)
    : ui::GuiApp(std::move(options)), config_(std::move(config)), designerOptions_(std::move(designerOptions))
{
}

DesignerApp::~DesignerApp() = default;

// ---------------------------------------------------------------------------
// Startup / shutdown
// ---------------------------------------------------------------------------

void DesignerApp::onStart()
{
    renderer_ = std::make_unique<gfx::SceneRenderer>();
    // Tooth libraries: the lab's shared folder and the user's own.
    auto& libraries = crown::ToothLibraryRegistry::instance();
    if (!config_.dataRoot.empty())
        libraries.scanDirectory(config_.librariesRoot());
    libraries.scanDirectory(platform::configDir() / "libraries");
    if (!headless())
        ui::dialogs::init();
    expert_ = designerOptions_.expertMode;
    stepsExpanded_ = designerOptions_.expandSteps;
    // Sidebar visibility is remembered in designer.ini (per user, next to the window layout; read
    // by ImGui on the first frame).
    static ImGuiSettingsHandler panels;
    panels.TypeName = "OcclusaCAD";
    panels.TypeHash = ImHashStr("OcclusaCAD");
    panels.UserData = this;
    panels.ReadOpenFn = [](ImGuiContext*, ImGuiSettingsHandler*, const char* name) -> void* {
        return std::strcmp(name, "Panels") == 0 ? reinterpret_cast<void*>(1) : nullptr;
    };
    panels.ReadLineFn = [](ImGuiContext*, ImGuiSettingsHandler* h, void*, const char* line) {
        auto* app = static_cast<DesignerApp*>(h->UserData);
        if (std::strncmp(line, "Left=", 5) == 0)
            app->showLeftPanel_ = app->savedLeftPanel_ = std::atoi(line + 5) != 0;
        else if (std::strncmp(line, "Right=", 6) == 0)
            app->showRightPanel_ = app->savedRightPanel_ = std::atoi(line + 6) != 0;
    };
    panels.WriteAllFn = [](ImGuiContext*, ImGuiSettingsHandler* h, ImGuiTextBuffer* buf) {
        auto* app = static_cast<DesignerApp*>(h->UserData);
        buf->appendf("[%s][Panels]\nLeft=%d\nRight=%d\n\n", h->TypeName, app->showLeftPanel_ ? 1 : 0, app->showRightPanel_ ? 1 : 0);
        app->savedLeftPanel_ = app->showLeftPanel_;
        app->savedRightPanel_ = app->showRightPanel_;
    };
    ImGui::AddSettingsHandler(&panels);
    workflow_ = &workflow::defaultWorkflow();

    for (const auto& s : workflow::allSteps()) {
        std::unique_ptr<Step> st;
        switch (s.id) {
        case StepId::LoadData: st = makeLoadDataStep(); break;
        case StepId::VolumeSetup: st = makeVolumeSetupStep(); break;
        case StepId::ScanAlignment: st = makeScanAlignmentStep(); break;
        case StepId::Review: st = makeReviewStep(); break;
        case StepId::MarginLine: st = makeMarginLineStep(); break;
        case StepId::InsertionAxis: st = makeInsertionAxisStep(); break;
        case StepId::CrownDesign: st = makeCrownDesignStep(); break;
        default: st = makePlaceholderStep(s.id); break;
        }
        steps_[s.id] = std::move(st);
    }

    if (designerOptions_.caseUuid)
        openCase();
    if (designerOptions_.demoCrownTruth) {
        CrownDemo d;
        d.truthFile = *designerOptions_.demoCrownTruth;
        d.maxMarginError = designerOptions_.demoMaxMarginError;
        d.save = designerOptions_.demoSave;
        d.library = designerOptions_.demoLibrary;
        crownDemo_ = d;
    }

    for (const auto& p : designerOptions_.dicomPaths)
        loadDicom(p, platform::pathToUtf8(fs::absolute(p)));
    for (const auto& p : designerOptions_.scanPaths)
        loadScan(p, platform::pathToUtf8(fs::absolute(p)), db::FileRole::ScanOther, platform::pathToUtf8(p.stem()));

    StepId start = workflow_->steps.front();
    if (savedState_)
        if (auto s = workflow::stepFromKey(savedState_->currentStep))
            start = *s;
    if (designerOptions_.startStep)
        if (auto s = workflow::stepFromKey(*designerOptions_.startStep))
            start = *s;
    current_ = start;
    if (designerOptions_.viewPreset)
        main3D_.setPreset(*designerOptions_.viewPreset);
    step(current_).onEnter(*this);
    layout_ = step(current_).preferredLayout();
}

void DesignerApp::onShutdown()
{
    if (repo_ && record_ && lockHeld_) {
        try {
            repo_->releaseLock(record_->uuid, db::currentUserTag());
        } catch (const std::exception& e) {
            log::warn("Could not release case lock: {}", e.what());
        }
    }
    // GL objects must be destroyed while the context is alive.
    doc_.volume.reset();
    doc_.scans.clear();
    doc_.overlays.clear();
    doc_.restorations.clear();
    doc_.bridges.clear();
    renderer_.reset();
    if (!headless())
        ui::dialogs::shutdown();
}

bool DesignerApp::onCloseRequested()
{
    if (tasks_.busy())
        return false;
    if (doc_.modified && !readOnly_) {
        showClosePrompt_ = true;
        return false;
    }
    return true;
}

void DesignerApp::openCase()
{
    try {
        repo_ = db::openRepository(config_);
        record_ = repo_->loadCase(*designerOptions_.caseUuid);
        if (!record_) {
            ui::showError("Open case", "The case " + *designerOptions_.caseUuid + " does not exist in " + repo_->location());
            return;
        }
        if (const auto* wf = workflow::findWorkflow(record_->workflow))
            workflow_ = wf;
        const db::LockResult lock = repo_->acquireLock(record_->uuid, db::currentUserTag(), false);
        if (lock.acquired) {
            lockHeld_ = true;
        } else {
            lockHolder_ = lock.holder;
            readOnly_ = true;
            showLockPrompt_ = true;
        }
        if (auto js = repo_->loadDesignState(record_->uuid)) {
            try {
                savedState_ = DesignState::fromJson(*js);
                doc_.pendingRestorations = savedState_->restorations;
                doc_.pendingBridges = savedState_->bridges;
                for (const auto& k : savedState_->completedSteps)
                    if (auto s = workflow::stepFromKey(k))
                        completed_.insert(*s);
            } catch (const std::exception& e) {
                log::error("Saved design could not be read: {}", e.what());
            }
        }
        setTitle(std::format("OcclusaCAD - {} - {}", record_->caseNumber, record_->patientDisplayName()));
        restoreOrLoadCaseData();
    } catch (const std::exception& e) {
        record_.reset();
        ui::showError("Open case", e.what());
    }
}

fs::path DesignerApp::resolveSource(const std::string& source) const
{
    const fs::path p = platform::pathFromUtf8(source);
    if (p.is_absolute() || !record_ || !repo_)
        return p;
    return repo_->files().resolve(*record_, source);
}

void DesignerApp::restoreOrLoadCaseData()
{
    if (!record_)
        return;
    if (savedState_ && savedState_->volumeSource && !savedState_->volumeSource->empty()) {
        loadDicom(resolveSource(*savedState_->volumeSource), *savedState_->volumeSource, savedState_->volumeDisplay);
    } else if (const auto* f = record_->firstFile(db::FileRole::Dicom)) {
        loadDicom(resolveSource(f->relativePath), f->relativePath);
    }
    std::set<std::string> restored;
    if (savedState_) {
        for (const auto& s : savedState_->scans) {
            loadScan(resolveSource(s.source), s.source, s.role, s.label, s);
            restored.insert(s.source);
        }
    }
    for (const auto& f : record_->files)
        if (db::isScanRole(f.role) && !restored.count(f.relativePath))
            loadScan(resolveSource(f.relativePath), f.relativePath, f.role, f.label.empty() ? scanRoleLabel(f.role) : f.label);
}

// ---------------------------------------------------------------------------
// Loading
// ---------------------------------------------------------------------------

void DesignerApp::enqueue(std::function<void()> action)
{
    queue_.push_back(std::move(action));
}

void DesignerApp::processQueue()
{
    while (!tasks_.busy() && !queue_.empty() && seriesChoice_.empty()) {
        auto action = std::move(queue_.front());
        queue_.pop_front();
        action();
    }
}

void DesignerApp::loadDicom(const fs::path& path, const std::string& source, std::optional<VolumeDisplay> display)
{
    enqueue([this, path, source, display] {
        tasks_.start("Loading CBCT", [this, path, source, display](const ProgressFn& progress) -> ui::TaskRunner::Continuation {
            auto scan = dicom::scanForSeries(path, [&](float f, const std::string& m) { return progress(f * 0.2f, m); });
            std::vector<dicom::SeriesInfo> supported;
            for (auto& s : scan.series)
                if (s.supported)
                    supported.push_back(s);
            if (supported.empty()) {
                if (!scan.series.empty())
                    throw std::runtime_error("The DICOM data uses an unsupported compression: " + scan.series.front().transferSyntax);
                throw std::runtime_error("No DICOM image series found in " + platform::pathToUtf8(path));
            }
            if (supported.size() > 1) {
                return [this, supported, source, display] {
                    seriesChoice_ = supported;
                    seriesChoiceSource_ = source;
                    seriesChoiceDisplay_ = display;
                };
            }
            dicom::LoadReport report;
            auto vol = std::make_shared<Volume>(
                dicom::loadSeries(supported.front(), [&](float f, const std::string& m) { return progress(0.2f + f * 0.8f, m); }, &report));
            return [this, vol, source, display, report] {
                const bool wasModified = doc_.modified;
                VolumeObject vo;
                vo.volume = vol;
                vo.gpu = std::make_unique<gfx::GpuVolume>(*vol);
                vo.source = source;
                const auto& d = vol->geometry.dims;
                vo.label = std::format("{} ({}x{}x{})", vol->info.seriesDescription.empty() ? "CBCT" : vol->info.seriesDescription, d.x, d.y, d.z);
                VolumeDisplay disp;
                const auto [wc, ww] = vol->suggestWindow();
                disp.windowCenter = wc;
                disp.windowWidth = ww;
                disp.isoValue = vol->suggestBoneThreshold();
                disp.color = ui::palette().boneColor;
                if (display)
                    disp = *display;
                vo.display = disp;
                doc_.volume = std::move(vo);
                doc_.cursor = vol->geometry.worldCenter();
                if (savedState_ && vol->containsWorld(savedState_->cursor))
                    doc_.cursor = savedState_->cursor;
                placeUnregisteredScans();
                fitAllViews();
                doc_.touch();
                doc_.modified = wasModified;
                for (const auto& w : report.warnings)
                    ui::toast(ui::ToastKind::Warning, w);
                log::info("CBCT loaded: {} - suggested threshold {:.0f}", doc_.volume->label, doc_.volume->display.isoValue);
            };
        });
    });
}

void DesignerApp::loadScan(const fs::path& path, const std::string& source, db::FileRole role, const std::string& label, std::optional<SavedScan> saved)
{
    enqueue([this, path, source, role, label, saved] {
        tasks_.start("Loading scan", [this, path, source, role, label, saved](const ProgressFn& progress) -> ui::TaskRunner::Continuation {
            progress(0.1f, "Reading " + platform::pathToUtf8(path.filename()));
            auto mesh = std::make_shared<Mesh>(readStl(path));
            progress(1.0f, "Uploading");
            return [this, mesh, source, role, label, saved] {
                const bool wasModified = doc_.modified;
                ScanObject& s = doc_.addScan(mesh, source, label, role);
                s.gpu = std::make_unique<gfx::GpuMesh>(*mesh);
                s.color = ui::palette().scanColors[(doc_.scans.size() - 1) % 4];
                if (saved) {
                    s.setTransform(saved->transform);
                    s.initialTransform = saved->transform;
                    s.color = saved->color;
                    s.opacity = saved->opacity;
                    s.visible = saved->visible;
                    s.registration = saved->registration;
                } else {
                    placeUnregisteredScans();
                }
                if (selectedScan_ == 0)
                    selectedScan_ = s.id;
                fitAllViews();
                doc_.modified = wasModified;
                log::info("Scan loaded: {} ({} triangles)", label, mesh->triangleCount());
            };
        });
    });
}

void DesignerApp::placeUnregisteredScans()
{
    // Before registration a scan's coordinates are arbitrary (scanner space). Park it at the
    // centre of the CBCT so both are visible; the registration step computes the real pose.
    if (!doc_.volume)
        return;
    const glm::dvec3 center = doc_.volume->volume->geometry.worldCenter();
    for (auto& s : doc_.scans) {
        if (s->registration.registered || s->transform != glm::dmat4(1.0))
            continue;
        const glm::dvec3 c = s->mesh->bounds().center();
        s->setTransform(glm::translate(glm::dmat4(1.0), center - c));
        s->initialTransform = s->transform;
    }
}

// ---------------------------------------------------------------------------
// Views
// ---------------------------------------------------------------------------

View3D& DesignerApp::view3D(ViewId id)
{
    switch (id) {
    case ViewId::ScanPick: return scanPick_;
    case ViewId::VolumePick: return volumePick_;
    default: return main3D_;
    }
}

SliceView& DesignerApp::sliceView(ViewId id)
{
    switch (id) {
    case ViewId::Coronal: return coronal_;
    case ViewId::Sagittal: return sagittal_;
    default: return axial_;
    }
}

void DesignerApp::fitAllViews()
{
    main3D_.requestFit();
    scanPick_.requestFit();
    volumePick_.requestFit();
    axial_.requestFit();
    coronal_.requestFit();
    sagittal_.requestFit();
}

void DesignerApp::centerCursorOn(const glm::dvec3& world)
{
    if (doc_.volume) {
        const Aabb b = doc_.volume->volume->geometry.worldBounds();
        doc_.cursor = glm::clamp(world, b.min, b.max);
    }
}

// ---------------------------------------------------------------------------
// Workflow
// ---------------------------------------------------------------------------

Step& DesignerApp::step(StepId id)
{
    return *steps_.at(id);
}

bool DesignerApp::inWorkflow(StepId id) const
{
    return workflowIndex(id) >= 0;
}

int DesignerApp::workflowIndex(StepId id) const
{
    for (std::size_t i = 0; i < workflow_->steps.size(); ++i)
        if (workflow_->steps[i] == id)
            return static_cast<int>(i);
    return -1;
}

void DesignerApp::goToStep(StepId id)
{
    if (id == current_)
        return;
    step(current_).onLeave(*this);
    current_ = id;
    step(current_).onEnter(*this);
    layout_ = step(current_).preferredLayout();
    requestRedraw();
}

void DesignerApp::next()
{
    if (step(current_).blocker(*this))
        return;
    completed_.insert(current_);
    const int idx = workflowIndex(current_);
    if (idx >= 0 && idx + 1 < static_cast<int>(workflow_->steps.size()))
        goToStep(workflow_->steps[static_cast<std::size_t>(idx + 1)]);
    else if (idx < 0)
        goToStep(workflow_->steps.front());
}

void DesignerApp::back()
{
    const int idx = workflowIndex(current_);
    if (idx > 0)
        goToStep(workflow_->steps[static_cast<std::size_t>(idx - 1)]);
}

// ---------------------------------------------------------------------------
// Saving
// ---------------------------------------------------------------------------

DesignState DesignerApp::captureState() const
{
    DesignState st;
    st.workflow = workflow_->key;
    st.currentStep = workflow::stepInfo(current_).key;
    for (StepId s : completed_)
        st.completedSteps.emplace_back(workflow::stepInfo(s).key);
    st.cursor = doc_.cursor;
    if (doc_.volume) {
        st.volumeSource = doc_.volume->source;
        st.volumeDisplay = doc_.volume->display;
    }
    for (const auto& s : doc_.scans) {
        SavedScan ss;
        ss.source = s->source;
        ss.label = s->label;
        ss.role = s->role;
        ss.transform = s->transform;
        ss.color = s->color;
        ss.opacity = s->opacity;
        ss.visible = s->visible;
        ss.registration = s->registration;
        st.scans.push_back(ss);
    }
    st.numbering = numbering();
    st.restorations = captureRestorations(*this);
    st.bridges = captureBridges(*this);
    return st;
}

void DesignerApp::saveDesign(bool finish)
{
    if (!record_ || !repo_) {
        exportDesign();
        return;
    }
    if (readOnly_) {
        ui::showError("Save", "This case is open read-only because it is locked by " + lockHolder_ + ".");
        return;
    }
    // Snapshot what the worker needs; the worker only touches files, never the document.
    struct ScanExport {
        int id;
        std::shared_ptr<const Mesh> mesh;
        glm::dmat4 transform;
        std::string label;
        std::string source;
        db::FileRole role;
        bool registered;
    };
    std::vector<ScanExport> scans;
    for (const auto& s : doc_.scans)
        scans.push_back({s->id, s->mesh, s->transform, s->label, s->source, s->role, s->registration.registered});
    const std::vector<CrownExport> crowns = crownExports(*this, true);
    // Outputs from an earlier save (e.g. under the other tooth numbering) are replaced, but only when
    // every designed restoration is exported now, so nothing is lost if one is not rebuilt yet.
    const bool replaceOldOutputs = allRestorationsExported(*this);
    const std::optional<std::string> volumeSource = doc_.volume ? std::optional<std::string>(doc_.volume->source) : std::nullopt;
    const db::CaseRecord record = *record_;
    db::ICaseRepository* repo = repo_.get();

    tasks_.start("Saving design", [=, this](const ProgressFn& progress) -> ui::TaskRunner::Continuation {
        // 1. Data loaded from outside the case is copied into the case folder.
        std::map<std::string, std::string> imported; // old source -> case-relative path
        std::vector<db::CaseFile> newFiles;
        if (volumeSource && platform::pathFromUtf8(*volumeSource).is_absolute()) {
            const fs::path p = platform::pathFromUtf8(*volumeSource);
            progress(0.05f, "Copying CBCT into the case");
            const std::string rel = fs::is_directory(p) ? repo->files().importDirectory(record, p, db::FileRole::Dicom)
                                                        : repo->files().importFile(record, p, db::FileRole::Dicom);
            imported[*volumeSource] = rel;
            newFiles.push_back(db::CaseFile{0, db::FileRole::Dicom, rel, "CBCT", time::nowUtcIso8601()});
        }
        for (const auto& s : scans) {
            if (!platform::pathFromUtf8(s.source).is_absolute())
                continue;
            progress(0.3f, "Copying " + s.label + " into the case");
            const std::string rel = repo->files().importFile(record, platform::pathFromUtf8(s.source), s.role);
            imported[s.source] = rel;
            newFiles.push_back(db::CaseFile{0, s.role, rel, s.label, time::nowUtcIso8601()});
        }
        // 2. Registered scans exported in CBCT coordinates for downstream tools.
        std::vector<db::CaseFile> outputs;
        const fs::path designDir = repo->files().designDirectory(record);
        for (std::size_t i = 0; i < scans.size(); ++i) {
            const auto& s = scans[i];
            if (!s.registered)
                continue;
            progress(0.5f + 0.4f * static_cast<float>(i) / static_cast<float>(scans.size()), "Exporting " + s.label);
            const fs::path out = designDir / platform::pathFromUtf8(db::LocalFileStore::sanitizeFileName(s.label) + "_cbct_aligned.stl");
            writeStlBinary(out, *s.mesh, s.transform, "OcclusaCAD: scan registered to CBCT (LPS mm)");
            outputs.push_back(db::CaseFile{0, db::FileRole::DesignOutput, repo->files().relativize(record, out), s.label + " (aligned to CBCT)",
                                           time::nowUtcIso8601()});
        }
        // 3. Restorations, in the coordinates of the scan they were designed on.
        for (const auto& c : crowns) {
            progress(0.92f, "Exporting " + c.label);
            const fs::path out = designDir / platform::pathFromUtf8(c.fileStem + ".stl");
            writeStlBinary(out, *c.mesh, glm::dmat4(1.0), c.header);
            outputs.push_back(db::CaseFile{0, db::FileRole::DesignOutput, repo->files().relativize(record, out), c.label, time::nowUtcIso8601()});
        }
        std::vector<std::string> stale;
        if (replaceOldOutputs)
            for (const auto& f : record.files) {
                if (f.role != db::FileRole::DesignOutput || !isGeneratedRestorationFile(f.relativePath))
                    continue;
                if (std::any_of(outputs.begin(), outputs.end(), [&](const db::CaseFile& o) { return o.relativePath == f.relativePath; }))
                    continue;
                std::error_code ec;
                fs::remove(repo->files().resolve(record, f.relativePath), ec);
                stale.push_back(f.relativePath);
            }
        progress(1.0f, "Updating case database");
        return [this, imported, newFiles, outputs, stale, finish] {
            // Apply new sources to the document, then persist state + record on the UI thread.
            for (auto& s : doc_.scans)
                if (auto it = imported.find(s->source); it != imported.end())
                    s->source = it->second;
            if (doc_.volume)
                if (auto it = imported.find(doc_.volume->source); it != imported.end())
                    doc_.volume->source = it->second;
            if (finish)
                completed_.insert(current_);
            const DesignState state = captureState();
            try {
                repo_->saveDesignState(record_->uuid, state.toJson(), db::currentUserTag());
                // Re-read the record so concurrent edits in OcclusaCAD DB (e.g. notes) are preserved.
                auto fresh = repo_->loadCase(record_->uuid);
                if (!fresh)
                    throw std::runtime_error("The case was deleted");
                for (const auto& f : newFiles)
                    fresh->files.push_back(f);
                std::erase_if(fresh->files, [&](const db::CaseFile& f) { return std::find(stale.begin(), stale.end(), f.relativePath) != stale.end(); });
                for (const auto& o : outputs) {
                    auto it = std::find_if(fresh->files.begin(), fresh->files.end(), [&](const db::CaseFile& f) { return f.relativePath == o.relativePath; });
                    if (it == fresh->files.end())
                        fresh->files.push_back(o);
                    else
                        it->addedUtc = o.addedUtc;
                }
                if (finish)
                    fresh->status = db::CaseStatus::Designed;
                else if (fresh->status == db::CaseStatus::New)
                    fresh->status = db::CaseStatus::InDesign;
                repo_->updateCase(*fresh);
                record_ = std::move(fresh);
                doc_.modified = false;
                ui::toast(ui::ToastKind::Success, finish ? "Design finished and saved to " + record_->caseNumber : "Design saved to " + record_->caseNumber);
                if (closeAfterSave_)
                    quit();
            } catch (const std::exception& e) {
                closeAfterSave_ = false;
                ui::showError("Save design", e.what());
            }
        };
    });
}

void DesignerApp::exportDesign()
{
    const auto crowns = crownExports(*this, true);
    bool anyScan = false;
    for (const auto& s : doc_.scans)
        anyScan |= s->registration.registered;
    if (!anyScan && crowns.empty()) {
        ui::showError("Export", "There is nothing to export yet: no scan is aligned to a CBCT and no crown is designed.");
        return;
    }
    auto dir = ui::dialogs::pickFolder();
    if (!dir)
        return;
    int count = 0;
    try {
        for (const auto& s : doc_.scans) {
            if (!s->registration.registered)
                continue;
            const fs::path out = *dir / platform::pathFromUtf8(db::LocalFileStore::sanitizeFileName(s->label) + "_cbct_aligned.stl");
            writeStlBinary(out, *s->mesh, s->transform, "OcclusaCAD: scan registered to CBCT (LPS mm)");
            ++count;
        }
        for (const auto& c : crowns) {
            writeStlBinary(*dir / platform::pathFromUtf8(c.fileStem + ".stl"), *c.mesh, glm::dmat4(1.0), c.header);
            ++count;
        }
        ui::toast(ui::ToastKind::Success, std::format("Exported {} file(s)", count));
    } catch (const std::exception& e) {
        ui::showError("Export", e.what());
    }
}

// ---------------------------------------------------------------------------
// Frame
// ---------------------------------------------------------------------------

void DesignerApp::onFrame()
{
    processQueue();
    handleShortcuts();
    drawMenuBar();

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float toolbarH = ImGui::GetFrameHeight() * 2.0f;
    const float statusH = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 0.8f;
    drawToolbar(toolbarH);

    // Dock space between toolbar and status bar.
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x, vp->WorkPos.y + toolbarH));
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, vp->WorkSize.y - toolbarH - statusH));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::Begin("##dockhost", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoNavFocus);
    ImGui::PopStyleVar(3);
    // v2: 25% / 50% / 25% default (a new id so layouts saved by older versions are replaced).
    const ImGuiID dockId = ImGui::GetID("OcclusaDock.v2");
    if (firstFrame_ || resetLayout_) {
        ImGuiDockNode* node = ImGui::DockBuilderGetNode(dockId);
        if (resetLayout_ || !node || !node->IsSplitNode() || headless())
            setupDockLayout(dockId);
        if (resetLayout_)
            showLeftPanel_ = showRightPanel_ = true;
        resetLayout_ = false;
    }
    ImGui::DockSpace(dockId, ImVec2(0, 0), ImGuiDockNodeFlags_None);
    ImGui::End();

    if (firstFrame_) {
        // The command line wins over the remembered sidebar state (loaded by now).
        if (designerOptions_.showLeftPanel)
            showLeftPanel_ = *designerOptions_.showLeftPanel;
        if (designerOptions_.showRightPanel)
            showRightPanel_ = *designerOptions_.showRightPanel;
    }
    if (showLeftPanel_ != savedLeftPanel_ || showRightPanel_ != savedRightPanel_)
        ImGui::MarkIniSettingsDirty(); // ImGui writes designer.ini shortly after
    step(current_).update(*this);
    drawViewports();
    // Hidden sidebars are not submitted, so their dock nodes collapse and the viewports take the space.
    if (showLeftPanel_)
        drawWorkflowPanel();
    if (showRightPanel_)
        drawObjectsPanel();
    if (showRightPanel_ && ImGui::Begin(kLogWindow)) {
        for (const auto& e : log::recent(300)) {
            const ui::Palette& p = ui::palette();
            const ImVec4 c = e.level == log::Level::Error ? p.danger : e.level == log::Level::Warning ? p.warning : p.textMuted;
            ImGui::TextColored(c, "%s", e.timestamp.c_str());
            ImGui::SameLine();
            ImGui::TextWrapped("%s", e.message.c_str());
        }
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4)
            ImGui::SetScrollHereY(1.0f);
    }
    if (showRightPanel_)
        ImGui::End();

    drawStatusBar(statusH);
    drawSeriesChooser();
    drawLockPrompt();
    drawClosePrompt();
    drawHelp();
    tasks_.update();
    ui::drawModals();
    ui::drawToasts();

    if (designerOptions_.demoAutoAlign)
        runDemoAutoAlign();
    syncRestorations(*this);
    if (crownDemo_)
        runCrownDemo(*this, *crownDemo_);
    if (tasks_.busy() || !queue_.empty())
        requestRedraw();
    firstFrame_ = false;
}

void DesignerApp::setupDockLayout(ImGuiID dockspaceId)
{
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::DockBuilderRemoveNode(dockspaceId);
    ImGui::DockBuilderAddNode(dockspaceId, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspaceId, vp->WorkSize);
    ImGuiID left = 0, rest = 0, right = 0, center = 0;
    ImGui::DockBuilderSplitNode(dockspaceId, ImGuiDir_Left, 0.25f, &left, &rest);
    ImGui::DockBuilderSplitNode(rest, ImGuiDir_Right, 1.0f / 3.0f, &right, &center); // 25% of the whole width
    ImGui::DockBuilderDockWindow(kWorkflowWindow, left);
    ImGui::DockBuilderDockWindow(kObjectsWindow, right);
    ImGui::DockBuilderDockWindow(kLogWindow, right);
    ImGui::DockBuilderDockWindow(kViewportWindow, center);
    if (ImGuiDockNode* n = ImGui::DockBuilderGetNode(center))
        n->LocalFlags |= ImGuiDockNodeFlags_NoTabBar;
    if (ImGuiDockNode* n = ImGui::DockBuilderGetNode(left))
        n->LocalFlags |= ImGuiDockNodeFlags_NoTabBar;
    ImGui::DockBuilderFinish(dockspaceId);
}

void DesignerApp::handleShortcuts()
{
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput)
        return;
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false) && caseMode())
        saveDesign(false);
    if (ImGui::IsKeyPressed(ImGuiKey_F1, false))
        showHelp_ = !showHelp_;
    if (ImGui::IsKeyPressed(ImGuiKey_F, false) && !io.KeyCtrl)
        fitAllViews();
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_E, false))
        expert_ = !expert_;
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_LeftBracket, false))
        showLeftPanel_ = !showLeftPanel_;
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_RightBracket, false))
        showRightPanel_ = !showRightPanel_;
}

void DesignerApp::drawMenuBar()
{
    if (!ImGui::BeginMainMenuBar())
        return;
    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("Open DICOM folder...", nullptr, false, !headless()))
            if (auto p = ui::dialogs::pickFolder())
                loadDicom(*p, platform::pathToUtf8(*p));
        if (ImGui::MenuItem("Open DICOM file...", nullptr, false, !headless()))
            if (auto p = ui::dialogs::openFile({{"DICOM", "dcm"}, {"All files", "*"}}))
                loadDicom(*p, platform::pathToUtf8(*p));
        if (ImGui::MenuItem("Open scan (STL)...", nullptr, false, !headless()))
            for (const auto& p : ui::dialogs::openFiles({{"STL", "stl"}}))
                loadScan(p, platform::pathToUtf8(p), db::FileRole::ScanOther, platform::pathToUtf8(p.stem()));
        ImGui::Separator();
        if (ImGui::MenuItem("Save design", "Ctrl+S", false, caseMode() && !readOnly_))
            saveDesign(false);
        if (ImGui::MenuItem("Export design...", nullptr, false, !doc_.scans.empty() && !headless()))
            exportDesign();
        ImGui::Separator();
        if (ImGui::MenuItem("Exit"))
            if (onCloseRequested())
                quit();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
        if (ImGui::MenuItem("Standard layout", nullptr, layout_ == ViewLayout::Standard))
            layout_ = ViewLayout::Standard;
        if (ImGui::MenuItem("Alignment layout", nullptr, layout_ == ViewLayout::Alignment))
            layout_ = ViewLayout::Alignment;
        if (ImGui::MenuItem("3D only", nullptr, layout_ == ViewLayout::Single3D))
            layout_ = ViewLayout::Single3D;
        if (ImGui::MenuItem("2 x 2", nullptr, layout_ == ViewLayout::Slices))
            layout_ = ViewLayout::Slices;
        ImGui::Separator();
        if (ImGui::MenuItem("Fit all views", "F"))
            fitAllViews();
        const bool persp = main3D_.camera.projection == gfx::Camera::Projection::Perspective;
        if (ImGui::MenuItem("Perspective projection", nullptr, persp))
            main3D_.camera.projection = persp ? gfx::Camera::Projection::Orthographic : gfx::Camera::Projection::Perspective;
        ImGui::Separator();
        if (ImGui::MenuItem("System theme", nullptr, themeMode() == ui::ThemeMode::System))
            setThemeMode(ui::ThemeMode::System);
        if (ImGui::MenuItem("Light theme", nullptr, themeMode() == ui::ThemeMode::Light))
            setThemeMode(ui::ThemeMode::Light);
        if (ImGui::MenuItem("Dark theme", nullptr, themeMode() == ui::ThemeMode::Dark))
            setThemeMode(ui::ThemeMode::Dark);
        ImGui::Separator();
        if (ImGui::MenuItem("Steps panel", "Ctrl+[", showLeftPanel_))
            showLeftPanel_ = !showLeftPanel_;
        if (ImGui::MenuItem("Objects panel", "Ctrl+]", showRightPanel_))
            showRightPanel_ = !showRightPanel_;
        if (ImGui::MenuItem("Reset window layout"))
            resetLayout_ = true; // handled where the dock space lives (its id depends on that context)
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Workflow")) {
        if (ImGui::MenuItem("Wizard mode", nullptr, !expert_))
            expert_ = false;
        if (ImGui::MenuItem("Expert mode", "Ctrl+E", expert_))
            expert_ = true;
        ImGui::Separator();
        for (const auto& wf : workflow::allWorkflows()) {
            if (ImGui::MenuItem(wf.title.c_str(), nullptr, workflow_ == &wf)) {
                workflow_ = &wf;
                if (!inWorkflow(current_))
                    goToStep(wf.steps.front());
            }
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("Mouse & keyboard", "F1"))
            showHelp_ = true;
        ImGui::Separator();
        ImGui::MenuItem("OcclusaCAD 0.1.0 (MVP)", nullptr, false, false);
        ImGui::MenuItem(gfx::glRendererString(), nullptr, false, false);
        ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
}

void DesignerApp::drawToolbar(float height)
{
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const ui::Palette& pal = ui::palette();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, height));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(ImGui::GetStyle().WindowPadding.x, (height - ImGui::GetFrameHeight()) * 0.5f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, pal.surface);
    ImGui::Begin("##toolbar", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking);
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(3);

    if (panelToggle("##leftpanel", true, showLeftPanel_, "steps panel", "Ctrl+["))
        showLeftPanel_ = !showLeftPanel_;
    ImGui::SameLine();
    {
        ui::fonts::Scope f(ui::fonts::semibold(), ui::fonts::kHeadingSize);
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(pal.accent, "OcclusaCAD");
    }
    ImGui::SameLine(0, ImGui::GetFontSize() * 1.5f);
    ImGui::AlignTextToFramePadding();
    if (record_) {
        ImGui::TextUnformatted(record_->caseNumber.c_str());
        ImGui::SameLine();
        ui::mutedText("%s", record_->patientDisplayName().c_str());
        if (readOnly_) {
            ImGui::SameLine();
            ui::pill("Read-only", pal.warning);
        }
    } else {
        ui::mutedText("No case (files opened directly)");
    }

    // Centre: wizard / expert switch.
    const float segW = ImGui::GetFontSize() * 5.5f;
    ImGui::SameLine(ImGui::GetWindowWidth() * 0.5f - segW);
    int mode = expert_ ? 1 : 0;
    if (ui::segmented("mode", {"Wizard", "Expert"}, mode, segW))
        expert_ = mode == 1;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Wizard guides you step by step. Expert lets you open any tool directly (Ctrl+E).");

    // Right: layout, views, theme, save.
    const float unit = ImGui::GetFontSize();
    const float rightBlock = unit * 35.0f;
    ImGui::SameLine(ImGui::GetWindowWidth() - rightBlock);
    int layoutIdx = static_cast<int>(layout_);
    if (ui::segmented("layout", {"Standard", "Align", "3D", "2x2"}, layoutIdx, unit * 5.0f))
        layout_ = static_cast<ViewLayout>(layoutIdx);
    ImGui::SameLine();
    if (ImGui::Button("View"))
        ImGui::OpenPopup("##viewpresets");
    if (ImGui::BeginPopup("##viewpresets")) {
        const std::pair<const char*, gfx::ViewPreset> presets[] = {{"Front", gfx::ViewPreset::Front}, {"Back", gfx::ViewPreset::Back},
                                                                   {"Left", gfx::ViewPreset::Left},   {"Right", gfx::ViewPreset::Right},
                                                                   {"Top (occlusal lower)", gfx::ViewPreset::Top},
                                                                   {"Bottom (occlusal upper)", gfx::ViewPreset::Bottom}};
        for (const auto& [name, preset] : presets)
            if (ImGui::MenuItem(name))
                main3D_.setPreset(preset);
        ImGui::Separator();
        if (ImGui::MenuItem("Fit all", "F"))
            fitAllViews();
        ImGui::EndPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button(darkTheme() ? "Light" : "Dark"))
        setThemeMode(darkTheme() ? ui::ThemeMode::Light : ui::ThemeMode::Dark);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Switch between light and dark mode");
    ImGui::SameLine();
    if (ui::primaryButton("Save", ImVec2(unit * 4.5f, 0), caseMode() && !readOnly_ && !tasks_.busy()))
        saveDesign(false);
    ImGui::SameLine();
    if (panelToggle("##rightpanel", false, showRightPanel_, "objects panel", "Ctrl+]"))
        showRightPanel_ = !showRightPanel_;
    ImGui::End();
}

void DesignerApp::drawStepList()
{
    const ui::Palette& pal = ui::palette();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float r = ImGui::GetFontSize() * 0.62f;
    const float rowH = ImGui::GetFrameHeight() * 1.05f;

    auto row = [&](StepId id, int number, bool enabled, bool otherWorkflow) {
        const auto& info = workflow::stepInfo(id);
        const bool isCurrent = id == current_;
        const bool done = completed_.count(id) != 0;
        ImGui::PushID(static_cast<int>(id));
        const ImVec2 p = ImGui::GetCursorScreenPos();
        ImGui::BeginDisabled(!enabled);
        if (ImGui::Selectable("##step", isCurrent, ImGuiSelectableFlags_None, ImVec2(0, rowH)) && enabled) {
            goToStep(id);
            stepsExpanded_ = false;
        }
        ImGui::EndDisabled();
        const ImVec2 c(p.x + r + 2, p.y + rowH * 0.5f);
        const ImU32 col = isCurrent ? ui::toU32(pal.accent) : done ? ui::toU32(pal.success) : ui::toU32(pal.textMuted, enabled ? 1.0f : 0.5f);
        if (done && !isCurrent) {
            dl->AddCircleFilled(c, r, col, 24);
            // Check mark.
            dl->AddPolyline(std::array<ImVec2, 3>{ImVec2(c.x - r * 0.45f, c.y), ImVec2(c.x - r * 0.1f, c.y + r * 0.38f), ImVec2(c.x + r * 0.5f, c.y - r * 0.38f)}.data(),
                            3, IM_COL32_WHITE, 2.0f);
        } else if (isCurrent) {
            dl->AddCircleFilled(c, r, col, 24);
            const std::string n = number > 0 ? std::to_string(number) : "-";
            const ImVec2 ts = ImGui::CalcTextSize(n.c_str());
            dl->AddText(ImVec2(c.x - ts.x * 0.5f, c.y - ts.y * 0.5f), IM_COL32_WHITE, n.c_str());
        } else {
            dl->AddCircle(c, r, col, 24, 1.5f);
            const std::string n = number > 0 ? std::to_string(number) : "-";
            const ImVec2 ts = ImGui::CalcTextSize(n.c_str());
            dl->AddText(ImVec2(c.x - ts.x * 0.5f, c.y - ts.y * 0.5f), col, n.c_str());
        }
        const ImU32 textCol = enabled ? ui::toU32(isCurrent ? pal.text : (done ? pal.text : pal.textMuted)) : ui::toU32(pal.textMuted, 0.6f);
        ImGui::PushFont(isCurrent ? ui::fonts::semibold() : ui::fonts::regular(), 0.0f);
        dl->AddText(ImVec2(c.x + r + ImGui::GetFontSize() * 0.6f, p.y + (rowH - ImGui::GetFontSize()) * 0.5f), textCol, info.title);
        ImGui::PopFont();
        if (!info.implemented || otherWorkflow) {
            const char* tag = !info.implemented ? "planned" : "other";
            const ImVec2 ts = ImGui::CalcTextSize(tag);
            const float x = p.x + ImGui::GetContentRegionAvail().x - ts.x - 6;
            dl->AddText(ImVec2(x, p.y + (rowH - ts.y) * 0.5f), ui::toU32(pal.textMuted, 0.8f), tag);
        }
        if (ImGui::IsItemHovered() && info.summary)
            ImGui::SetItemTooltip("%s", info.summary);
        ImGui::PopID();
    };

    if (!expert_) {
        const int currentIdx = workflowIndex(current_);
        int firstIncomplete = static_cast<int>(workflow_->steps.size());
        for (std::size_t i = 0; i < workflow_->steps.size(); ++i)
            if (!completed_.count(workflow_->steps[i])) {
                firstIncomplete = static_cast<int>(i);
                break;
            }
        for (std::size_t i = 0; i < workflow_->steps.size(); ++i) {
            const StepId id = workflow_->steps[i];
            const bool enabled = static_cast<int>(i) <= std::max(currentIdx, firstIncomplete) || completed_.count(id);
            row(id, static_cast<int>(i) + 1, enabled, false);
        }
        if (currentIdx < 0) {
            ImGui::Spacing();
            ui::mutedText("Current tool is not part of this workflow.");
            row(current_, 0, true, true);
        }
    } else {
        for (const auto& [group, ids] : workflow::stepGroups()) {
            ImGui::PushStyleColor(ImGuiCol_Text, pal.textMuted);
            ImGui::TextUnformatted(group.c_str());
            ImGui::PopStyleColor();
            for (StepId id : ids) {
                const int idx = workflowIndex(id);
                row(id, idx >= 0 ? idx + 1 : 0, true, idx < 0);
            }
        }
    }
}

void DesignerApp::drawStepHeader()
{
    const ui::Palette& pal = ui::palette();
    const auto& info = workflow::stepInfo(current_);
    const int idx = workflowIndex(current_);
    const float font = ImGui::GetFontSize();
    const float h = font * 3.2f;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float w = ImGui::GetContentRegionAvail().x;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const bool clicked = ImGui::InvisibleButton("##stepheader", ImVec2(w, h));
    const bool hovered = ImGui::IsItemHovered();
    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), ui::toU32(hovered || stepsExpanded_ ? pal.surfaceAlt : pal.surface), ImGui::GetStyle().FrameRounding);
    dl->AddRect(p, ImVec2(p.x + w, p.y + h), ui::toU32(pal.border), ImGui::GetStyle().FrameRounding);
    if (clicked)
        stepsExpanded_ = !stepsExpanded_;
    if (hovered)
        ImGui::SetTooltip(stepsExpanded_ ? "Hide the steps" : "Show all steps");

    const float pad = font * 0.7f;
    // Line 1: workflow and mode.
    const std::string sub = expert_ ? workflow_->title + "  -  Expert mode"
                                    : std::format("{}  -  step {} of {}", workflow_->title, std::max(idx + 1, 1), workflow_->steps.size());
    dl->PushClipRect(p, ImVec2(p.x + w - pad * 2.5f, p.y + h), true);
    dl->AddText(ImVec2(p.x + pad, p.y + pad * 0.6f), ui::toU32(pal.textMuted), sub.c_str());
    // Line 2: badge with the step number and the step title.
    const float r = font * 0.62f;
    const ImVec2 c(p.x + pad + r, p.y + h - pad * 0.7f - font * 0.6f);
    const bool done = completed_.count(current_) != 0;
    dl->AddCircleFilled(c, r, ui::toU32(done ? pal.success : pal.accent), 24);
    const std::string n = idx >= 0 ? std::to_string(idx + 1) : "-";
    const ImVec2 ts = ImGui::CalcTextSize(n.c_str());
    dl->AddText(ImVec2(c.x - ts.x * 0.5f, c.y - ts.y * 0.5f), IM_COL32_WHITE, n.c_str());
    {
        ui::fonts::Scope f(ui::fonts::semibold(), ui::fonts::kHeadingSize);
        const float hf = ImGui::GetFontSize();
        dl->AddText(ImVec2(c.x + r + font * 0.6f, c.y - hf * 0.5f), ui::toU32(pal.text), info.title);
    }
    dl->PopClipRect();
    // Chevron.
    const float cx = p.x + w - pad * 1.4f, cy = p.y + h * 0.5f, cs = font * 0.3f;
    const ImU32 cc = ui::toU32(hovered ? pal.text : pal.textMuted);
    if (stepsExpanded_)
        dl->AddTriangleFilled(ImVec2(cx - cs, cy + cs * 0.5f), ImVec2(cx + cs, cy + cs * 0.5f), ImVec2(cx, cy - cs * 0.6f), cc);
    else
        dl->AddTriangleFilled(ImVec2(cx - cs, cy - cs * 0.5f), ImVec2(cx + cs, cy - cs * 0.5f), ImVec2(cx, cy + cs * 0.6f), cc);
    ImGui::Spacing();
}

void DesignerApp::drawWorkflowPanel()
{
    const ui::Palette& pal = ui::palette();
    if (!ImGui::Begin(kWorkflowWindow)) {
        ImGui::End();
        return;
    }
    const int idx = workflowIndex(current_);
    // Compact: only the current step; clicking it shows the whole list.
    drawStepHeader();
    if (stepsExpanded_) {
        const float listH = std::min(ImGui::GetContentRegionAvail().y * 0.5f,
                                     (expert_ ? static_cast<float>(workflow::allSteps().size() + workflow::stepGroups().size()) : static_cast<float>(workflow_->steps.size())) *
                                         ImGui::GetFrameHeightWithSpacing() * 1.05f);
        ImGui::BeginChild("##steplist", ImVec2(0, listH), ImGuiChildFlags_None);
        drawStepList();
        ImGui::EndChild();
    }
    ImGui::Separator();
    ImGui::Spacing();

    Step& st = step(current_);
    const float footerH = ImGui::GetFrameHeight() * 1.3f + ImGui::GetStyle().ItemSpacing.y * 2;
    ImGui::BeginChild("##steppanel", ImVec2(0, -footerH), ImGuiChildFlags_None);
    ui::wrappedMutedText(st.info().guidance);
    ImGui::Spacing();
    st.drawPanel(*this);
    ImGui::EndChild();

    // Footer navigation.
    ImGui::Separator();
    const float w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    const float h = ImGui::GetFrameHeight() * 1.2f;
    const bool canBack = idx > 0;
    if (ui::button("Back", ImVec2(w, h), canBack && !tasks_.busy()))
        back();
    ImGui::SameLine();
    const auto blocker = st.blocker(*this);
    const bool last = idx == static_cast<int>(workflow_->steps.size()) - 1;
    const char* nextLabel = last ? "Finish" : (idx < 0 ? "Return to workflow" : "Next");
    if (ui::primaryButton(nextLabel, ImVec2(w, h), !blocker && !tasks_.busy())) {
        if (last) {
            completed_.insert(current_);
            saveDesign(true);
        } else {
            next();
        }
    }
    if (blocker && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("%s", blocker->c_str());
    (void)pal;
    ImGui::End();
}

void DesignerApp::drawObjectsPanel()
{
    const ui::Palette& pal = ui::palette();
    if (!ImGui::Begin(kObjectsWindow)) {
        ImGui::End();
        return;
    }
    bool cbctWorkflow = doc_.volume.has_value();
    for (StepId st : workflow_->steps)
        cbctWorkflow |= st == StepId::VolumeSetup || st == StepId::ScanAlignment;
    if (cbctWorkflow)
        ui::subheading("CBCT");
    if (!cbctWorkflow) {
        // Crown & bridge without a CBCT: nothing to show here.
    } else if (doc_.volume) {
        VolumeObject& v = *doc_.volume;
        ImGui::Checkbox("##volvis", &v.display.visible);
        ImGui::SameLine();
        ImGui::TextUnformatted(v.label.c_str());
        const auto& g = v.volume->geometry;
        ui::mutedText("%.2f x %.2f x %.2f mm voxels", g.spacing.x, g.spacing.y, g.spacing.z);
        if (!v.volume->info.patientName.empty())
            ui::mutedText("Patient: %s", v.volume->info.patientName.c_str());
        ImGui::SetNextItemWidth(-FLT_MIN);
        int mode = static_cast<int>(v.display.mode);
        if (ImGui::Combo("##mode", &mode, "Surface (threshold)\0X-ray (MIP)\0Volume rendering\0"))
            v.display.mode = static_cast<gfx::VolumeMode>(mode);
    } else {
        ui::mutedText("Not loaded");
    }
    if (cbctWorkflow)
        ImGui::Spacing();
    ui::subheading("Scans");
    if (doc_.scans.empty())
        ui::mutedText("None loaded");
    int removeId = 0;
    for (auto& s : doc_.scans) {
        ImGui::PushID(s->id);
        ImGui::Checkbox("##vis", &s->visible);
        ImGui::SameLine();
        float col[3] = {s->color.r, s->color.g, s->color.b};
        if (ImGui::ColorEdit3("##col", col, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel)) {
            s->color = glm::vec3(col[0], col[1], col[2]);
        }
        ImGui::SameLine();
        if (ImGui::Selectable(s->label.c_str(), selectedScan_ == s->id, ImGuiSelectableFlags_AllowOverlap))
            selectedScan_ = s->id;
        if (ImGui::BeginPopupContextItem()) {
            if (ImGui::MenuItem("Reset alignment")) {
                s->setTransform(s->initialTransform);
                s->registration = {};
                markModified();
            }
            if (ImGui::MenuItem("Remove from scene"))
                removeId = s->id;
            ImGui::EndPopup();
        }
        ImGui::Indent(ImGui::GetFrameHeight() * 2 + ImGui::GetStyle().ItemSpacing.x * 2);
        if (s->registration.registered) {
            ui::pill(std::format("Aligned  {:.2f} mm", s->registration.surfaceRms > 0 ? s->registration.surfaceRms : s->registration.landmarkRms).c_str(),
                     pal.success);
            ImGui::SameLine();
        } else if (cbctWorkflow) {
            ui::pill("Not aligned", pal.warning);
            ImGui::SameLine();
        }
        ui::mutedText("%s", scanRoleLabel(s->role).c_str());
        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::SliderFloat("##opacity", &s->opacity, 0.1f, 1.0f, "Opacity %.2f");
        ImGui::Unindent(ImGui::GetFrameHeight() * 2 + ImGui::GetStyle().ItemSpacing.x * 2);
        ImGui::PopID();
        ImGui::Spacing();
    }
    if (removeId) {
        doc_.removeScan(removeId);
        if (selectedScan_ == removeId)
            selectedScan_ = doc_.scans.empty() ? 0 : doc_.scans.front()->id;
    }
    bool anyCrown = false;
    for (const auto& r : doc_.restorations)
        anyCrown |= r.crown != nullptr;
    if (anyCrown) {
        ImGui::Spacing();
        ui::subheading("Restorations");
        for (const auto& r : doc_.restorations) {
            if (!r.crown)
                continue;
            auto it = doc_.overlays.find(std::format("crown:{}", r.tooth));
            if (it == doc_.overlays.end())
                continue;
            ImGui::PushID(r.tooth);
            if (ImGui::Checkbox("##cvis", &it->second.visible))
                doc_.redraw();
            ImGui::SameLine();
            ImGui::Text("%s %s", r.isPontic() ? "Pontic" : r.params.coping ? "Coping" : "Crown", toothText(r.tooth).c_str());
            ImGui::SameLine();
            ui::mutedText("%.0f mm3", r.crown->volume);
            ImGui::PopID();
        }
    }
    if (doc_.volume) {
        ImGui::Spacing();
        ui::subheading("Cursor");
        ui::mutedText("%.1f, %.1f, %.1f mm (LPS)", doc_.cursor.x, doc_.cursor.y, doc_.cursor.z);
    }
    ImGui::End();
}

void DesignerApp::drawViewports()
{
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    const bool open = ImGui::Begin(kViewportWindow, nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();
    if (!open) {
        ImGui::End();
        return;
    }
    RenderServices rs{*renderer_, doc_};
    Step& st = step(current_);
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float gap = 2.0f;

    auto place3D = [&](View3D& v, ViewId id, ImVec2 pos, ImVec2 size) {
        ImGui::SetCursorScreenPos(pos);
        const ViewEvents ev = v.draw(rs, size, st.overlay(*this, id));
        st.onViewEvent(*this, id, ev);
    };
    auto placeSlice = [&](SliceView& v, ViewId id, ImVec2 pos, ImVec2 size) {
        ImGui::SetCursorScreenPos(pos);
        const ViewEvents ev = v.draw(rs, size, st.overlay(*this, id));
        st.onViewEvent(*this, id, ev);
    };

    // Without a CBCT the slice views would be empty.
    const ViewLayout layout = (!doc_.volume && (layout_ == ViewLayout::Standard || layout_ == ViewLayout::Slices)) ? ViewLayout::Single3D : layout_;
    switch (layout) {
    case ViewLayout::Single3D:
        place3D(main3D_, ViewId::Main3D, origin, avail);
        break;
    case ViewLayout::Slices: {
        const ImVec2 cell((avail.x - gap) * 0.5f, (avail.y - gap) * 0.5f);
        place3D(main3D_, ViewId::Main3D, origin, cell);
        placeSlice(axial_, ViewId::Axial, ImVec2(origin.x + cell.x + gap, origin.y), cell);
        placeSlice(coronal_, ViewId::Coronal, ImVec2(origin.x, origin.y + cell.y + gap), cell);
        placeSlice(sagittal_, ViewId::Sagittal, ImVec2(origin.x + cell.x + gap, origin.y + cell.y + gap), cell);
        break;
    }
    case ViewLayout::Alignment: {
        const float topH = (avail.y - gap) * 0.56f;
        const float halfW = (avail.x - gap) * 0.5f;
        if (scanPick_.scanFilter == 0 || !doc_.findScan(scanPick_.scanFilter))
            scanPick_.scanFilter = selectedScan_;
        place3D(scanPick_, ViewId::ScanPick, origin, ImVec2(halfW, topH));
        place3D(volumePick_, ViewId::VolumePick, ImVec2(origin.x + halfW + gap, origin.y), ImVec2(halfW, topH));
        const float bottomY = origin.y + topH + gap;
        const float bottomH = avail.y - topH - gap;
        place3D(main3D_, ViewId::Main3D, ImVec2(origin.x, bottomY), ImVec2(halfW, bottomH));
        SliceView& sv = alignmentSlice_ == SliceOrientation::Axial ? axial_ : alignmentSlice_ == SliceOrientation::Coronal ? coronal_ : sagittal_;
        const ViewId sid = alignmentSlice_ == SliceOrientation::Axial ? ViewId::Axial : alignmentSlice_ == SliceOrientation::Coronal ? ViewId::Coronal : ViewId::Sagittal;
        placeSlice(sv, sid, ImVec2(origin.x + halfW + gap, bottomY), ImVec2(halfW, bottomH));
        // Orientation switch for the verification slice.
        const float s = ImGui::GetStyle().FontScaleDpi;
        ImGui::SetCursorScreenPos(ImVec2(origin.x + avail.x - ImGui::GetFontSize() * 13.5f, bottomY + 6 * s));
        int o = static_cast<int>(alignmentSlice_);
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6 * s, 2 * s));
        if (ui::segmented("alignslice", {"Axial", "Coronal", "Sagittal"}, o, ImGui::GetFontSize() * 4.3f))
            alignmentSlice_ = static_cast<SliceOrientation>(o);
        ImGui::PopStyleVar();
        break;
    }
    case ViewLayout::Standard:
    default: {
        const float leftW = (avail.x - gap) * 0.62f;
        const float rightW = avail.x - leftW - gap;
        const float sliceH = (avail.y - 2 * gap) / 3.0f;
        place3D(main3D_, ViewId::Main3D, origin, ImVec2(leftW, avail.y));
        placeSlice(axial_, ViewId::Axial, ImVec2(origin.x + leftW + gap, origin.y), ImVec2(rightW, sliceH));
        placeSlice(coronal_, ViewId::Coronal, ImVec2(origin.x + leftW + gap, origin.y + sliceH + gap), ImVec2(rightW, sliceH));
        placeSlice(sagittal_, ViewId::Sagittal, ImVec2(origin.x + leftW + gap, origin.y + 2 * (sliceH + gap)), ImVec2(rightW, sliceH));
        break;
    }
    }
    ImGui::End();
}

void DesignerApp::drawStatusBar(float height)
{
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x, vp->WorkPos.y + vp->WorkSize.y - height));
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, height));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(ImGui::GetStyle().WindowPadding.x, ImGui::GetStyle().WindowPadding.y * 0.3f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImGui::GetStyle().Colors[ImGuiCol_MenuBarBg]);
    ImGui::Begin("##statusbar", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoMove);
    ImGui::AlignTextToFramePadding();
    const auto recent = log::recent(1);
    ui::mutedText("%s", recent.empty() ? "Ready" : recent.back().message.c_str());
    const std::string right = std::format("{}  |  {}", repo_ ? repo_->backendName() + " " + (record_ ? record_->caseNumber : "") : std::string("No database"),
                                          db::currentUserTag());
    ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::CalcTextSize(right.c_str()).x - ImGui::GetStyle().WindowPadding.x * 2);
    ui::mutedText("%s", right.c_str());
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);
}

void DesignerApp::drawSeriesChooser()
{
    if (seriesChoice_.empty())
        return;
    if (!ImGui::IsPopupOpen("Choose DICOM series"))
        ImGui::OpenPopup("Choose DICOM series");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Choose DICOM series", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("The folder contains several image series. Select the CBCT volume:");
        ImGui::Spacing();
        int chosen = -1;
        for (std::size_t i = 0; i < seriesChoice_.size(); ++i)
            if (ImGui::Selectable(seriesChoice_[i].displayName().c_str(), false))
                chosen = static_cast<int>(i);
        ImGui::Spacing();
        if (ImGui::Button("Cancel")) {
            seriesChoice_.clear();
            ImGui::CloseCurrentPopup();
        }
        if (chosen >= 0) {
            const dicom::SeriesInfo series = seriesChoice_[static_cast<std::size_t>(chosen)];
            const std::string source = seriesChoiceSource_;
            seriesChoice_.clear();
            ImGui::CloseCurrentPopup();
            // Load the chosen stack directly.
            const fs::path first = series.files.front();
            loadDicom(series.files.size() == 1 ? first : first.parent_path(), source, seriesChoiceDisplay_);
            // TODO: remember the chosen series UID so folders with several stacks re-open the same one.
        }
        ImGui::EndPopup();
    }
}

void DesignerApp::drawLockPrompt()
{
    if (showLockPrompt_) {
        ImGui::OpenPopup("Case in use");
        showLockPrompt_ = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Case in use", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("This case is currently open in OcclusaCAD on %s.", lockHolder_.c_str());
        ui::mutedText("Open it read-only, or take over if that session is no longer active.");
        ImGui::Spacing();
        if (ui::primaryButton("Open read-only")) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Take over")) {
            try {
                if (repo_->acquireLock(record_->uuid, db::currentUserTag(), true).acquired) {
                    lockHeld_ = true;
                    readOnly_ = false;
                }
            } catch (const std::exception& e) {
                ui::showError("Take over", e.what());
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void DesignerApp::drawClosePrompt()
{
    if (showClosePrompt_) {
        ImGui::OpenPopup("Unsaved design");
        showClosePrompt_ = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Unsaved design", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted(caseMode() ? "Save the design before closing?" : "Discard the current alignment?");
        ImGui::Spacing();
        const float w = ImGui::GetFontSize() * 7;
        if (caseMode()) {
            // S / Enter: save, D / N: don't save, Esc: cancel (Alt optional, as on Windows).
            if (ui::accessButton("Save", 0, ImGuiKey_S, ImVec2(w, 0), true) || ui::accessKeyPressed(ImGuiKey_Enter) ||
                ui::accessKeyPressed(ImGuiKey_KeypadEnter)) {
                closeAfterSave_ = true;
                saveDesign(false);
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
        }
        if (ui::accessButton(caseMode() ? "Don't save" : "Discard", 0, ImGuiKey_D, ImVec2(w, 0)) || ui::accessKeyPressed(ImGuiKey_N)) {
            doc_.modified = false;
            quit();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(w, 0)) || ui::accessKeyPressed(ImGuiKey_Escape))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

void DesignerApp::drawHelp()
{
    if (!showHelp_)
        return;
    ImGui::SetNextWindowSize(ImVec2(ImGui::GetFontSize() * 30, 0), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::Begin("Mouse & keyboard", &showHelp_, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize)) {
        auto line = [](const char* key, const char* what) {
            ui::keyHint(key);
            ImGui::SameLine(ImGui::GetFontSize() * 11);
            ImGui::TextUnformatted(what);
        };
        ui::subheading("3D views");
        line("Left drag", "Rotate");
        line("Right / middle drag", "Pan");
        line("Wheel", "Zoom at the mouse position");
        line("Left click", "Pick a point (alignment step)");
        ImGui::Spacing();
        ui::subheading("Slice views");
        line("Left click / drag", "Move the crosshair");
        line("Wheel", "Next / previous slice (Shift: 5 slices)");
        line("Ctrl + wheel", "Zoom");
        line("Middle / Shift + left drag", "Pan");
        line("Right drag", "Window / level");
        line("Double click", "Fit");
        ImGui::Spacing();
        ui::subheading("General");
        line("F", "Fit all views");
        line("Ctrl + E", "Toggle wizard / expert mode");
        line("Ctrl + [  /  Ctrl + ]", "Show / hide the steps and objects panels");
        line("Ctrl + S", "Save design");
    }
    ImGui::End();
}

// ---------------------------------------------------------------------------
// Demo automation (used for headless verification and screenshots)
// ---------------------------------------------------------------------------

void DesignerApp::runDemoAutoAlign()
{
    if (demoState_ < 0 || tasks_.busy() || !queue_.empty())
        return;
    if (!doc_.volume || doc_.scans.empty())
        return;
    auto* align = dynamic_cast<ScanAlignmentStep*>(steps_.at(StepId::ScanAlignment).get());
    if (!align)
        return;
    ScanObject& scan = *doc_.scans.front();
    if (demoState_ == 0) {
        goToStep(StepId::ScanAlignment);
        selectedScan_ = scan.id;
        // Simulate a technician picking four landmarks on the scan and the corresponding CBCT positions.
        glm::dmat4 truth(1.0);
        if (designerOptions_.demoGroundTruth) {
            std::ifstream in(*designerOptions_.demoGroundTruth);
            const auto j = nlohmann::json::parse(in);
            const auto& m = j["scanToCbct_columnMajor"];
            for (int c = 0; c < 4; ++c)
                for (int r = 0; r < 4; ++r)
                    truth[c][r] = m[static_cast<std::size_t>(c * 4 + r)].get<double>();
        }
        const Aabb b = scan.mesh->bounds();
        std::vector<glm::dvec3> scanPts;
        const glm::dvec3 targets[4] = {{b.min.x, b.min.y, b.max.z}, {b.max.x, b.min.y, b.max.z}, {b.center().x, b.max.y, b.max.z}, {b.center().x, b.min.y, b.center().z}};
        for (const auto& t : targets) {
            std::size_t best = 0;
            double bestD = 1e300;
            for (std::size_t i = 0; i < scan.mesh->positions.size(); ++i) {
                const double d = glm::length(glm::dvec3(scan.mesh->positions[i]) - t);
                if (d < bestD) {
                    bestD = d;
                    best = i;
                }
            }
            scanPts.push_back(glm::dvec3(scan.mesh->positions[best]));
        }
        std::vector<glm::dvec3> cbctPts;
        const glm::dvec3 noise[4] = {{0.6, -0.4, 0.3}, {-0.5, 0.5, -0.2}, {0.3, 0.6, 0.5}, {-0.4, -0.3, -0.6}};
        for (std::size_t i = 0; i < scanPts.size(); ++i)
            cbctPts.push_back(transformPoint(truth, scanPts[i]) + noise[i]);
        align->setLandmarks(*this, scanPts, cbctPts);
        align->alignFromLandmarks(*this, true);
        demoState_ = 1;
        return;
    }
    if (demoState_ == 1 && scan.registration.registered) {
        if (designerOptions_.demoGroundTruth) {
            std::ifstream in(*designerOptions_.demoGroundTruth);
            const auto j = nlohmann::json::parse(in);
            const auto& m = j["scanToCbct_columnMajor"];
            glm::dmat4 truth(1.0);
            for (int c = 0; c < 4; ++c)
                for (int r = 0; r < 4; ++r)
                    truth[c][r] = m[static_cast<std::size_t>(c * 4 + r)].get<double>();
            double maxErr = 0.0, sum = 0.0;
            std::size_t n = 0;
            for (std::size_t i = 0; i < scan.mesh->positions.size(); i += 50) {
                const glm::dvec3 p(scan.mesh->positions[i]);
                const double e = glm::length(transformPoint(scan.transform, p) - transformPoint(truth, p));
                maxErr = std::max(maxErr, e);
                sum += e;
                ++n;
            }
            log::info("DEMO registration error vs ground truth: mean {:.3f} mm, max {:.3f} mm", sum / static_cast<double>(n), maxErr);
            if (designerOptions_.demoMaxError > 0.0 && maxErr > designerOptions_.demoMaxError) {
                log::error("DEMO registration error {:.3f} mm exceeds the limit of {:.3f} mm", maxErr, designerOptions_.demoMaxError);
                setExitCode(3);
            }
        }
        layout_ = ViewLayout::Alignment;
        demoState_ = 2;
        if (designerOptions_.demoSave && caseMode()) {
            completed_.insert(StepId::LoadData);
            completed_.insert(StepId::VolumeSetup);
            completed_.insert(StepId::ScanAlignment);
            saveDesign(false);
        }
        return;
    }
    if (demoState_ == 2 && !tasks_.busy()) {
        demoState_ = -1;
    }
}

} // namespace occlusa::designer
