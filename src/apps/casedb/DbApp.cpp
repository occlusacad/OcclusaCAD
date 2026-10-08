#include "apps/casedb/DbApp.h"

#include "core/Dental.h"
#include "core/Log.h"
#include "core/Platform.h"
#include "core/Time.h"
#include "core/Workflow.h"
#include "core/dicom/DicomSeries.h"
#include "ui/FileDialog.h"
#include "ui/Fonts.h"
#include "ui/Theme.h"
#include "ui/ToothChart.h"
#include "ui/Widgets.h"

#include <imgui.h>
#include <imgui_stdlib.h>

#include <algorithm>
#include <format>
#include <map>

namespace fs = std::filesystem;

namespace occlusa::casedb {

using db::CaseStatus;
using db::FileRole;

namespace {

ImVec4 statusColor(CaseStatus s)
{
    const auto& p = ui::palette();
    switch (s) {
    case CaseStatus::New: return p.accent;
    case CaseStatus::InDesign: return p.warning;
    case CaseStatus::Designed: return p.success;
    case CaseStatus::Exported: return ImVec4(0.55f, 0.45f, 0.85f, 1.0f);
    case CaseStatus::Archived: return p.textMuted;
    }
    return p.accent;
}

const char* workflowTitle(const std::string& key)
{
    if (const auto* wf = workflow::findWorkflow(key))
        return wf->title.c_str();
    return "(none)";
}

std::vector<std::string> restorationKeys(const db::CaseRecord& r)
{
    std::vector<std::string> keys;
    for (const auto& x : r.restorations)
        keys.push_back(x.type);
    return keys;
}

} // namespace

DbApp::DbApp(ui::AppOptions options, AppConfig config, fs::path configPath, std::string selectOnStart)
    : ui::GuiApp(std::move(options)), config_(std::move(config)), configPath_(std::move(configPath)), selectOnStart_(std::move(selectOnStart))
{
}

void DbApp::onStart()
{
    if (!headless())
        ui::dialogs::init();
    if (!config_.isConfigured()) {
        showSetup_ = true;
        setupRoot_ = platform::pathToUtf8(AppConfig::suggestedDataRoot());
        return;
    }
    openRepository();
    if (repo_ && !selectOnStart_.empty()) {
        if (selectOnStart_ == "first" && !cases_.empty())
            selectCase(cases_.front().uuid);
        else if (selectOnStart_ == "new")
            newCase();
        else
            selectCase(selectOnStart_);
    }
}

void DbApp::onShutdown()
{
    if (!headless())
        ui::dialogs::shutdown();
}

bool DbApp::onCloseRequested()
{
    if (tasks_.busy())
        return false;
    if (dirty()) {
        pending_ = Pending::Close;
        openPendingPrompt_ = true;
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Data
// ---------------------------------------------------------------------------

void DbApp::openRepository()
{
    repo_.reset();
    repoError_.clear();
    try {
        repo_ = db::openRepository(config_);
        refreshList();
    } catch (const std::exception& e) {
        repoError_ = e.what();
        log::error("Cannot open case database: {}", e.what());
    }
}

void DbApp::refreshList()
{
    if (!repo_)
        return;
    try {
        db::CaseQuery q;
        q.text = search_;
        q.includeArchived = showArchived_;
        if (statusFilter_ >= 0)
            q.status = db::allCaseStatuses()[static_cast<std::size_t>(statusFilter_)];
        cases_ = repo_->listCases(q);
        repoError_.clear();
    } catch (const std::exception& e) {
        repoError_ = e.what();
    }
    lastRefresh_ = ImGui::GetCurrentContext() ? ImGui::GetTime() : 0.0;
}

bool DbApp::dirty() const
{
    if (!editing_)
        return false;
    if (isNew_)
        return true;
    return draft_.contentDiffers(original_);
}

void DbApp::requestSelect(const std::string& uuid)
{
    if (editing_ && !isNew_ && draft_.uuid == uuid)
        return;
    if (dirty()) {
        pending_ = Pending::Select;
        pendingUuid_ = uuid;
        openPendingPrompt_ = true;
        return;
    }
    selectCase(uuid);
}

void DbApp::requestNew()
{
    if (dirty()) {
        pending_ = Pending::New;
        openPendingPrompt_ = true;
        return;
    }
    newCase();
}

void DbApp::selectCase(const std::string& uuid)
{
    if (!repo_)
        return;
    try {
        auto rec = repo_->loadCase(uuid);
        if (!rec) {
            ui::toast(ui::ToastKind::Warning, "The case no longer exists");
            refreshList();
            return;
        }
        if (rec->workflow.empty() || !workflow::findWorkflow(rec->workflow))
            rec->workflow = workflow::workflowForRestorations(restorationKeys(*rec));
        original_ = *rec;
        draft_ = *rec;
        editing_ = true;
        isNew_ = false;
        workflowManual_ = draft_.workflow != workflow::workflowForRestorations(restorationKeys(draft_));
    } catch (const std::exception& e) {
        ui::showError("Open case", e.what());
    }
}

void DbApp::newCase()
{
    draft_ = db::CaseRecord{};
    original_ = draft_;
    draft_.technician = platform::userName();
    draft_.workflow = workflow::defaultWorkflow().key;
    editing_ = true;
    isNew_ = true;
    workflowManual_ = false;
}

bool DbApp::save()
{
    if (!repo_ || !editing_)
        return false;
    if (draft_.patientLastName.empty() && draft_.patientFirstName.empty()) {
        ui::showError("Save case", "Please enter the patient's name.");
        return false;
    }
    try {
        if (isNew_) {
            draft_ = repo_->createCase(draft_);
            isNew_ = false;
            ui::toast(ui::ToastKind::Success, "Case " + draft_.caseNumber + " created");
        } else {
            repo_->updateCase(draft_);
            ui::toast(ui::ToastKind::Success, "Case " + draft_.caseNumber + " saved");
        }
        original_ = draft_;
        refreshList();
        return true;
    } catch (const db::ConcurrencyError& e) {
        ui::showError("Save case", e.what());
    } catch (const std::exception& e) {
        ui::showError("Save case", e.what());
    }
    return false;
}

void DbApp::revert()
{
    if (isNew_) {
        editing_ = false;
        isNew_ = false;
        return;
    }
    selectCase(draft_.uuid);
}

void DbApp::deleteCurrent()
{
    if (!repo_ || !editing_)
        return;
    if (isNew_) {
        editing_ = false;
        isNew_ = false;
        return;
    }
    try {
        const std::string number = draft_.caseNumber;
        repo_->deleteCase(draft_.uuid);
        editing_ = false;
        refreshList();
        ui::toast(ui::ToastKind::Info, "Case " + number + " deleted");
    } catch (const std::exception& e) {
        ui::showError("Delete case", e.what());
    }
}

void DbApp::setArchived(bool archived)
{
    draft_.status = archived ? CaseStatus::Archived : CaseStatus::New;
    save();
}

bool DbApp::ensurePersisted()
{
    if (!isNew_)
        return true;
    return save();
}

void DbApp::onRestorationsChanged()
{
    std::sort(draft_.restorations.begin(), draft_.restorations.end(), [](const auto& a, const auto& b) { return a.tooth < b.tooth; });
    if (!workflowManual_)
        draft_.workflow = workflow::workflowForRestorations(restorationKeys(draft_));
}

fs::path DbApp::designerExecutable() const
{
    const fs::path dir = platform::executableDir();
    const std::string name = platform::executableName("OcclusaCAD");
    const fs::path candidates[] = {
        dir / name,
        // macOS: sibling application bundle.
        dir / ".." / ".." / ".." / "OcclusaCAD.app" / "Contents" / "MacOS" / name,
    };
    for (const auto& c : candidates)
        if (fs::exists(c))
            return c;
    return candidates[0];
}

void DbApp::launchDesigner()
{
    if (!ensurePersisted())
        return;
    if (dirty() && !save())
        return;
    if (draft_.restorations.empty()) {
        ui::showError("Design", "Select at least one tooth and restoration type before designing.");
        return;
    }
    const fs::path exe = designerExecutable();
    if (!fs::exists(exe)) {
        ui::showError("Design", "OcclusaCAD was not found next to OcclusaCAD DB:\n" + platform::pathToUtf8(exe));
        return;
    }
    if (!draft_.lockedBy.empty() && draft_.lockedBy != db::currentUserTag()) {
        ui::showError("Design", "This case is currently open in OcclusaCAD on " + draft_.lockedBy +
                                    ".\nIt will open read-only until that session closes.");
    }
    std::vector<std::string> args = {"--case", draft_.uuid};
    if (configPath_ != AppConfig::defaultPath())
        args.insert(args.end(), {"--config", platform::pathToUtf8(configPath_)});
    // The data folder may come from the command line (--data-root) rather than the saved settings.
    if (!config_.dataRoot.empty())
        args.insert(args.end(), {"--data-root", platform::pathToUtf8(config_.dataRoot)});
    std::string error;
    if (!platform::launchDetached(exe, args, &error)) {
        ui::showError("Design", "Could not start OcclusaCAD: " + error);
        return;
    }
    if (draft_.status == CaseStatus::New) {
        try {
            repo_->setStatus(draft_.uuid, CaseStatus::InDesign);
            selectCase(draft_.uuid);
            refreshList();
        } catch (const std::exception& e) {
            log::warn("Could not update case status: {}", e.what());
        }
    }
    ui::toast(ui::ToastKind::Info, "Opening " + draft_.caseNumber + " in OcclusaCAD...");
}

void DbApp::importDicomFolder()
{
    if (!ensurePersisted())
        return;
    auto folder = ui::dialogs::pickFolder();
    if (!folder)
        return;
    const db::CaseRecord record = draft_;
    db::ICaseRepository* repo = repo_.get();
    const fs::path src = *folder;
    tasks_.start("Importing CBCT", [this, record, repo, src](const ProgressFn& progress) -> ui::TaskRunner::Continuation {
        // Validate before copying: the folder must contain at least one loadable image stack.
        const auto scan = dicom::scanForSeries(src, [&](float f, const std::string& m) { return progress(f * 0.3f, m); });
        if (scan.series.empty())
            throw std::runtime_error("No DICOM images were found in " + platform::pathToUtf8(src));
        const auto& s = scan.series.front();
        const std::string label = s.seriesDescription.empty() ? std::format("CBCT {}x{}x{}", s.columns, s.rows, s.sliceCount)
                                                              : std::format("{} ({}x{}x{})", s.seriesDescription, s.columns, s.rows, s.sliceCount);
        const std::string rel = repo->files().importDirectory(record, src, FileRole::Dicom,
                                                              [&](float f, const std::string& m) { return progress(0.3f + f * 0.7f, m); });
        // Continuations run on the UI thread; the task runner is owned by (and never outlives) the app.
        return [this, rel, label] {
            draft_.files.push_back(db::CaseFile{0, FileRole::Dicom, rel, label, time::nowUtcIso8601()});
            save();
        };
    });
}

void DbApp::importDicomFile()
{
    // Single multi-frame DICOM files are handled like folders by the series scanner.
    if (!ensurePersisted())
        return;
    auto file = ui::dialogs::openFile({{"DICOM", "dcm"}, {"All files", "*"}});
    if (!file)
        return;
    const db::CaseRecord record = draft_;
    db::ICaseRepository* repo = repo_.get();
    const fs::path src = *file;
    tasks_.start("Importing CBCT", [this, record, repo, src](const ProgressFn& progress) -> ui::TaskRunner::Continuation {
        const auto scan = dicom::scanForSeries(src, progress);
        if (scan.series.empty())
            throw std::runtime_error("The file is not a supported DICOM image");
        const auto& s = scan.series.front();
        const std::string label = std::format("{} ({}x{}x{})", s.seriesDescription.empty() ? "CBCT" : s.seriesDescription, s.columns,
                                              s.rows, s.sliceCount);
        const std::string rel = repo->files().importFile(record, src, FileRole::Dicom, progress);
        return [this, rel, label] {
            draft_.files.push_back(db::CaseFile{0, FileRole::Dicom, rel, label, time::nowUtcIso8601()});
            save();
        };
    });
}

void DbApp::importScans(FileRole role)
{
    if (!ensurePersisted())
        return;
    auto files = ui::dialogs::openFiles({{"Surface scans", "stl"}});
    if (files.empty())
        return;
    const db::CaseRecord record = draft_;
    db::ICaseRepository* repo = repo_.get();
    tasks_.start("Importing scans", [this, record, repo, files, role](const ProgressFn& progress) -> ui::TaskRunner::Continuation {
        std::vector<db::CaseFile> added;
        for (std::size_t i = 0; i < files.size(); ++i) {
            const float base = static_cast<float>(i) / static_cast<float>(files.size());
            const std::string rel = repo->files().importFile(record, files[i], role, [&](float f, const std::string& m) {
                return progress(base + f / static_cast<float>(files.size()), m);
            });
            added.push_back(db::CaseFile{0, role, rel, platform::pathToUtf8(files[i].stem()), time::nowUtcIso8601()});
        }
        return [this, added] {
            draft_.files.insert(draft_.files.end(), added.begin(), added.end());
            save();
        };
    });
}

void DbApp::removeFile(std::size_t index)
{
    if (index >= draft_.files.size())
        return;
    const db::CaseFile file = draft_.files[index];
    draft_.files.erase(draft_.files.begin() + static_cast<std::ptrdiff_t>(index));
    if (save()) {
        try {
            repo_->files().remove(draft_, file.relativePath);
        } catch (const std::exception& e) {
            log::warn("Could not remove {}: {}", file.relativePath, e.what());
        }
    }
}

// ---------------------------------------------------------------------------
// Frame
// ---------------------------------------------------------------------------

void DbApp::applyConfigTheme()
{
    setThemeMode(ui::themeModeFromString(config_.theme));
}

void DbApp::onFrame()
{
    // Periodic refresh so changes from other workstations appear.
    if (repo_ && !tasks_.busy() && ImGui::GetTime() - lastRefresh_ > 10.0)
        refreshList();

    // Keyboard shortcuts.
    const ImGuiIO& io = ImGui::GetIO();
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_N, false) && repo_)
        requestNew();
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false) && dirty())
        save();

    drawMenuBar();

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float statusH = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y;
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, vp->WorkSize.y - statusH));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::Begin("##main", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoDocking);
    ImGui::PopStyleVar(2);

    drawHeader();
    ImGui::Spacing();

    if (!repo_) {
        ui::beginCard("##norepo");
        ui::heading("Case database unavailable");
        ui::wrappedMutedText(repoError_.empty() ? "No data folder is configured." : repoError_.c_str());
        ImGui::Spacing();
        if (ui::primaryButton("Choose data folder..."))
            showSettings_ = true;
        ui::endCard();
    } else {
        const float listW = ImGui::GetContentRegionAvail().x * 0.48f;
        ImGui::BeginChild("##list", ImVec2(listW, 0), ImGuiChildFlags_ResizeX);
        drawCaseList();
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("##editor", ImVec2(0, 0));
        drawEditor();
        ImGui::EndChild();
    }
    ImGui::End();

    drawStatusBar();
    drawSetupDialog();
    drawSettingsDialog();
    drawPendingPrompt();
    drawDeletePrompt();
    tasks_.update();
    ui::drawModals();
    ui::drawToasts();
}

void DbApp::drawMenuBar()
{
    if (!ImGui::BeginMainMenuBar())
        return;
    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("New case", "Ctrl+N", false, repo_ != nullptr))
            requestNew();
        if (ImGui::MenuItem("Save case", "Ctrl+S", false, dirty()))
            save();
        ImGui::Separator();
        if (ImGui::MenuItem("Open data folder", nullptr, false, config_.isConfigured()))
            platform::openInFileBrowser(config_.dataRoot);
        if (ImGui::MenuItem("Settings..."))
            showSettings_ = true;
        ImGui::Separator();
        if (ImGui::MenuItem("Exit"))
            if (onCloseRequested())
                quit();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
        const ui::ThemeMode mode = themeMode();
        if (ImGui::MenuItem("System theme", nullptr, mode == ui::ThemeMode::System)) {
            config_.theme = "system";
            applyConfigTheme();
        }
        if (ImGui::MenuItem("Light theme", nullptr, mode == ui::ThemeMode::Light)) {
            config_.theme = "light";
            applyConfigTheme();
        }
        if (ImGui::MenuItem("Dark theme", nullptr, mode == ui::ThemeMode::Dark)) {
            config_.theme = "dark";
            applyConfigTheme();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Refresh case list", "F5"))
            refreshList();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help")) {
        ImGui::MenuItem("OcclusaCAD DB 0.1.0 (MVP)", nullptr, false, false);
        ImGui::EndMenu();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F5, false))
        refreshList();
    ImGui::EndMainMenuBar();
}

void DbApp::drawHeader()
{
    const float right = ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x;
    {
        ui::fonts::Scope f(ui::fonts::semibold(), ui::fonts::kTitleSize);
        ImGui::TextUnformatted("OcclusaCAD DB");
    }
    ImGui::SameLine();
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + ImGui::GetFontSize() * 0.45f);
    ui::mutedText("Case management");

    const float btnW = ImGui::GetFontSize() * 9.0f;
    ImGui::SameLine(right - btnW);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() - ImGui::GetFontSize() * 0.3f);
    if (ui::primaryButton("+  New case", ImVec2(btnW, ImGui::GetFrameHeight() * 1.15f), repo_ != nullptr))
        requestNew();
}

void DbApp::drawCaseList()
{
    const ui::Palette& p = ui::palette();
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
    if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_F, false))
        ImGui::SetKeyboardFocusHere();
    if (ImGui::InputTextWithHint("##search", "Search case, patient, practice...  (Ctrl+F)", &search_))
        refreshList();
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 9.0f);
    std::string previewStr = statusFilter_ < 0 ? "All statuses" : std::string(db::displayName(db::allCaseStatuses()[static_cast<std::size_t>(statusFilter_)]));
    if (ImGui::BeginCombo("##status", previewStr.c_str())) {
        if (ImGui::Selectable("All statuses", statusFilter_ < 0)) {
            statusFilter_ = -1;
            refreshList();
        }
        for (std::size_t i = 0; i < db::allCaseStatuses().size(); ++i) {
            const std::string name(db::displayName(db::allCaseStatuses()[i]));
            if (ImGui::Selectable(name.c_str(), statusFilter_ == static_cast<int>(i))) {
                statusFilter_ = static_cast<int>(i);
                refreshList();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("Archived", &showArchived_))
        refreshList();

    ImGui::Spacing();
    const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable |
                                  ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_PadOuterX;
    if (ImGui::BeginTable("##cases", 6, flags, ImVec2(0, 0))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Case", ImGuiTableColumnFlags_WidthStretch, 1.1f);
        ImGui::TableSetupColumn("Patient", ImGuiTableColumnFlags_WidthStretch, 1.3f);
        ImGui::TableSetupColumn("Practice", ImGuiTableColumnFlags_WidthStretch, 1.1f);
        ImGui::TableSetupColumn("Teeth", ImGuiTableColumnFlags_WidthStretch, 0.8f);
        ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthStretch, 0.9f);
        ImGui::TableSetupColumn("Modified", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableHeadersRow();

        for (const auto& c : cases_) {
            ImGui::TableNextRow(ImGuiTableRowFlags_None, ImGui::GetFrameHeight() * 1.25f);
            ImGui::TableSetColumnIndex(0);
            const bool selected = editing_ && !isNew_ && draft_.uuid == c.uuid;
            ImGui::PushID(c.uuid.c_str());
            ImGui::AlignTextToFramePadding();
            if (ImGui::Selectable(c.caseNumber.c_str(), selected, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap,
                                  ImVec2(0, ImGui::GetFrameHeight() * 1.1f)))
                requestSelect(c.uuid);
            if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                requestSelect(c.uuid);
                if (!dirty() && editing_ && draft_.uuid == c.uuid)
                    launchDesigner();
            }
            ImGui::TableSetColumnIndex(1);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(c.patient.c_str());
            ImGui::TableSetColumnIndex(2);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(c.practice.c_str());
            ImGui::TableSetColumnIndex(3);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(dental::formatToothList(c.teeth, dental::numberingFromString(config_.toothNumbering)).c_str());
            ImGui::TableSetColumnIndex(4);
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + ImGui::GetStyle().FramePadding.y * 0.5f);
            ui::pill(std::string(db::displayName(c.status)).c_str(), statusColor(c.status));
            if (!c.lockedBy.empty() && ImGui::BeginItemTooltip()) {
                ImGui::Text("Open in OcclusaCAD on %s", c.lockedBy.c_str());
                ImGui::EndTooltip();
            }
            ImGui::TableSetColumnIndex(5);
            ImGui::AlignTextToFramePadding();
            ImGui::PushStyleColor(ImGuiCol_Text, p.textMuted);
            ImGui::TextUnformatted(time::utcIsoToLocalDisplay(c.modifiedUtc).c_str());
            ImGui::PopStyleColor();
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (cases_.empty()) {
        ImGui::Spacing();
        ui::mutedText(search_.empty() ? "No cases yet. Create one with \"New case\"." : "No cases match the search.");
    }
}

void DbApp::drawEditor()
{
    if (!editing_) {
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        ImGui::SetCursorPos(ImVec2(ImGui::GetCursorPosX() + avail.x * 0.15f, ImGui::GetCursorPosY() + avail.y * 0.35f));
        ImGui::BeginGroup();
        ui::heading("No case selected");
        ui::mutedText("Select a case from the list or create a new one.");
        ImGui::Spacing();
        if (ui::primaryButton("+  New case"))
            requestNew();
        ImGui::EndGroup();
        return;
    }

    // Title row.
    {
        ui::fonts::Scope f(ui::fonts::semibold(), ui::fonts::kHeadingSize);
        if (isNew_)
            ImGui::TextUnformatted("New case");
        else
            ImGui::Text("%s", draft_.caseNumber.c_str());
    }
    ImGui::SameLine();
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 2);
    ui::pill(std::string(db::displayName(draft_.status)).c_str(), statusColor(draft_.status));
    if (dirty()) {
        ImGui::SameLine();
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 2);
        ui::pill("Unsaved changes", ui::palette().warning);
    }
    if (!draft_.lockedBy.empty()) {
        ImGui::SameLine();
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 2);
        ui::pill(("Open on " + draft_.lockedBy).c_str(), ui::palette().textMuted);
    }

    drawActions();
    ImGui::Spacing();

    ImGui::BeginChild("##editor_scroll", ImVec2(0, 0), ImGuiChildFlags_None);
    const float colW = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    ImGui::BeginGroup();
    ImGui::PushItemWidth(colW);
    drawPatientCard();
    ImGui::Spacing();
    drawCaseCard();
    ImGui::PopItemWidth();
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    drawToothCard();
    ImGui::EndGroup();
    ImGui::Spacing();
    drawFilesCard();
    ImGui::EndChild();
}

void DbApp::drawActions()
{
    const float h = ImGui::GetFrameHeight() * 1.15f;
    const float unit = ImGui::GetFontSize();
    if (ui::primaryButton("Design in OcclusaCAD", ImVec2(unit * 14, h), !tasks_.busy()))
        launchDesigner();
    ImGui::SameLine();
    if (ui::button("Save", ImVec2(unit * 6, h), dirty()))
        save();
    ImGui::SameLine();
    if (ui::button(isNew_ ? "Discard" : "Revert", ImVec2(unit * 6, h), dirty()))
        revert();
    ImGui::SameLine();
    if (ImGui::Button("More...", ImVec2(unit * 6, h)))
        ImGui::OpenPopup("##case_more");
    if (ImGui::BeginPopup("##case_more")) {
        if (ImGui::MenuItem("Open case folder", nullptr, false, !isNew_ && repo_)) {
            try {
                platform::openInFileBrowser(repo_->files().caseDirectory(draft_));
            } catch (const std::exception& e) {
                ui::showError("Open folder", e.what());
            }
        }
        if (draft_.status == CaseStatus::Archived) {
            if (ImGui::MenuItem("Restore from archive", nullptr, false, !isNew_))
                setArchived(false);
        } else if (ImGui::MenuItem("Archive case", nullptr, false, !isNew_)) {
            setArchived(true);
        }
        if (ImGui::MenuItem("Release design lock", nullptr, false, !draft_.lockedBy.empty())) {
            try {
                repo_->acquireLock(draft_.uuid, db::currentUserTag(), true);
                repo_->releaseLock(draft_.uuid, db::currentUserTag());
                selectCase(draft_.uuid);
                refreshList();
            } catch (const std::exception& e) {
                ui::showError("Release lock", e.what());
            }
        }
        ImGui::Separator();
        if (ImGui::MenuItem(isNew_ ? "Discard new case" : "Delete case..."))
            openDeletePrompt_ = true;
        ImGui::EndPopup();
    }
}

void DbApp::drawPatientCard()
{
    ui::beginCard("##patient", ImVec2(ImGui::CalcItemWidth(), 0));
    ui::subheading("Patient");
    if (ui::beginForm("##patientform")) {
        ui::formField("Last name", draft_.patientLastName);
        ui::formField("First name", draft_.patientFirstName);
        ui::formField("Date of birth", draft_.patientBirthDate, "YYYY-MM-DD");
        ui::formField("Patient ID / chart", draft_.patientReference);
        ui::endForm();
    }
    ui::endCard();
}

void DbApp::drawCaseCard()
{
    ui::beginCard("##casecard", ImVec2(ImGui::CalcItemWidth(), 0));
    ui::subheading("Case");
    if (ui::beginForm("##caseform")) {
        const std::string hint = isNew_ && repo_ ? "auto: " + repo_->nextCaseNumber() : std::string();
        ui::formField("Case number", draft_.caseNumber, hint.empty() ? nullptr : hint.c_str());
        ui::formField("Due date", draft_.dueDate, "YYYY-MM-DD");
        ui::formField("Practice", draft_.practice);
        ui::formField("Dentist", draft_.dentist);
        ui::formField("Technician", draft_.technician);
        ui::formLabel("Status");
        if (ImGui::BeginCombo("##status_edit", std::string(db::displayName(draft_.status)).c_str())) {
            for (CaseStatus s : db::allCaseStatuses())
                if (ImGui::Selectable(std::string(db::displayName(s)).c_str(), s == draft_.status))
                    draft_.status = s;
            ImGui::EndCombo();
        }
        ui::endForm();
    }

    ImGui::PushStyleColor(ImGuiCol_Text, ui::palette().textMuted);
    ImGui::TextUnformatted("Workflow");
    ImGui::PopStyleColor();
    ImGui::SetNextItemWidth(-FLT_MIN);
    const std::string autoKey = workflow::workflowForRestorations(restorationKeys(draft_));
    const std::string preview = std::string(workflowTitle(draft_.workflow)) + (workflowManual_ ? "" : "  (automatic)");
    if (ImGui::BeginCombo("##workflow", preview.c_str())) {
        if (ImGui::Selectable((std::string("Automatic: ") + workflowTitle(autoKey)).c_str(), !workflowManual_)) {
            workflowManual_ = false;
            draft_.workflow = autoKey;
        }
        ImGui::Separator();
        for (const auto& wf : workflow::allWorkflows()) {
            if (ImGui::Selectable(wf.title.c_str(), workflowManual_ && draft_.workflow == wf.key)) {
                workflowManual_ = true;
                draft_.workflow = wf.key;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", wf.description.c_str());
        }
        ImGui::EndCombo();
    }

    ImGui::PushStyleColor(ImGuiCol_Text, ui::palette().textMuted);
    ImGui::TextUnformatted("Notes");
    ImGui::PopStyleColor();
    ImGui::InputTextMultiline("##notes", &draft_.notes, ImVec2(-FLT_MIN, ImGui::GetTextLineHeight() * 4));
    ui::endCard();
}

void DbApp::drawToothCard()
{
    ui::beginCard("##teeth", ImVec2(0, 0));
    ui::subheading("Teeth & restorations");
    ImGui::SameLine();
    ui::helpMarker("Choose a restoration type, then click teeth on the chart. Click a tooth again to remove it.");
    ImGui::Spacing();

    // Restoration type palette grouped by category (wrapping buttons).
    const float maxX = ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
    for (const std::string& cat : dental::restorationCategories()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ui::palette().textMuted);
        ImGui::TextUnformatted(cat.c_str());
        ImGui::PopStyleColor();
        bool first = true;
        for (const auto& t : dental::restorationTypes()) {
            if (cat != t.category)
                continue;
            const std::string label = std::string(t.label);
            const float w = ImGui::CalcTextSize(label.c_str()).x + ImGui::GetStyle().FramePadding.x * 2 + ImGui::GetFontSize();
            if (!first) {
                const float next = ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + w;
                if (next < maxX)
                    ImGui::SameLine();
            }
            first = false;
            const bool sel = selectedType_ == t.key;
            const ImVec4 col(t.color[0], t.color[1], t.color[2], 1.0f);
            ImGui::PushStyleColor(ImGuiCol_Button, sel ? col : ImVec4(col.x, col.y, col.z, 0.18f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(col.x, col.y, col.z, sel ? 1.0f : 0.35f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, col);
            ImGui::PushStyleColor(ImGuiCol_Text, sel ? ui::palette().onAccent : ui::palette().text);
            if (ImGui::Button(label.c_str(), ImVec2(w, 0)))
                selectedType_ = t.key;
            ImGui::PopStyleColor(4);
        }
    }
    ImGui::Spacing();

    std::map<int, std::string> assigned;
    for (const auto& r : draft_.restorations)
        assigned[r.tooth] = r.type;
    ui::ToothChartStyle style;
    style.numbering = dental::numberingFromString(config_.toothNumbering);
    style.width = std::min(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 30.0f);
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (ImGui::GetContentRegionAvail().x - style.width) * 0.5f);
    if (const int tooth = ui::toothChart("##chart", assigned, style)) {
        auto it = std::find_if(draft_.restorations.begin(), draft_.restorations.end(), [&](const auto& r) { return r.tooth == tooth; });
        if (it != draft_.restorations.end() && it->type == selectedType_)
            draft_.restorations.erase(it);
        else if (it != draft_.restorations.end())
            it->type = selectedType_;
        else
            draft_.restorations.push_back(db::Restoration{0, tooth, selectedType_, "", "", "", ""});
        onRestorationsChanged();
    }

    if (!draft_.restorations.empty()) {
        ImGui::Spacing();
        if (ImGui::BeginTable("##rest", 5, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Tooth", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 3.0f);
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthStretch, 1.4f);
            ImGui::TableSetupColumn("Material / implant", ImGuiTableColumnFlags_WidthStretch, 1.4f);
            ImGui::TableSetupColumn("Shade", ImGuiTableColumnFlags_WidthStretch, 0.6f);
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
            ImGui::TableHeadersRow();
            int removeIndex = -1;
            for (std::size_t i = 0; i < draft_.restorations.size(); ++i) {
                auto& r = draft_.restorations[i];
                ImGui::PushID(static_cast<int>(i));
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(dental::toothLabel(r.tooth, style.numbering).c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::SetNextItemWidth(-1);
                const auto* t = dental::findRestorationType(r.type);
                if (ImGui::BeginCombo("##type", t ? t->label : r.type.c_str())) {
                    for (const auto& rt : dental::restorationTypes())
                        if (ImGui::Selectable(rt.label, r.type == rt.key)) {
                            r.type = rt.key;
                            onRestorationsChanged();
                        }
                    ImGui::EndCombo();
                }
                ImGui::TableSetColumnIndex(2);
                ImGui::SetNextItemWidth(-1);
                const bool implant = t && std::string_view(t->category) != "Crown & bridge";
                if (implant)
                    ImGui::InputTextWithHint("##sys", "Implant system", &r.implantSystem);
                else
                    ImGui::InputTextWithHint("##mat", "Material", &r.material);
                ImGui::TableSetColumnIndex(3);
                ImGui::SetNextItemWidth(-1);
                ImGui::InputTextWithHint("##shade", "Shade", &r.shade);
                ImGui::TableSetColumnIndex(4);
                if (ImGui::Button("x", ImVec2(ImGui::GetFrameHeight(), 0)))
                    removeIndex = static_cast<int>(i);
                ImGui::PopID();
            }
            ImGui::EndTable();
            if (removeIndex >= 0) {
                draft_.restorations.erase(draft_.restorations.begin() + removeIndex);
                onRestorationsChanged();
            }
        }
    }
    ui::endCard();
}

void DbApp::drawFilesCard()
{
    ui::beginCard("##files", ImVec2(0, 0));
    ui::subheading("Data");
    ImGui::SameLine();
    ui::helpMarker("Files are copied into the case folder on the shared data drive, so every workstation can open them.");
    ImGui::Spacing();
    const bool canImport = !tasks_.busy() && repo_ && !headless();
    if (ui::button("Add CBCT folder...", ImVec2(0, 0), canImport))
        importDicomFolder();
    ImGui::SameLine();
    if (ui::button("Add CBCT file...", ImVec2(0, 0), canImport))
        importDicomFile();
    ImGui::SameLine();
    if (ui::button("Add scans...", ImVec2(0, 0), canImport))
        ImGui::OpenPopup("##scanrole");
    if (ImGui::BeginPopup("##scanrole")) {
        for (FileRole r : {FileRole::ScanUpper, FileRole::ScanLower, FileRole::ScanBite, FileRole::ScanOther})
            if (ImGui::MenuItem(std::string(db::displayName(r)).c_str()))
                importScans(r);
        ImGui::EndPopup();
    }
    ImGui::Spacing();
    if (draft_.files.empty()) {
        ui::mutedText("No data attached yet. Implant planning needs a CBCT and at least one surface scan.");
    } else if (ImGui::BeginTable("##filetable", 4, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthStretch, 0.8f);
        ImGui::TableSetupColumn("Description", ImGuiTableColumnFlags_WidthStretch, 1.2f);
        ImGui::TableSetupColumn("Location", ImGuiTableColumnFlags_WidthStretch, 1.5f);
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 5.0f);
        ImGui::TableHeadersRow();
        int removeIndex = -1;
        for (std::size_t i = 0; i < draft_.files.size(); ++i) {
            auto& f = draft_.files[i];
            ImGui::PushID(static_cast<int>(i));
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(std::string(db::displayName(f.role)).c_str());
            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(-1);
            ImGui::InputText("##label", &f.label);
            ImGui::TableSetColumnIndex(2);
            ImGui::AlignTextToFramePadding();
            ui::mutedText("%s", f.relativePath.c_str());
            ImGui::TableSetColumnIndex(3);
            if (ui::button("Remove", ImVec2(-1, 0), !tasks_.busy()))
                removeIndex = static_cast<int>(i);
            ImGui::PopID();
        }
        ImGui::EndTable();
        if (removeIndex >= 0)
            removeFile(static_cast<std::size_t>(removeIndex));
    }
    ui::endCard();
}

void DbApp::drawStatusBar()
{
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float h = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y;
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x, vp->WorkPos.y + vp->WorkSize.y - h));
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, h));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(ImGui::GetStyle().WindowPadding.x, ImGui::GetStyle().WindowPadding.y * 0.4f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImGui::GetStyle().Colors[ImGuiCol_MenuBarBg]);
    ImGui::Begin("##statusbar", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoMove);
    ImGui::AlignTextToFramePadding();
    if (repo_)
        ui::mutedText("%s  |  %s  |  %zu cases  |  %s", repo_->backendName().c_str(), repo_->location().c_str(), cases_.size(),
                      db::currentUserTag().c_str());
    else
        ui::mutedText("Not connected");
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);
}

// ---------------------------------------------------------------------------
// Dialogs
// ---------------------------------------------------------------------------

void DbApp::drawSetupDialog()
{
    if (!showSetup_)
        return;
    if (!ImGui::IsPopupOpen("Welcome to OcclusaCAD"))
        ImGui::OpenPopup("Welcome to OcclusaCAD");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(ImGui::GetFontSize() * 36, 0));
    if (ImGui::BeginPopupModal("Welcome to OcclusaCAD", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
        ui::heading("Set up the lab data folder");
        ui::wrappedMutedText("OcclusaCAD keeps the case database and all case files in one data folder. To share cases between "
                             "workstations, choose a folder on a network share that every workstation can reach (for example "
                             "\\\\server\\dental\\OcclusaCAD or /Volumes/Lab/OcclusaCAD).");
        ImGui::Spacing();
        ui::labeledInput("Data folder", setupRoot_, nullptr, -ImGui::GetFontSize() * 6.5f);
        ImGui::SameLine();
        if (ui::button("Browse...", ImVec2(-1, 0), !headless()))
            if (auto p = ui::dialogs::pickFolder())
                setupRoot_ = platform::pathToUtf8(*p);
        ImGui::Spacing();
        ImGui::BeginDisabled();
        bool cloud = false;
        ImGui::Checkbox("Use OcclusaCAD Cloud instead (coming soon)", &cloud);
        ImGui::EndDisabled();
        ImGui::Spacing();
        if (ui::primaryButton("Continue", ImVec2(ImGui::GetFontSize() * 8, 0), !setupRoot_.empty())) {
            config_.databaseBackend = "sqlite";
            config_.dataRoot = platform::pathFromUtf8(setupRoot_);
            try {
                config_.save(configPath_);
            } catch (const std::exception& e) {
                ui::showError("Settings", e.what());
            }
            openRepository();
            showSetup_ = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void DbApp::drawSettingsDialog()
{
    if (showSettings_) {
        ImGui::OpenPopup("Settings");
        showSettings_ = false;
        setupRoot_ = platform::pathToUtf8(config_.dataRoot);
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(ImGui::GetFontSize() * 34, 0));
    if (ImGui::BeginPopupModal("Settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ui::subheading("Appearance");
        int theme = config_.theme == "light" ? 1 : config_.theme == "dark" ? 2 : 0;
        if (ui::segmented("theme", {"System", "Light", "Dark"}, theme, ImGui::GetFontSize() * 6)) {
            config_.theme = theme == 1 ? "light" : theme == 2 ? "dark" : "system";
            applyConfigTheme();
        }
        ImGui::Spacing();
        ui::subheading("Tooth numbering");
        int numbering = config_.toothNumbering == "universal" ? 1 : 0;
        if (ui::segmented("numbering", {"FDI (ISO 3950)", "Universal"}, numbering, ImGui::GetFontSize() * 9))
            config_.toothNumbering = numbering == 1 ? "universal" : "fdi";
        ImGui::Spacing();
        ui::subheading("Case database");
        int backend = config_.databaseBackend == "cloud" ? 1 : 0;
        if (ui::segmented("backend", {"Data folder (SQLite)", "Cloud (preview)"}, backend, ImGui::GetFontSize() * 11))
            config_.databaseBackend = backend == 1 ? "cloud" : "sqlite";
        if (config_.databaseBackend == "sqlite") {
            ui::labeledInput("Data folder (local or network share)", setupRoot_, nullptr, -ImGui::GetFontSize() * 6.5f);
            ImGui::SameLine();
            if (ui::button("Browse...", ImVec2(-1, 0), !headless()))
                if (auto p = ui::dialogs::pickFolder(config_.dataRoot))
                    setupRoot_ = platform::pathToUtf8(*p);
        } else {
            ui::labeledInput("Endpoint", config_.cloudEndpoint, "https://cloud.example.com", -1);
            ui::labeledInput("Tenant", config_.cloudTenant, nullptr, -1);
            ui::wrappedMutedText("The cloud backend is not available in this version. Cases cannot be loaded or saved while it is selected.");
        }
        ImGui::Spacing();
        ImGui::Separator();
        if (ui::primaryButton("Save", ImVec2(ImGui::GetFontSize() * 7, 0))) {
            const bool rootChanged = platform::pathFromUtf8(setupRoot_) != config_.dataRoot;
            config_.dataRoot = platform::pathFromUtf8(setupRoot_);
            try {
                config_.save(configPath_);
            } catch (const std::exception& e) {
                ui::showError("Settings", e.what());
            }
            if (rootChanged || !repo_ || repo_->backendName() != (config_.databaseBackend == "cloud" ? "Cloud (preview)" : "SQLite")) {
                editing_ = false;
                openRepository();
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(ImGui::GetFontSize() * 7, 0))) {
            config_ = AppConfig::load(configPath_);
            applyConfigTheme();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void DbApp::drawPendingPrompt()
{
    if (openPendingPrompt_) {
        ImGui::OpenPopup("Unsaved changes");
        openPendingPrompt_ = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Unsaved changes", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Save changes to %s?", isNew_ ? "the new case" : draft_.caseNumber.c_str());
        ImGui::Spacing();
        auto proceed = [&] {
            const Pending p = pending_;
            pending_ = Pending::None;
            if (p == Pending::Select)
                selectCase(pendingUuid_);
            else if (p == Pending::New)
                newCase();
            else if (p == Pending::Close)
                quit();
        };
        const float w = ImGui::GetFontSize() * 7;
        // S / Enter: save, D / N: don't save, Esc: cancel (Alt optional, as on Windows).
        if (ui::accessButton("Save", 0, ImGuiKey_S, ImVec2(w, 0), true) || ui::accessKeyPressed(ImGuiKey_Enter) ||
            ui::accessKeyPressed(ImGuiKey_KeypadEnter)) {
            ImGui::CloseCurrentPopup();
            if (save())
                proceed();
            else
                pending_ = Pending::None;
        }
        ImGui::SameLine();
        if (ui::accessButton("Don't save", 0, ImGuiKey_D, ImVec2(w, 0)) || ui::accessKeyPressed(ImGuiKey_N)) {
            ImGui::CloseCurrentPopup();
            editing_ = false;
            isNew_ = false;
            proceed();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(w, 0)) || ui::accessKeyPressed(ImGuiKey_Escape)) {
            pending_ = Pending::None;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void DbApp::drawDeletePrompt()
{
    if (openDeletePrompt_) {
        if (isNew_) {
            deleteCurrent();
        } else {
            ImGui::OpenPopup("Delete case");
        }
        openDeletePrompt_ = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Delete case", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Permanently delete case %s and all of its files?", draft_.caseNumber.c_str());
        ui::mutedText("This cannot be undone. Consider archiving the case instead.");
        ImGui::Spacing();
        if (ui::dangerButton("Delete", ImVec2(ImGui::GetFontSize() * 7, 0))) {
            deleteCurrent();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(ImGui::GetFontSize() * 7, 0)))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

} // namespace occlusa::casedb
