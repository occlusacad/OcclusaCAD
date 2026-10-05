#pragma once

#include "core/AppConfig.h"
#include "db/CaseRepository.h"
#include "ui/App.h"
#include "ui/Task.h"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace occlusa::casedb {

// OcclusaCAD DB: case management front end (patient/case details, restoration
// selection on a tooth chart, data attachment) that launches OcclusaCAD for design.
class DbApp final : public ui::GuiApp {
public:
    DbApp(ui::AppOptions options, AppConfig config, std::filesystem::path configPath, std::string selectOnStart);

protected:
    void onStart() override;
    void onFrame() override;
    bool onCloseRequested() override;
    void onShutdown() override;

private:
    enum class Pending { None, Select, New, Close };

    // Data
    void openRepository();
    void refreshList();
    bool dirty() const;
    void requestSelect(const std::string& uuid);
    void requestNew();
    void selectCase(const std::string& uuid);
    void newCase();
    bool save();
    void revert();
    void deleteCurrent();
    void setArchived(bool archived);
    void launchDesigner();
    void importDicomFolder();
    void importDicomFile();
    void importScans(db::FileRole role);
    void removeFile(std::size_t index);
    bool ensurePersisted();
    void onRestorationsChanged();

    // UI
    void drawMenuBar();
    void drawHeader();
    void drawCaseList();
    void drawEditor();
    void drawCaseCard();
    void drawPatientCard();
    void drawToothCard();
    void drawFilesCard();
    void drawActions();
    void drawSetupDialog();
    void drawSettingsDialog();
    void drawPendingPrompt();
    void drawDeletePrompt();
    void drawStatusBar();
    void applyConfigTheme();
    std::filesystem::path designerExecutable() const;

    AppConfig config_;
    std::filesystem::path configPath_;
    std::unique_ptr<db::ICaseRepository> repo_;
    std::string repoError_;

    std::vector<db::CaseSummary> cases_;
    std::string search_;
    int statusFilter_ = -1; // -1 = all active
    bool showArchived_ = false;
    double lastRefresh_ = -1000.0;

    bool editing_ = false;
    bool isNew_ = false;
    db::CaseRecord original_;
    db::CaseRecord draft_;
    bool workflowManual_ = false;
    std::string selectedType_ = "implant_planning";

    Pending pending_ = Pending::None;
    std::string pendingUuid_;
    bool openPendingPrompt_ = false;
    bool openDeletePrompt_ = false;
    bool showSettings_ = false;
    bool showSetup_ = false;
    std::string setupRoot_;
    std::string selectOnStart_;
    ui::TaskRunner tasks_;
};

} // namespace occlusa::casedb
