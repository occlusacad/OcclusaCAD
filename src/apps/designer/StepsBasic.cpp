// Load data, volume setup, review and placeholder steps.
#include "apps/designer/DesignerApp.h"
#include "core/Platform.h"
#include "ui/FileDialog.h"
#include "ui/Fonts.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"

#include <imgui.h>

#include <cmath>
#include <format>

namespace fs = std::filesystem;

namespace occlusa::designer {

using workflow::StepId;

// ---------------------------------------------------------------------------
// Load data
// ---------------------------------------------------------------------------

namespace {

class LoadDataStep final : public Step {
public:
    LoadDataStep() : Step(StepId::LoadData) {}

    std::optional<std::string> blocker(const DesignerApp& app) const override
    {
        bool needsCbct = false;
        for (StepId s : app.workflowDef().steps)
            needsCbct |= s == StepId::VolumeSetup || s == StepId::ScanAlignment;
        if (needsCbct && !app.doc().volume)
            return "Load the patient's CBCT (DICOM).";
        if (app.doc().scans.empty())
            return "Load at least one surface scan (STL).";
        return std::nullopt;
    }

    void drawPanel(DesignerApp& app) override
    {
        const ui::Palette& pal = ui::palette();
        Document& doc = app.doc();
        const bool dialogs = !app.headless() && !app.tasks().busy();

        if (app.caseMode()) {
            ui::beginCard("##casedata");
            ui::subheading("Case data");
            const db::CaseRecord& rec = *app.caseRecord();
            int shown = 0;
            for (const auto& f : rec.files) {
                if (f.role == db::FileRole::DesignOutput)
                    continue;
                ++shown;
                ImGui::PushID(f.relativePath.c_str());
                bool loaded = false;
                if (f.role == db::FileRole::Dicom)
                    loaded = doc.volume && doc.volume->source == f.relativePath;
                else
                    for (const auto& s : doc.scans)
                        loaded |= s->source == f.relativePath;
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(f.label.empty() ? f.relativePath.c_str() : f.label.c_str());
                ImGui::SameLine(ImGui::GetContentRegionAvail().x - ImGui::GetFontSize() * 4.0f);
                if (loaded) {
                    ui::pill("Loaded", pal.success);
                } else if (ui::button("Load", ImVec2(ImGui::GetFontSize() * 4.0f, 0), !app.tasks().busy())) {
                    if (f.role == db::FileRole::Dicom)
                        app.loadDicom(app.resolveSource(f.relativePath), f.relativePath);
                    else
                        app.loadScan(app.resolveSource(f.relativePath), f.relativePath, f.role, f.label.empty() ? f.relativePath : f.label);
                }
                ui::mutedText("%s", std::string(db::displayName(f.role)).c_str());
                ImGui::PopID();
            }
            if (shown == 0)
                ui::wrappedMutedText("No data is attached to this case. Add files in OcclusaCAD DB or open them below.");
            ui::endCard();
            ImGui::Spacing();
        }

        ui::beginCard("##loaded");
        ui::subheading("In the scene");
        if (doc.volume) {
            const auto& g = doc.volume->volume->geometry;
            ImGui::TextUnformatted(doc.volume->label.c_str());
            ui::mutedText("%.0f x %.0f x %.0f mm field of view", g.dims.x * g.spacing.x, g.dims.y * g.spacing.y, g.dims.z * g.spacing.z);
        } else {
            ImGui::TextColored(pal.warning, "No CBCT");
        }
        for (const auto& s : doc.scans) {
            ImGui::TextUnformatted(s->label.c_str());
            ImGui::SameLine();
            ui::mutedText("%zu triangles", s->mesh->triangleCount());
        }
        if (doc.scans.empty())
            ImGui::TextColored(pal.warning, "No surface scans");
        ui::endCard();
        ImGui::Spacing();

        ui::beginCard("##open");
        ui::subheading("Open other files");
        const float w = ImGui::GetContentRegionAvail().x;
        if (ui::button("DICOM folder...", ImVec2(w, 0), dialogs))
            if (auto p = ui::dialogs::pickFolder())
                app.loadDicom(*p, platform::pathToUtf8(*p));
        if (ui::button("DICOM file...", ImVec2(w, 0), dialogs))
            if (auto p = ui::dialogs::openFile({{"DICOM", "dcm"}, {"All files", "*"}}))
                app.loadDicom(*p, platform::pathToUtf8(*p));
        if (ui::button("Surface scans (STL)...", ImVec2(w, 0), dialogs))
            for (const auto& p : ui::dialogs::openFiles({{"STL", "stl"}}))
                app.loadScan(p, platform::pathToUtf8(p), db::FileRole::ScanOther, platform::pathToUtf8(p.stem()));
        if (app.caseMode())
            ui::wrappedMutedText("Files opened here are copied into the case folder when you save.");
        ui::endCard();
    }
};

// ---------------------------------------------------------------------------
// Volume setup
// ---------------------------------------------------------------------------

class VolumeSetupStep final : public Step {
public:
    VolumeSetupStep() : Step(StepId::VolumeSetup) {}

    void onEnter(DesignerApp&) override { tintApplied_ = nullptr; }
    void onLeave(DesignerApp& app) override
    {
        if (app.doc().volume)
            app.doc().volume->display.showThresholdOnSlices = false;
    }
    std::optional<std::string> blocker(const DesignerApp& app) const override
    {
        if (!app.doc().volume)
            return "Load a CBCT first.";
        return std::nullopt;
    }

    void drawPanel(DesignerApp& app) override
    {
        if (!app.doc().volume) {
            ui::mutedText("Load a CBCT first.");
            return;
        }
        VolumeObject& vo = *app.doc().volume;
        VolumeDisplay& d = vo.display;
        const Volume& vol = *vo.volume;
        const auto [lo, hi] = vol.valueRange();
        updateHistogram(vol);
        if (tintApplied_ != &vol) {
            // Tint above-threshold voxels while this step is open (the volume may load after entering).
            d.showThresholdOnSlices = true;
            tintApplied_ = &vol;
        }
        const ui::Palette& pal = ui::palette();

        // Threshold with histogram.
        ui::beginCard("##threshold");
        ui::subheading("Bone & teeth threshold");
        ui::wrappedMutedText("Voxels above the threshold form the 3D surface and are tinted in the slices.");
        ImGui::Spacing();
        const float plotW = ImGui::GetContentRegionAvail().x;
        const float plotH = ImGui::GetFontSize() * 4.0f;
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(pal.textMuted.x, pal.textMuted.y, pal.textMuted.z, 0.7f));
        ImGui::PlotHistogram("##hist", histogram_.data(), static_cast<int>(histogram_.size()), 0, nullptr, 0.0f, histMax_, ImVec2(plotW, plotH));
        ImGui::PopStyleColor();
        const float tx = p0.x + plotW * static_cast<float>((d.isoValue - lo) / std::max(hi - lo, 1.0));
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddLine(ImVec2(tx, p0.y), ImVec2(tx, p0.y + plotH), ui::toU32(pal.accent), 2.0f);
        const float wl = p0.x + plotW * static_cast<float>((d.windowCenter - d.windowWidth * 0.5 - lo) / std::max(hi - lo, 1.0));
        const float wh = p0.x + plotW * static_cast<float>((d.windowCenter + d.windowWidth * 0.5 - lo) / std::max(hi - lo, 1.0));
        dl->AddRectFilled(ImVec2(std::max(wl, p0.x), p0.y + plotH - 4), ImVec2(std::min(wh, p0.x + plotW), p0.y + plotH), ui::toU32(pal.success, 0.6f));

        float iso = static_cast<float>(d.isoValue);
        ImGui::SetNextItemWidth(-ImGui::GetFontSize() * 4.5f);
        if (ImGui::SliderFloat("##iso", &iso, static_cast<float>(lo), static_cast<float>(hi), "Threshold %.0f"))
            d.isoValue = iso;
        ImGui::SameLine();
        if (ImGui::Button("Auto##iso", ImVec2(-FLT_MIN, 0)))
            d.isoValue = vol.suggestBoneThreshold();
        ImGui::Checkbox("Show threshold in slices", &d.showThresholdOnSlices);
        ui::endCard();
        ImGui::Spacing();

        // Window / level.
        ui::beginCard("##window");
        ui::subheading("Slice contrast");
        const std::pair<const char*, std::pair<double, double>> presets[] = {
            {"Auto", vol.suggestWindow()}, {"Bone", {d.isoValue + 400.0, 2500.0}}, {"Teeth", {d.isoValue + 900.0, 2000.0}}, {"Soft", {80.0, 600.0}}};
        const float bw = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 3) / 4.0f;
        for (int i = 0; i < 4; ++i) {
            if (i)
                ImGui::SameLine();
            if (ImGui::Button(presets[i].first, ImVec2(bw, 0))) {
                d.windowCenter = presets[i].second.first;
                d.windowWidth = presets[i].second.second;
            }
        }
        float wc = static_cast<float>(d.windowCenter), ww = static_cast<float>(d.windowWidth);
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::SliderFloat("##wc", &wc, static_cast<float>(lo), static_cast<float>(hi), "Level %.0f"))
            d.windowCenter = wc;
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::SliderFloat("##ww", &ww, 1.0f, static_cast<float>(hi - lo), "Window %.0f"))
            d.windowWidth = ww;
        ui::mutedText("Tip: right-drag in a slice view to adjust.");
        ui::endCard();
        ImGui::Spacing();

        // 3D rendering.
        ui::beginCard("##render");
        ui::subheading("3D rendering");
        int mode = static_cast<int>(d.mode);
        if (ui::segmented("rmode", {"Surface", "X-ray", "Volume"}, mode, (ImGui::GetContentRegionAvail().x - 2) / 3.0f))
            d.mode = static_cast<gfx::VolumeMode>(mode);
        float col[3] = {d.color.r, d.color.g, d.color.b};
        if (ImGui::ColorEdit3("Colour", col, ImGuiColorEditFlags_NoInputs))
            d.color = glm::vec3(col[0], col[1], col[2]);
        if (d.mode == gfx::VolumeMode::DirectVolume) {
            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::SliderFloat("##opacity", &d.opacity, 0.1f, 4.0f, "Opacity %.2f");
        }
        ImGui::Checkbox("Show CBCT in 3D", &d.visible);
        ui::endCard();
        ImGui::Spacing();

        // Region of interest.
        ui::beginCard("##crop");
        ui::subheading("Region of interest");
        ui::wrappedMutedText("Crop away the skull and spine to focus on the jaw.");
        const char* axes[3] = {"Left-right", "Front-back", "Up-down"};
        for (int a = 0; a < 3; ++a) {
            ImGui::PushID(a);
            float mn = d.cropMin[a], mx = d.cropMax[a];
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::DragFloatRange2("##crop", &mn, &mx, 0.003f, 0.0f, 1.0f, (std::string(axes[a]) + " %.2f").c_str(), "%.2f")) {
                d.cropMin[a] = std::clamp(mn, 0.0f, mx - 0.02f);
                d.cropMax[a] = std::clamp(mx, mn + 0.02f, 1.0f);
            }
            ImGui::PopID();
        }
        if (ImGui::Button("Reset region", ImVec2(-FLT_MIN, 0))) {
            d.cropMin = glm::vec3(0.0f);
            d.cropMax = glm::vec3(1.0f);
        }
        ui::endCard();
    }

private:
    void updateHistogram(const Volume& vol)
    {
        if (histogramFor_ == &vol)
            return;
        histogramFor_ = &vol;
        const auto h = vol.histogram(160);
        histogram_.assign(h.size(), 0.0f);
        histMax_ = 0.0f;
        for (std::size_t i = 0; i < h.size(); ++i) {
            histogram_[i] = std::log1p(static_cast<float>(h[i]));
            histMax_ = std::max(histMax_, histogram_[i]);
        }
    }

    const Volume* histogramFor_ = nullptr;
    const Volume* tintApplied_ = nullptr;
    std::vector<float> histogram_;
    float histMax_ = 1.0f;
};

// ---------------------------------------------------------------------------
// Review
// ---------------------------------------------------------------------------

class ReviewStep final : public Step {
public:
    ReviewStep() : Step(StepId::Review) {}

    void drawPanel(DesignerApp& app) override
    {
        const ui::Palette& pal = ui::palette();
        Document& doc = app.doc();
        ui::beginCard("##summary");
        ui::subheading("Summary");
        if (auto* rec = app.caseRecord()) {
            ImGui::Text("Case %s", rec->caseNumber.c_str());
            ui::mutedText("%s  -  %s", rec->patientDisplayName().c_str(), rec->practice.c_str());
        }
        ImGui::Spacing();
        if (doc.volume)
            ImGui::Text("CBCT: %s", doc.volume->label.c_str());
        for (const auto& s : doc.scans) {
            ImGui::TextUnformatted(s->label.c_str());
            ImGui::SameLine();
            if (s->registration.registered)
                ui::pill(std::format("aligned {:.2f} mm", s->registration.surfaceRms > 0 ? s->registration.surfaceRms : s->registration.landmarkRms).c_str(), pal.success);
            else if (doc.volume)
                ui::pill("not aligned", pal.warning);
            else
                ImGui::NewLine();
        }
        for (const auto& r : doc.restorations) {
            if (!r.supported())
                continue;
            ImGui::Text("%s %d", r.isPontic() ? "Pontic" : r.params.coping ? "Coping" : "Crown", r.tooth);
            ImGui::SameLine();
            if (r.crown)
                ui::pill(std::format("{:.0f} mm3, min. {:.2f} mm", r.crown->volume, r.crown->minThickness).c_str(), r.crown->watertight ? pal.success : pal.danger);
            else if (r.crownGenerated)
                ui::pill("designed", pal.success);
            else
                ui::pill("not designed", pal.warning);
        }
        for (const auto& b : doc.bridges) {
            ImGui::Text("Bridge %s", b.label().c_str());
            ImGui::SameLine();
            if (b.result && b.result->ok)
                ui::pill(std::format("merged, {:.0f} mm3", b.result->volume).c_str(), b.result->watertight ? pal.success : pal.danger);
            else if (b.result)
                ui::pill("merge failed", pal.danger);
            else
                ui::pill("merged when saved", pal.accent);
        }
        ImGui::Spacing();
        int done = 0;
        for (StepId id : app.workflowDef().steps)
            done += app.isStepComplete(id) ? 1 : 0;
        ui::mutedText("%d of %zu workflow steps completed", done, app.workflowDef().steps.size());
        ui::endCard();
        ImGui::Spacing();

        ui::beginCard("##save");
        ui::subheading("Save");
        if (app.caseMode()) {
            ui::wrappedMutedText("Saves the design to the case. Crowns are written to the case's design folder as STL in the "
                                 "coordinates of their scan; aligned scans in CBCT coordinates.");
            ImGui::Spacing();
            if (ui::primaryButton("Save design", ImVec2(-FLT_MIN, 0), !app.readOnly() && !app.tasks().busy()))
                app.saveDesign(false);
        } else {
            ui::wrappedMutedText("No case is open. Export the crowns and aligned scans to use them elsewhere.");
        }
        if (ui::button("Export to folder...", ImVec2(-FLT_MIN, 0), !doc.scans.empty() && !app.headless()))
            app.exportDesign();
        ui::endCard();
        ImGui::Spacing();
        ui::beginCard("##next");
        ui::subheading("Coming next");
        ui::wrappedMutedText("Inlays and veneers, the panoramic curve, nerve tracing, implant placement and surgical "
                             "guide design are planned tools. Switch to Expert mode to preview where they fit in the workflow.");
        ui::endCard();
    }
};

// ---------------------------------------------------------------------------
// Placeholder for tools not implemented yet
// ---------------------------------------------------------------------------

class PlaceholderStep final : public Step {
public:
    explicit PlaceholderStep(StepId id) : Step(id) {}

    void drawPanel(DesignerApp& app) override
    {
        const ui::Palette& pal = ui::palette();
        ui::beginCard("##planned");
        ui::pill("Planned", pal.accent);
        ImGui::Spacing();
        ImGui::TextWrapped("%s is not available in this version of OcclusaCAD.", info().title);
        ImGui::Spacing();
        ui::wrappedMutedText(std::format("It belongs to the {} tools: {}.", info().group, info().summary).c_str());
        bool inWorkflow = false;
        for (StepId s : app.workflowDef().steps)
            inWorkflow |= s == id();
        ImGui::Spacing();
        if (!inWorkflow)
            ui::wrappedMutedText("This tool is not part of the current workflow; it was opened from Expert mode.");
        else
            ui::wrappedMutedText("You can continue with the next step; this step will be completed when the tool is available.");
        ui::endCard();
    }
};

} // namespace

std::unique_ptr<Step> makeLoadDataStep()
{
    return std::make_unique<LoadDataStep>();
}
std::unique_ptr<Step> makeVolumeSetupStep()
{
    return std::make_unique<VolumeSetupStep>();
}
std::unique_ptr<Step> makeReviewStep()
{
    return std::make_unique<ReviewStep>();
}
std::unique_ptr<Step> makePlaceholderStep(StepId id)
{
    return std::make_unique<PlaceholderStep>(id);
}

} // namespace occlusa::designer
