#include "apps/designer/StepScanAlignment.h"

#include "apps/designer/DesignerApp.h"
#include "core/Geometry.h"
#include "core/IsoSurface.h"
#include "core/Log.h"
#include "ui/Fonts.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"

#include <ImGuizmo.h>
#include <imgui.h>

#include <format>

namespace occlusa::designer {

namespace {

constexpr ImU32 kScanPointColor = IM_COL32(255, 176, 59, 255);
constexpr ImU32 kCbctPointColor = IM_COL32(64, 200, 255, 255);

void drawMarker(ImDrawList* dl, ImVec2 p, int number, ImU32 color, bool emphasised = false)
{
    const float s = ImGui::GetStyle().FontScaleDpi;
    const float r = (emphasised ? 9.0f : 8.0f) * s;
    dl->AddCircleFilled(p, r + 1.5f * s, IM_COL32(0, 0, 0, 140), 20);
    dl->AddCircleFilled(p, r, color, 20);
    dl->AddCircle(p, r, IM_COL32_WHITE, 20, 1.5f * s);
    const std::string n = std::to_string(number);
    ui::fonts::Scope f(ui::fonts::semibold(), ui::fonts::kBaseSize * 0.85f);
    const ImVec2 ts = ImGui::CalcTextSize(n.c_str());
    dl->AddText(ImVec2(p.x - ts.x * 0.5f, p.y - ts.y * 0.5f), IM_COL32(20, 20, 24, 255), n.c_str());
}

void drawHint(ImDrawList* dl, const Projector& proj, const char* text, ImU32 accent)
{
    const float s = ImGui::GetStyle().FontScaleDpi;
    const ImVec2 ts = ImGui::CalcTextSize(text);
    const ImVec2 pos((proj.min.x + proj.max.x - ts.x) * 0.5f, proj.max.y - ts.y - 16 * s);
    dl->AddRectFilled(ImVec2(pos.x - 10 * s, pos.y - 5 * s), ImVec2(pos.x + ts.x + 10 * s, pos.y + ts.y + 5 * s), IM_COL32(15, 18, 22, 200), 6 * s);
    dl->AddRect(ImVec2(pos.x - 10 * s, pos.y - 5 * s), ImVec2(pos.x + ts.x + 10 * s, pos.y + ts.y + 5 * s), accent, 6 * s, 1.5f * s);
    dl->AddText(pos, IM_COL32(240, 242, 245, 255), text);
}

glm::dmat4 rotationAboutPoint(const glm::dvec3& c, const glm::dvec3& axis, double deg)
{
    return glm::translate(glm::dmat4(1.0), c) * glm::rotate(glm::dmat4(1.0), glm::radians(deg), axis) * glm::translate(glm::dmat4(1.0), -c);
}

} // namespace

ScanAlignmentStep::ScanAlignmentStep() : Step(workflow::StepId::ScanAlignment)
{
    icp_.maxCorrespondenceDistance = 1.5;
    icp_.trimFraction = 0.8;
    icp_.finalCorrespondenceDistance = 0.4;
    icp_.maxIterations = 80;
}

void ScanAlignmentStep::onEnter(DesignerApp& app)
{
    if (app.doc().volume) {
        app.view3D(ViewId::VolumePick).requestFit();
        if (icpIso_ == 0.0)
            icpIso_ = app.doc().volume->display.isoValue;
    }
    app.view3D(ViewId::ScanPick).pickCursor = true;
    app.view3D(ViewId::VolumePick).pickCursor = true;
}

void ScanAlignmentStep::onLeave(DesignerApp& app)
{
    app.view3D(ViewId::Main3D).gizmoScan = 0;
    app.view3D(ViewId::ScanPick).pickCursor = false;
    app.view3D(ViewId::VolumePick).pickCursor = false;
    manual_ = false;
}

ScanObject* ScanAlignmentStep::activeScan(DesignerApp& app)
{
    ScanObject* s = app.doc().findScan(app.selectedScan());
    if (!s && !app.doc().scans.empty()) {
        s = app.doc().scans.front().get();
        app.selectScan(s->id);
    }
    if (s && s->id != scanId_) {
        // Landmarks belong to one scan.
        scanId_ = s->id;
        scanPts_.clear();
        cbctPts_.clear();
        residuals_.clear();
        deviation_.reset();
        lastIcp_.reset();
        picking_ = 0;
        app.view3D(ViewId::ScanPick).scanFilter = s->id;
        app.view3D(ViewId::ScanPick).requestFit();
    }
    return s;
}

std::optional<std::string> ScanAlignmentStep::blocker(const DesignerApp& app) const
{
    if (app.doc().scans.empty())
        return "Load at least one surface scan first.";
    if (!app.doc().volume)
        return "Load the CBCT first.";
    for (const auto& s : app.doc().scans)
        if (s->visible && !s->registration.registered)
            return "Align every visible scan to the CBCT (or hide scans you do not need).";
    return std::nullopt;
}

void ScanAlignmentStep::setLandmarks(DesignerApp& app, std::vector<glm::dvec3> scanLocal, std::vector<glm::dvec3> cbctWorld)
{
    activeScan(app); // binds the landmarks to the selected scan before they are set
    scanPts_ = std::move(scanLocal);
    cbctPts_ = std::move(cbctWorld);
    residuals_.clear();
}

void ScanAlignmentStep::pushUndo(const ScanObject& scan)
{
    undo_.emplace_back(scan.id, scan.transform);
    if (undo_.size() > 50)
        undo_.erase(undo_.begin());
}

void ScanAlignmentStep::applyTransform(DesignerApp& app, ScanObject& scan, const glm::dmat4& t, const std::string& method)
{
    pushUndo(scan);
    scan.setTransform(t);
    scan.registration.method = method;
    app.markModified();
}

std::optional<glm::dvec3> ScanAlignmentStep::pickVolume(const DesignerApp& app, const Ray& ray) const
{
    // March the ray through the (cropped) volume and return the first threshold crossing,
    // matching what the CBCT view renders.
    const auto& vo = *app.doc().volume;
    const Volume& vol = *vo.volume;
    const glm::dmat4 w2t = vol.geometry.worldToTexture();
    const glm::dvec3 o = transformPoint(w2t, ray.origin);
    const glm::dvec3 d = transformVector(w2t, ray.direction);
    Aabb box;
    box.min = glm::dvec3(vo.display.cropMin);
    box.max = glm::dvec3(vo.display.cropMax);
    const double dl = glm::length(d);
    double t0 = 0, t1 = 0;
    if (!intersectRayAabb(Ray{o, d / dl}, box, t0, t1))
        return std::nullopt;
    // Convert texture-space distances back to world distances.
    t0 = std::max(t0, 0.0) / dl;
    t1 = t1 / dl;
    const double step = std::min({vol.geometry.spacing.x, vol.geometry.spacing.y, vol.geometry.spacing.z}) * 0.5;
    const double iso = vo.display.isoValue;
    double prev = t0;
    for (double t = t0; t <= t1; t += step) {
        if (vol.sampleWorld(ray.at(t)) >= iso) {
            double a = prev, b = t;
            for (int i = 0; i < 12; ++i) {
                const double m = 0.5 * (a + b);
                (vol.sampleWorld(ray.at(m)) >= iso ? b : a) = m;
            }
            return ray.at(b);
        }
        prev = t;
    }
    return std::nullopt;
}

void ScanAlignmentStep::alignFromLandmarks(DesignerApp& app, bool refineAfter)
{
    ScanObject* scan = activeScan(app);
    if (!scan)
        return;
    const std::size_t n = std::min(scanPts_.size(), cbctPts_.size());
    if (n < 3) {
        ui::showError("Point-pair alignment", "Pick at least three matching points on the scan and on the CBCT.");
        return;
    }
    std::vector<glm::dvec3> src(scanPts_.begin(), scanPts_.begin() + static_cast<std::ptrdiff_t>(n));
    std::vector<glm::dvec3> dst(cbctPts_.begin(), cbctPts_.begin() + static_cast<std::ptrdiff_t>(n));
    const auto res = rigidFromPointPairs(src, dst);
    if (!res) {
        ui::showError("Point-pair alignment", "The picked points are (almost) on a line. Pick points spread across the arch.");
        return;
    }
    residuals_ = res->residuals;
    applyTransform(app, *scan, res->transform, "point_pairs");
    scan->registration.registered = true;
    scan->registration.landmarkRms = res->rms;
    scan->registration.surfaceRms = 0.0;
    log::info("Point-pair alignment of '{}': {} pairs, rms {:.2f} mm", scan->label, n, res->rms);
    // Put the slice crosshair on the scan so its outline shows in the MPR views.
    app.centerCursorOn(scan->worldBounds().center());
    if (res->rms > 2.0)
        ui::toast(ui::ToastKind::Warning, std::format("Landmark residual is high ({:.1f} mm). Check that the point order matches.", res->rms));
    app.fitAllViews();
    if (refineAfter)
        refine(app);
    else
        updateDeviation(app);
}

void ScanAlignmentStep::refine(DesignerApp& app)
{
    ScanObject* scan = activeScan(app);
    if (!scan || !app.doc().volume)
        return;
    VolumeObject& vo = *app.doc().volume;
    const double iso = icpIso_ > 0.0 ? icpIso_ : vo.display.isoValue;
    // Region of interest: the scan's current footprint plus a margin for the remaining misalignment.
    Aabb region = scan->worldBounds();
    const double margin = icp_.maxCorrespondenceDistance + 6.0;
    region.min -= glm::dvec3(margin);
    region.max += glm::dvec3(margin);

    std::shared_ptr<const IcpTarget> cached;
    if (vo.icpTarget && std::abs(vo.icpTargetIso - iso) < 1e-6 && vo.icpTargetRegion.valid() &&
        glm::all(glm::lessThanEqual(vo.icpTargetRegion.min, region.min)) && glm::all(glm::greaterThanEqual(vo.icpTargetRegion.max, region.max)))
        cached = vo.icpTarget;

    const std::shared_ptr<const Volume> volume = vo.volume;
    const std::shared_ptr<const Mesh> mesh = scan->mesh;
    const glm::dmat4 start = scan->transform;
    const int id = scan->id;
    const IcpOptions opts = icp_;

    app.tasks().start("Refining alignment", [=, &app, this](const ProgressFn& progress) -> ui::TaskRunner::Continuation {
        std::shared_ptr<const IcpTarget> target = cached;
        Aabb targetRegion = region;
        if (!target) {
            // Voxel ROI from the world region (enlarged to cover future refinements nearby).
            Aabb big = region;
            big.min -= glm::dvec3(10.0);
            big.max += glm::dvec3(10.0);
            const Aabb vbox = big.transformed(volume->geometry.worldToVoxel());
            IsoSurfaceOptions iopt;
            iopt.isoValue = iso;
            iopt.roiMin = glm::max(glm::ivec3(glm::floor(vbox.min)), glm::ivec3(0));
            iopt.roiMax = glm::min(glm::ivec3(glm::ceil(vbox.max)) + 1, volume->geometry.dims);
            const glm::ivec3 ext = iopt.roiMax - iopt.roiMin;
            const double voxels = static_cast<double>(ext.x) * ext.y * ext.z;
            iopt.step = voxels > 60e6 ? 2 : 1;
            Mesh surface = extractIsoSurface(*volume, iopt, [&](float f, const std::string& m) { return progress(f * 0.4f, m); });
            if (surface.positions.size() < 100)
                throw std::runtime_error("No CBCT surface found at the current threshold near the scan. Lower the threshold or improve the initial alignment.");
            target = std::make_shared<IcpTarget>(IcpTarget::fromMesh(surface));
            targetRegion = big;
        }
        const IcpResult result = refineIcp(*mesh, start, *target, opts, [&](float f, const std::string& m) { return progress(0.4f + 0.55f * f, m); });
        const DeviationStats dev = measureDeviation(*mesh, result.transform, *target, 0.5, opts.maxCorrespondenceDistance * 2.0);
        return [=, &app, this] {
            if (app.doc().volume) {
                app.doc().volume->icpTarget = target;
                app.doc().volume->icpTargetIso = iso;
                app.doc().volume->icpTargetRegion = targetRegion;
            }
            ScanObject* s = app.doc().findScan(id);
            if (!s)
                return;
            if (result.iterations == 0 || result.inlierFraction < 0.2) {
                ui::showError("Refine alignment", "The scan is too far from the CBCT surface to refine automatically. "
                                                  "Align it with point pairs first, or increase the search distance.");
                return;
            }
            const std::string method = s->registration.method.rfind("point_pairs", 0) == 0 ? "point_pairs+icp" : "icp";
            applyTransform(app, *s, result.transform, method);
            s->registration.registered = true;
            s->registration.surfaceRms = result.rms;
            s->registration.fractionWithinTolerance = dev.fractionWithin;
            lastIcp_ = result;
            deviation_ = dev;
            // Landmark residuals after refinement.
            residuals_.clear();
            for (std::size_t i = 0; i < std::min(scanPts_.size(), cbctPts_.size()); ++i)
                residuals_.push_back(glm::length(transformPoint(s->transform, scanPts_[i]) - cbctPts_[i]));
            ui::toast(ui::ToastKind::Success, std::format("Alignment refined: {:.2f} mm RMS, {:.0f}% of the scan within 0.5 mm", result.rms,
                                                          dev.fractionWithin * 100.0));
        };
    });
}

void ScanAlignmentStep::updateDeviation(DesignerApp& app)
{
    ScanObject* scan = activeScan(app);
    if (!scan || !app.doc().volume || !app.doc().volume->icpTarget) {
        deviation_.reset();
        return;
    }
    deviation_ = measureDeviation(*scan->mesh, scan->transform, *app.doc().volume->icpTarget, 0.5, icp_.maxCorrespondenceDistance * 2.0);
}

// ---------------------------------------------------------------------------
// Viewports
// ---------------------------------------------------------------------------

OverlayFn ScanAlignmentStep::overlay(DesignerApp& app, ViewId view)
{
    return [this, &app, view](ImDrawList* dl, const Projector& proj) {
        ScanObject* scan = app.doc().findScan(scanId_);
        const std::size_t nextIndex = std::min(scanPts_.size(), cbctPts_.size()) + 1;
        switch (view) {
        case ViewId::ScanPick:
            for (std::size_t i = 0; i < scanPts_.size(); ++i)
                if (auto p = proj(scanPts_[i]))
                    drawMarker(dl, *p, static_cast<int>(i + 1), kScanPointColor);
            if (scan && !manual_) {
                const std::string hint = scanPts_.size() <= cbctPts_.size()
                                             ? std::format("Click point {} on the scan", scanPts_.size() + 1)
                                             : std::format("Now click point {} on the CBCT", scanPts_.size());
                drawHint(dl, proj, hint.c_str(), scanPts_.size() <= cbctPts_.size() ? kScanPointColor : IM_COL32(120, 120, 120, 255));
            }
            break;
        case ViewId::VolumePick:
            for (std::size_t i = 0; i < cbctPts_.size(); ++i)
                if (auto p = proj(cbctPts_[i]))
                    drawMarker(dl, *p, static_cast<int>(i + 1), kCbctPointColor);
            if (scan && !manual_ && scanPts_.size() > cbctPts_.size()) {
                const std::string hint = std::format("Click the matching point {} on the CBCT", cbctPts_.size() + 1);
                drawHint(dl, proj, hint.c_str(), kCbctPointColor);
            }
            break;
        case ViewId::Main3D:
            if (scan && scan->registration.registered) {
                for (std::size_t i = 0; i < std::min(scanPts_.size(), cbctPts_.size()); ++i) {
                    if (auto p = proj(cbctPts_[i]))
                        drawMarker(dl, *p, static_cast<int>(i + 1), kCbctPointColor);
                }
            }
            (void)nextIndex;
            break;
        default: {
            // Slice views: CBCT landmarks lying close to the slice plane.
            SliceView& sv = app.sliceView(view);
            const Plane pl = sv.plane(app.doc());
            for (std::size_t i = 0; i < cbctPts_.size(); ++i)
                if (std::abs(pl.signedDistance(cbctPts_[i])) < 1.5)
                    if (auto p = proj(cbctPts_[i]))
                        drawMarker(dl, *p, static_cast<int>(i + 1), kCbctPointColor);
            break;
        }
        }
    };
}

void ScanAlignmentStep::onViewEvent(DesignerApp& app, ViewId view, const ViewEvents& ev)
{
    ScanObject* scan = activeScan(app);
    if (!scan)
        return;

    if (view == ViewId::Main3D) {
        View3D& v = app.view3D(ViewId::Main3D);
        v.gizmoScan = manual_ ? scan->id : 0;
        v.gizmoOperation = gizmoOp_;
        const bool using_ = ImGuizmo::IsUsing();
        if (v.gizmoDelta) {
            if (!gizmoWasUsing_)
                pushUndo(*scan);
            scan->setTransform(orthonormalize(*v.gizmoDelta * scan->transform));
            scan->registration.method = "manual";
            app.markModified();
        }
        if (gizmoWasUsing_ && !using_)
            updateDeviation(app);
        gizmoWasUsing_ = using_;
        // Clicking a surface centres the slices there (handy to verify the fit).
        if (ev.click && !manual_) {
            std::optional<glm::dvec3> best;
            double bestD = 1e300;
            for (const auto& s : app.doc().scans)
                if (s->visible)
                    if (auto hit = raycastMesh(*s->mesh, s->transform, *ev.click); hit && hit->distance < bestD) {
                        bestD = hit->distance;
                        best = hit->point;
                    }
            if (app.doc().volume && app.doc().volume->display.visible)
                if (auto p = pickVolume(app, *ev.click); p && glm::length(*p - ev.click->origin) < bestD)
                    best = p;
            if (best)
                app.centerCursorOn(*best);
        }
        return;
    }
    if (manual_)
        return;
    if (view == ViewId::ScanPick && ev.click) {
        if (scanPts_.size() > cbctPts_.size()) {
            ui::toast(ui::ToastKind::Info, "Pick the matching point on the CBCT first (right view).");
            return;
        }
        if (auto hit = raycastMesh(*scan->mesh, glm::dmat4(1.0), *ev.click))
            scanPts_.push_back(hit->point);
        return;
    }
    if (view == ViewId::VolumePick && ev.click && app.doc().volume) {
        if (cbctPts_.size() >= scanPts_.size()) {
            ui::toast(ui::ToastKind::Info, "Pick the point on the scan first (left view).");
            return;
        }
        if (auto p = pickVolume(app, *ev.click)) {
            cbctPts_.push_back(*p);
            app.centerCursorOn(*p);
        }
    }
}

// ---------------------------------------------------------------------------
// Panel
// ---------------------------------------------------------------------------

void ScanAlignmentStep::drawPanel(DesignerApp& app)
{
    const ui::Palette& pal = ui::palette();
    if (!app.doc().volume || app.doc().scans.empty()) {
        ui::beginCard("##need");
        ui::mutedText("%s", !app.doc().volume ? "Load a CBCT first." : "Load a surface scan first.");
        ui::endCard();
        return;
    }
    ScanObject* scan = activeScan(app);
    if (!scan)
        return;

    // Scan selector.
    ImGui::PushStyleColor(ImGuiCol_Text, pal.textMuted);
    ImGui::TextUnformatted("Scan to align");
    ImGui::PopStyleColor();
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginCombo("##scan", scan->label.c_str())) {
        for (const auto& s : app.doc().scans) {
            const std::string label = s->label + (s->registration.registered ? "   (aligned)" : "");
            if (ImGui::Selectable(label.c_str(), s->id == scan->id))
                app.selectScan(s->id);
        }
        ImGui::EndCombo();
    }
    scan = activeScan(app);
    ImGui::Spacing();

    int method = manual_ ? 1 : 0;
    if (ui::segmented("method", {"Point pairs", "Manual"}, method, (ImGui::GetContentRegionAvail().x) * 0.5f)) {
        manual_ = method == 1;
        if (manual_ && app.layout() == ViewLayout::Alignment)
            app.setLayout(ViewLayout::Standard);
        if (!manual_)
            app.setLayout(ViewLayout::Alignment);
    }
    ImGui::Spacing();
    if (manual_)
        drawManualSection(app, *scan);
    else
        drawLandmarkSection(app, *scan);
    ImGui::Spacing();
    drawRefineSection(app, *scan);
    ImGui::Spacing();
    drawVerification(app, *scan);
}

void ScanAlignmentStep::drawLandmarkSection(DesignerApp& app, ScanObject& scan)
{
    const ui::Palette& pal = ui::palette();
    ui::beginCard("##landmarks");
    ui::subheading("Point pairs");
    ui::wrappedMutedText("Click a feature on the scan (left), then the same feature on the CBCT (right). "
                         "Cusp tips and incisal edges spread around the arch work best.");
    ImGui::Spacing();
    const std::size_t pairs = std::min(scanPts_.size(), cbctPts_.size());
    if (ImGui::BeginTable("##pairs", 4, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 1.5f);
        ImGui::TableSetupColumn("Scan");
        ImGui::TableSetupColumn("CBCT");
        ImGui::TableSetupColumn("Error");
        ImGui::TableHeadersRow();
        const std::size_t rows = std::max(scanPts_.size(), cbctPts_.size());
        for (std::size_t i = 0; i < rows; ++i) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%zu", i + 1);
            ImGui::TableSetColumnIndex(1);
            if (i < scanPts_.size())
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kScanPointColor), "picked");
            else
                ui::mutedText("-");
            ImGui::TableSetColumnIndex(2);
            if (i < cbctPts_.size())
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kCbctPointColor), "picked");
            else
                ui::mutedText("-");
            ImGui::TableSetColumnIndex(3);
            if (i < residuals_.size())
                ImGui::TextColored(residuals_[i] > 1.5 ? pal.warning : pal.text, "%.2f mm", residuals_[i]);
            else
                ui::mutedText("-");
        }
        ImGui::EndTable();
    }
    if (pairs == 0 && scanPts_.empty())
        ui::mutedText("No points yet.");
    ImGui::Spacing();
    const float w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    if (ui::button("Undo last point", ImVec2(w, 0), !scanPts_.empty())) {
        if (scanPts_.size() > cbctPts_.size())
            scanPts_.pop_back();
        else {
            cbctPts_.pop_back();
        }
        residuals_.clear();
    }
    ImGui::SameLine();
    if (ui::button("Clear points", ImVec2(w, 0), !scanPts_.empty() || !cbctPts_.empty())) {
        scanPts_.clear();
        cbctPts_.clear();
        residuals_.clear();
    }
    ImGui::Checkbox("Refine automatically after aligning", &autoRefine_);
    ImGui::Spacing();
    const std::string label = pairs >= 3 ? std::format("Align with {} point pairs", pairs) : std::format("Align ({} of 3 pairs)", pairs);
    if (ui::primaryButton(label.c_str(), ImVec2(-FLT_MIN, ImGui::GetFrameHeight() * 1.2f), pairs >= 3 && !app.tasks().busy()))
        alignFromLandmarks(app, autoRefine_);
    ui::endCard();
    (void)scan;
}

void ScanAlignmentStep::drawRefineSection(DesignerApp& app, ScanObject& scan)
{
    ui::beginCard("##refine");
    ui::subheading("Surface refinement (ICP)");
    ui::wrappedMutedText("Fits the scan to the CBCT surface of the teeth. Gingiva and scan artefacts are ignored automatically.");
    ImGui::Spacing();
    const auto [lo, hi] = app.doc().volume->volume->valueRange();
    float iso = static_cast<float>(icpIso_ > 0 ? icpIso_ : app.doc().volume->display.isoValue);
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::SliderFloat("##icpiso", &iso, static_cast<float>(lo), static_cast<float>(hi), "Surface threshold %.0f"))
        icpIso_ = iso;
    float dist = static_cast<float>(icp_.maxCorrespondenceDistance);
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::SliderFloat("##icpdist", &dist, 0.3f, 5.0f, "Search distance %.1f mm"))
        icp_.maxCorrespondenceDistance = dist;
    float trim = static_cast<float>(icp_.trimFraction * 100.0);
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::SliderFloat("##icptrim", &trim, 40.0f, 100.0f, "Use best %.0f%% of matches"))
        icp_.trimFraction = trim / 100.0;
    ImGui::Spacing();
    if (ui::button("Refine alignment", ImVec2(-FLT_MIN, ImGui::GetFrameHeight() * 1.1f), !app.tasks().busy()))
        refine(app);
    if (lastIcp_)
        ui::mutedText("Last run: %d iterations, %.0f%% matched%s", lastIcp_->iterations, lastIcp_->inlierFraction * 100.0,
                      lastIcp_->converged ? ", converged" : "");
    ui::endCard();
    (void)scan;
}

void ScanAlignmentStep::drawManualSection(DesignerApp& app, ScanObject& scan)
{
    ui::beginCard("##manual");
    ui::subheading("Manual adjustment");
    ui::wrappedMutedText("Drag the gizmo in the 3D view, or nudge the scan in small steps. Check the outline in the slice views.");
    ImGui::Spacing();
    ui::segmented("gizmo", {"Move", "Rotate"}, gizmoOp_, ImGui::GetContentRegionAvail().x * 0.5f);
    ImGui::Spacing();

    const glm::dvec3 c = scan.worldBounds().center();
    const float bw = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 5) / 6.0f;
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
    ImGui::SliderFloat("##nudgemm", &nudgeMm_, 0.05f, 2.0f, "Step %.2f mm");
    struct Btn {
        const char* label;
        glm::dvec3 dir;
        const char* tip;
    };
    const Btn moves[6] = {{"R", {-1, 0, 0}, "Towards the patient's right"}, {"L", {1, 0, 0}, "Towards the patient's left"},
                          {"A", {0, -1, 0}, "Anterior"},                     {"P", {0, 1, 0}, "Posterior"},
                          {"I", {0, 0, -1}, "Inferior"},                     {"S", {0, 0, 1}, "Superior"}};
    for (int i = 0; i < 6; ++i) {
        if (i)
            ImGui::SameLine();
        ImGui::PushID(i);
        if (ImGui::Button(moves[i].label, ImVec2(bw, 0)))
            applyTransform(app, scan, glm::translate(glm::dmat4(1.0), moves[i].dir * static_cast<double>(nudgeMm_)) * scan.transform, "manual");
        ImGui::SetItemTooltip("%s", moves[i].tip);
        ImGui::PopID();
    }
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
    ImGui::SliderFloat("##nudgedeg", &nudgeDeg_, 0.1f, 5.0f, "Step %.1f deg");
    const Btn rots[6] = {{"-X", {1, 0, 0}, "Tilt (about left-right axis)"}, {"+X", {1, 0, 0}, "Tilt (about left-right axis)"},
                         {"-Y", {0, 1, 0}, "Roll (about front-back axis)"},  {"+Y", {0, 1, 0}, "Roll (about front-back axis)"},
                         {"-Z", {0, 0, 1}, "Turn (about vertical axis)"},    {"+Z", {0, 0, 1}, "Turn (about vertical axis)"}};
    for (int i = 0; i < 6; ++i) {
        if (i)
            ImGui::SameLine();
        ImGui::PushID(100 + i);
        if (ImGui::Button(rots[i].label, ImVec2(bw, 0)))
            applyTransform(app, scan, rotationAboutPoint(c, rots[i].dir, (i % 2 == 0 ? -1.0 : 1.0) * nudgeDeg_) * scan.transform, "manual");
        ImGui::SetItemTooltip("%s", rots[i].tip);
        ImGui::PopID();
    }
    ImGui::Spacing();
    if (ui::primaryButton("Accept alignment", ImVec2(-FLT_MIN, 0), !scan.registration.registered || scan.registration.method == "manual")) {
        scan.registration.registered = true;
        if (scan.registration.method.empty())
            scan.registration.method = "manual";
        updateDeviation(app);
        app.markModified();
    }
    ui::endCard();
}

void ScanAlignmentStep::drawVerification(DesignerApp& app, ScanObject& scan)
{
    const ui::Palette& pal = ui::palette();
    ui::beginCard("##verify");
    ui::subheading("Result");
    if (scan.registration.registered) {
        ui::pill("Aligned", pal.success);
        ImGui::SameLine();
        ui::mutedText("%s", scan.registration.method.c_str());
    } else {
        ui::pill("Not aligned", pal.warning);
    }
    if (scan.registration.landmarkRms > 0)
        ImGui::Text("Landmark RMS: %.2f mm", scan.registration.landmarkRms);
    if (deviation_ && deviation_->samples > 0) {
        ImGui::Text("Surface distance: mean %.2f mm, 90%% < %.2f mm", deviation_->mean, deviation_->p90);
        ImGui::ProgressBar(static_cast<float>(deviation_->fractionWithin), ImVec2(-FLT_MIN, 0),
                           std::format("{:.0f}% within 0.5 mm", deviation_->fractionWithin * 100.0).c_str());
        ui::wrappedMutedText("Points on gingiva have no CBCT counterpart, so 100% is not expected.");
    }
    ImGui::Spacing();
    ui::wrappedMutedText("Verify: the scan outline in the slice views should hug the tooth crowns.");
    ImGui::Spacing();
    const float w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    bool canUndo = false;
    for (const auto& u : undo_)
        canUndo |= u.first == scan.id;
    if (ui::button("Undo", ImVec2(w, 0), canUndo)) {
        for (auto it = undo_.rbegin(); it != undo_.rend(); ++it) {
            if (it->first == scan.id) {
                scan.setTransform(it->second);
                undo_.erase(std::next(it).base());
                app.markModified();
                updateDeviation(app);
                break;
            }
        }
    }
    ImGui::SameLine();
    if (ui::button("Reset", ImVec2(w, 0), true)) {
        pushUndo(scan);
        scan.setTransform(scan.initialTransform);
        scan.registration = {};
        residuals_.clear();
        deviation_.reset();
        app.markModified();
        app.fitAllViews();
    }
    ui::endCard();
}

std::unique_ptr<Step> makeScanAlignmentStep()
{
    return std::make_unique<ScanAlignmentStep>();
}

} // namespace occlusa::designer
