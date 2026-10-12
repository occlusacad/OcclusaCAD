// Crown & bridge tools: margin line, insertion axis and crown design.
#include "apps/designer/CrownSteps.h"

#include "apps/designer/DesignerApp.h"
#include "core/Dental.h"
#include "core/Geometry.h"
#include "core/Log.h"
#include "core/crown/Brush.h"
#include "core/crown/InsertionAxis.h"
#include "core/crown/ToothLibrary.h"
#include "core/Platform.h"
#include "ui/FileDialog.h"
#include "ui/Fonts.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"

#include <imgui.h>
#include <imgui_stdlib.h>
#include <json.hpp>

#include <cmath>
#include <format>
#include <fstream>
#include <limits>
#include <map>
#include <set>

namespace occlusa::designer {

using workflow::StepId;

namespace {

constexpr ImU32 kControlColor = IM_COL32(255, 150, 40, 255);
const glm::vec3 kMarginColor(1.0f, 0.55f, 0.12f);
const glm::vec3 kMarginColorInactive(0.75f, 0.55f, 0.35f);
const glm::vec3 kCrownColor(0.97f, 0.94f, 0.86f);

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

bool isToothBorne(const std::string& type)
{
    const auto* t = dental::findRestorationType(type);
    return t && std::string_view(t->workflow) == "crown_bridge";
}

ScanObject* scanById(DesignerApp& app, int id)
{
    return id ? app.doc().findScan(id) : nullptr;
}

std::shared_ptr<const crown::PrepScan> prepScanFor(DesignerApp& app, int scanId)
{
    auto it = app.doc().prepScans.find(scanId);
    return it == app.doc().prepScans.end() ? nullptr : it->second;
}

void requestPrepScan(DesignerApp& app, int scanId)
{
    Document& doc = app.doc();
    if (!scanId || doc.prepScans.count(scanId) || doc.prepScansRequested.count(scanId))
        return;
    ScanObject* s = doc.findScan(scanId);
    if (!s || !s->mesh)
        return;
    doc.prepScansRequested.insert(scanId);
    std::shared_ptr<const Mesh> mesh = s->mesh;
    app.enqueue([&app, scanId, mesh] {
        const bool started = app.tasks().start("Analysing scan", [&app, scanId, mesh](const ProgressFn& progress) -> ui::TaskRunner::Continuation {
            reportProgress(progress, 0.2f, "Curvature and spatial index");
            auto ps = std::make_shared<const crown::PrepScan>(mesh);
            return [&app, scanId, ps] {
                app.doc().prepScans[scanId] = ps;
                app.doc().prepScansRequested.erase(scanId);
                app.doc().redraw();
            };
        });
        if (!started)
            app.doc().prepScansRequested.erase(scanId);
    });
}

RestorationDesign* findRestoration(DesignerApp& app, int tooth)
{
    for (auto& r : app.doc().restorations)
        if (r.tooth == tooth)
            return &r;
    return nullptr;
}

std::string bridgeName(const DesignerApp& app, const BridgeDesign& b)
{
    return dental::toothList(b.teeth, app.numbering(), "-");
}

void initParams(RestorationDesign& r)
{
    r.params.kind = crown::toothKindFromFdi(r.tooth);
    r.params.upper = dental::isUpper(r.tooth);
    r.params.coping = r.type == "coping";
}

void assignDefaultScans(DesignerApp& app, RestorationDesign& r)
{
    auto& scans = app.doc().scans;
    if (scans.empty())
        return;
    const bool upper = r.tooth ? dental::isUpper(r.tooth) : false;
    const db::FileRole own = upper ? db::FileRole::ScanUpper : db::FileRole::ScanLower;
    const db::FileRole opposing = upper ? db::FileRole::ScanLower : db::FileRole::ScanUpper;
    if (!scanById(app, r.prepScanId)) {
        r.prepScanId = scans.front()->id;
        for (const auto& s : scans)
            if (s->role == own) {
                r.prepScanId = s->id;
                break;
            }
    }
    if (r.antagonistScanId && !scanById(app, r.antagonistScanId))
        r.antagonistScanId = 0;
    if (!r.antagonistScanId) {
        for (const auto& s : scans)
            if (s->role == opposing && s->id != r.prepScanId)
                r.antagonistScanId = s->id;
        if (!r.antagonistScanId && scans.size() == 2)
            for (const auto& s : scans)
                if (s->id != r.prepScanId && s->role != db::FileRole::ScanBite)
                    r.antagonistScanId = s->id;
    }
}

glm::dmat4 scanTransform(DesignerApp& app, int scanId)
{
    ScanObject* s = scanById(app, scanId);
    return s ? s->transform : glm::dmat4(1.0);
}

// Point on a scan under a world-space ray, in the scan's own coordinates.
std::optional<glm::vec3> pickOnScan(DesignerApp& app, int scanId, const Ray& ray)
{
    ScanObject* s = scanById(app, scanId);
    if (!s || !s->mesh)
        return std::nullopt;
    const glm::dmat4 inv = glm::inverse(s->transform);
    if (auto ps = prepScanFor(app, scanId)) {
        const glm::dvec3 o = transformPoint(inv, ray.origin);
        const glm::dvec3 d = glm::normalize(transformVector(inv, ray.direction));
        if (auto hit = ps->bvh().raycast(glm::vec3(o), glm::vec3(d)))
            return hit->point;
        return std::nullopt;
    }
    if (auto hit = raycastMesh(*s->mesh, s->transform, ray))
        return glm::vec3(transformPoint(inv, hit->point));
    return std::nullopt;
}

// Margin points of an abutment, or the basal outline of a pontic.
std::vector<glm::vec3> outlinePoints(DesignerApp& app, const RestorationDesign& r)
{
    std::vector<glm::vec3> pts;
    if (r.isPontic()) {
        if (r.die)
            for (std::uint32_t v : r.die->boundary)
                pts.push_back(r.die->mesh.positions[v]);
        return pts;
    }
    if (ScanObject* s = scanById(app, r.prepScanId); s && !r.margin.vertices.empty())
        pts = r.margin.points(*s->mesh);
    return pts;
}

Aabb marginBoundsWorld(DesignerApp& app, const RestorationDesign& r)
{
    Aabb box;
    ScanObject* s = scanById(app, r.prepScanId);
    if (!s)
        return box;
    std::vector<const RestorationDesign*> units{&r};
    if (BridgeDesign* b = app.doc().bridgeOf(r.tooth)) {
        units.clear();
        for (int t : b->teeth)
            if (RestorationDesign* u = findRestoration(app, t))
                units.push_back(u);
    }
    for (const RestorationDesign* u : units)
        for (const auto& p : outlinePoints(app, *u))
            box.expand(transformPoint(s->transform, glm::dvec3(p)));
    if (box.valid()) {
        const glm::dvec3 pad(5.0);
        box.min -= pad;
        box.max += pad;
    }
    return box;
}

void focusOn(DesignerApp& app, const RestorationDesign& r)
{
    const Aabb box = marginBoundsWorld(app, r);
    if (box.valid())
        app.view3D(ViewId::Main3D).focus(box);
}

void lookAlong(DesignerApp& app, const RestorationDesign& r, const glm::dvec3& axisLocal)
{
    const glm::dmat4 t = scanTransform(app, r.prepScanId);
    const glm::dvec3 a = glm::normalize(transformVector(t, axisLocal));
    glm::dvec3 up = r.orientation ? transformVector(t, r.orientation->buccal) : glm::dvec3(0, 0, 1);
    if (std::abs(glm::dot(up, a)) > 0.95)
        up = glm::dvec3(1, 0, 0);
    gfx::Camera& cam = app.view3D(ViewId::Main3D).camera;
    cam.setView(-a, up);
    focusOn(app, r);
}

// View the restoration from occlusal, tilted towards buccal so the anatomy reads in 3D.
void viewOcclusal(DesignerApp& app, const RestorationDesign& r, double tiltDeg)
{
    if (!r.insertionAxis && r.marginAxis == glm::dvec3(0.0))
        return;
    const glm::dvec3 axis = r.insertionAxis ? *r.insertionAxis : r.marginAxis;
    glm::dvec3 buccal = r.orientation ? r.orientation->buccal : glm::dvec3(0.0);
    buccal -= glm::dot(buccal, axis) * axis;
    if (glm::length(buccal) < 1e-6) {
        lookAlong(app, r, axis);
        return;
    }
    buccal = glm::normalize(buccal);
    const double t = glm::radians(tiltDeg);
    lookAlong(app, r, axis * std::cos(t) + buccal * std::sin(t));
}

// Look at a whole scan from its occlusal side (the mean normal of an intraoral scan points occlusally).
void viewScanOcclusal(DesignerApp& app, int scanId)
{
    ScanObject* s = scanById(app, scanId);
    if (!s || s->mesh->normals.empty())
        return;
    glm::dvec3 n(0.0);
    for (const auto& v : s->mesh->normals)
        n += glm::dvec3(v);
    if (glm::length(n) < 1e-9)
        return;
    n = glm::normalize(transformVector(s->transform, n));
    gfx::Camera& cam = app.view3D(ViewId::Main3D).camera;
    glm::dvec3 up = std::abs(n.z) < 0.9 ? glm::dvec3(0, 0, 1) : glm::dvec3(0, 1, 0);
    cam.setView(-n, up);
    app.view3D(ViewId::Main3D).focus(s->worldBounds());
}

void setAntagonistVisible(DesignerApp& app, bool visible, float opacity = 1.0f)
{
    for (auto& r : app.doc().restorations)
        if (ScanObject* ant = scanById(app, r.antagonistScanId)) {
            ant->visible = visible;
            ant->opacity = opacity;
        }
    app.doc().redraw();
}

bool antagonistVisible(DesignerApp& app)
{
    RestorationDesign* r = app.doc().active();
    ScanObject* ant = r ? scanById(app, r->antagonistScanId) : nullptr;
    return ant && ant->visible;
}

std::optional<glm::vec3> inferPrepPoint(const crown::PrepScan& ps, const crown::MarginLine& margin)
{
    const auto pts = margin.points(ps.mesh());
    const glm::dvec3 c = crown::centroid(pts);
    const glm::dvec3 va = crown::vectorArea(pts);
    if (glm::length(va) < 1e-9)
        return std::nullopt;
    const glm::dvec3 n = glm::normalize(va);
    for (double sgn : {1.0, -1.0}) {
        const glm::vec3 dir(-sgn * n);
        if (auto hit = ps.bvh().raycast(glm::vec3(c + sgn * n * 25.0), dir, 50.0f))
            if (glm::dot(hit->faceNormal, dir) < 0.0f && glm::length(glm::dvec3(hit->point) - c) < 15.0)
                return hit->point;
    }
    return std::nullopt;
}

void depthColor(float depth, glm::vec3& out)
{
    if (depth <= 0.01f) {
        out = glm::vec3(0.55f, 0.78f, 0.58f);
        return;
    }
    const float t = std::clamp(depth / 0.3f, 0.0f, 1.0f);
    out = glm::mix(glm::vec3(1.0f, 0.85f, 0.2f), glm::vec3(0.9f, 0.15f, 0.12f), t);
}

glm::vec3 contactColor(float sd)
{
    if (std::isnan(sd) || sd > 0.6f)
        return kCrownColor;
    if (sd < -0.05f)
        return {0.85f, 0.10f, 0.10f};
    if (sd < 0.0f)
        return {0.98f, 0.45f, 0.10f};
    if (sd < 0.1f)
        return {0.98f, 0.85f, 0.15f};
    if (sd < 0.3f)
        return {0.35f, 0.80f, 0.35f};
    return glm::mix(glm::vec3(0.30f, 0.60f, 0.95f), kCrownColor, std::clamp((sd - 0.3f) / 0.3f, 0.0f, 1.0f));
}

std::string key(const char* kind, int tooth)
{
    return std::format("{}:{}", kind, tooth);
}

void updateMarginOverlay(DesignerApp& app, const RestorationDesign& r, bool active)
{
    Document& doc = app.doc();
    ScanObject* s = scanById(app, r.prepScanId);
    if (!s || r.margin.vertices.size() < 2) {
        doc.removeOverlay(key("margin", r.tooth));
        return;
    }
    auto pts = crown::smoothPolyline(r.margin.points(*s->mesh), r.margin.closed, 2);
    DisplayMesh dm;
    dm.gpu = std::make_unique<gfx::GpuMesh>(makeTube(pts, r.margin.closed, 0.07f, 8));
    dm.scanId = r.prepScanId;
    dm.color = active ? kMarginColor : kMarginColorInactive;
    doc.setOverlay(key("margin", r.tooth), std::move(dm));
}

void updateDieOverlay(DesignerApp& app, const RestorationDesign& r, bool visible)
{
    Document& doc = app.doc();
    if (!r.die || !r.insertionAxis) {
        doc.removeOverlay(key("die", r.tooth));
        return;
    }
    if (r.isPontic())
        return; // the ridge under a pontic has no undercut map
    Mesh m = r.die->mesh;
    const glm::dvec3 cen = crown::centroid(outlinePoints(app, r));
    const crown::Blockout bo = crown::computeBlockout(m, *r.insertionAxis, cen);
    m.colors.resize(m.positions.size());
    for (std::size_t i = 0; i < m.positions.size(); ++i)
        depthColor(bo.depth[i], m.colors[i]);
    DisplayMesh dm;
    dm.gpu = std::make_unique<gfx::GpuMesh>(m);
    dm.scanId = r.prepScanId;
    dm.vertexColors = true;
    dm.depthBias = true;
    dm.visible = visible;
    doc.setOverlay(key("die", r.tooth), std::move(dm));
}

// The outer part of a shell (its cap over the die would z-fight with the scan).
Mesh shellOuter(const crown::CrownMesh& c)
{
    Mesh m;
    m.positions = c.mesh.positions;
    m.normals = c.mesh.normals;
    m.colors = c.mesh.colors;
    for (std::size_t t = 0; t + 2 < c.mesh.indices.size(); t += 3)
        if (c.mesh.indices[t] >= c.outerBegin || c.mesh.indices[t + 1] >= c.outerBegin || c.mesh.indices[t + 2] >= c.outerBegin)
            m.indices.insert(m.indices.end(), {c.mesh.indices[t], c.mesh.indices[t + 1], c.mesh.indices[t + 2]});
    return m;
}

// What the 3D view shows of a restoration depends on the step: nothing while the margin and axis
// are edited, the crown bottom in its step, the free library tooth or the adapted shell during
// tooth placement and free-forming, and the merged crown from the merge step on.
void updateCrownOverlay(DesignerApp& app, const RestorationDesign& r, bool distanceMap)
{
    Document& doc = app.doc();
    const StepId step = app.currentStep();
    const std::string k = key("crown", r.tooth);
    const bool hidden = step == StepId::MarginLine || step == StepId::InsertionAxis || step == StepId::CrownBottom;
    DisplayMesh dm;
    dm.scanId = r.prepScanId;
    dm.color = kCrownColor;
    dm.visible = !hidden;
    const bool editing = step == StepId::ToothPlacement || step == StepId::Freeforming || step == StepId::Connectors;
    if (!editing && !hidden)
        if (const BridgeDesign* b = doc.bridgeOf(r.tooth); b && b->result && b->result->ok) {
            doc.removeOverlay(k); // the merged bridge is shown instead
            return;
        }
    const bool showMerged = r.merged && r.merged->ok && !editing;
    if (showMerged) {
        dm.gpu = std::make_unique<gfx::GpuMesh>(r.merged->mesh);
    } else if (r.adapted && r.crown) {
        Mesh m = shellOuter(*r.crown);
        if (distanceMap && r.contacts && step == StepId::Freeforming) {
            const auto d = crown::contactDistances(*r.crown, *r.contacts);
            m.colors.assign(m.positions.size(), kCrownColor);
            for (std::size_t v = r.crown->outerBegin; v < d.size(); ++v)
                m.colors[v] = contactColor(d[v]);
            dm.vertexColors = true;
        }
        dm.gpu = std::make_unique<gfx::GpuMesh>(m);
    } else if (r.placed && r.libraryTooth) {
        dm.gpu = std::make_unique<gfx::GpuMesh>(*r.libraryTooth);
        dm.color = glm::vec3(0.93f, 0.90f, 0.80f);
        dm.opacity = 0.85f;
    } else {
        doc.removeOverlay(k);
        return;
    }
    doc.setOverlay(k, std::move(dm));
}

void setOverlaysVisible(DesignerApp& app, const std::string& prefix, bool visible)
{
    for (auto& [k, o] : app.doc().overlays)
        if (k.rfind(prefix, 0) == 0)
            o.visible = visible;
    app.doc().redraw();
}

// Crowns and bridge connectors.
void setRestorationsVisible(DesignerApp& app, bool visible)
{
    setOverlaysVisible(app, "crown:", visible);
    setOverlaysVisible(app, "connector:", visible);
}

void refreshMarginOverlays(DesignerApp& app)
{
    Document& doc = app.doc();
    for (std::size_t i = 0; i < doc.restorations.size(); ++i)
        updateMarginOverlay(app, doc.restorations[i], static_cast<int>(i) == doc.activeRestoration);
}

bool ensureAxis(DesignerApp& app, RestorationDesign& r);
bool ensurePonticBase(DesignerApp& app, RestorationDesign& r);

bool ensureDie(DesignerApp& app, RestorationDesign& r)
{
    if (r.isPontic())
        return ensurePonticBase(app, r);
    if (r.die)
        return true;
    if (!r.marginClosed())
        return false;
    auto ps = prepScanFor(app, r.prepScanId);
    if (!ps) {
        requestPrepScan(app, r.prepScanId);
        return false;
    }
    if (!r.prepPoint) {
        if (auto p = inferPrepPoint(*ps, r.margin))
            r.prepPoint = glm::dvec3(*p);
        else {
            r.dieError = "Could not tell which side of the margin is the preparation.";
            return false;
        }
    }
    std::string err;
    auto die = crown::extractDie(*ps, r.margin, glm::vec3(*r.prepPoint), &err);
    if (!die) {
        r.dieError = err;
        return false;
    }
    r.dieError.clear();
    r.die = std::make_shared<const crown::DieRegion>(std::move(*die));
    const auto pts = r.margin.points(ps->mesh());
    const glm::dvec3 c = crown::centroid(pts);
    glm::dvec3 va = crown::vectorArea(pts);
    if (glm::dot(*r.prepPoint - c, va) < 0.0)
        va = -va;
    r.marginAxis = glm::length(va) > 1e-9 ? glm::normalize(va) : glm::dvec3(0, 0, 1);
    return true;
}

// One insertion axis for all abutments of a bridge (computed once all their dies exist).
bool ensureBridgeAxis(DesignerApp& app, BridgeDesign& b)
{
    if (b.axis)
        return true;
    std::vector<const Mesh*> dies;
    glm::dvec3 sum(0.0);
    for (int t : b.teeth) {
        RestorationDesign* u = findRestoration(app, t);
        if (!u || u->isPontic())
            continue;
        if (!ensureDie(app, *u))
            return false;
        dies.push_back(&u->die->mesh);
        sum += u->marginAxis;
    }
    if (dies.empty() || glm::length(sum) < 1e-9)
        return false;
    b.axis = crown::optimizeCommonAxis(dies, glm::normalize(sum));
    return true;
}

bool ensureAxis(DesignerApp& app, RestorationDesign& r)
{
    if (r.isPontic())
        return ensurePonticBase(app, r);
    if (!ensureDie(app, r))
        return false;
    if (BridgeDesign* b = app.doc().bridgeOf(r.tooth)) {
        if (!ensureBridgeAxis(app, *b))
            return false;
        if (!r.insertionAxis || *r.insertionAxis != *b->axis) {
            r.insertionAxis = *b->axis;
            r.invalidateBase();
        }
    } else if (!r.insertionAxis) {
        r.insertionAxis = crown::optimizeInsertionAxis(r.die->mesh, r.marginAxis);
    }
    if (!r.orientation) {
        ScanObject* s = scanById(app, r.prepScanId);
        crown::CrownFrame f;
        const auto pts = r.margin.points(*s->mesh);
        f.origin = crown::centroid(pts);
        f.axis = *r.insertionAxis;
        crown::estimateToothOrientation(*s->mesh, pts, f);
        r.orientation = f;
    }
    return true;
}

crown::CrownFrame currentFrame(DesignerApp& app, const RestorationDesign& r)
{
    crown::CrownFrame f = *r.orientation;
    f.origin = crown::centroid(outlinePoints(app, r));
    f.axis = *r.insertionAxis;
    return f;
}

// The pontic's ridge base, placed between the abutments of its bridge.
bool ensurePonticBase(DesignerApp& app, RestorationDesign& r)
{
    if (r.die)
        return true;
    BridgeDesign* b = app.doc().bridgeOf(r.tooth);
    if (!b) {
        r.dieError = "A pontic needs abutment crowns on a neighbouring tooth.";
        return false;
    }
    if (!ensureBridgeAxis(app, *b)) {
        r.dieError = "Define the margins of the abutments first.";
        return false;
    }
    const glm::dvec3 A = *b->axis;
    std::vector<crown::BridgeLayoutUnit> layout;
    std::size_t self = 0;
    glm::dvec3 abutmentMesial(0.0), abutmentBuccal(0.0);
    for (int t : b->teeth) {
        RestorationDesign* u = findRestoration(app, t);
        if (!u)
            return false;
        crown::BridgeLayoutUnit lu;
        lu.tooth = t;
        lu.pontic = u->isPontic();
        if (!lu.pontic) {
            if (!ensureAxis(app, *u))
                return false;
            lu.center = crown::centroid(outlinePoints(app, *u));
            lu.mesial = u->orientation->mesial;
            abutmentMesial = u->orientation->mesial;
            abutmentBuccal = u->orientation->buccal;
        }
        if (t == r.tooth)
            self = layout.size();
        layout.push_back(lu);
    }
    if (!crown::layoutPontics(layout, A)) {
        r.dieError = "The pontic could not be placed.";
        return false;
    }
    glm::dvec3 mesial = crown::ponticMesial(layout, self, A);
    if (glm::length(mesial) < 1e-9)
        mesial = abutmentMesial;
    auto ps = prepScanFor(app, r.prepScanId);
    if (!ps) {
        requestPrepScan(app, r.prepScanId);
        return false;
    }
    const crown::ToothTemplate& tpl = crown::toothTemplate(r.params.kind, r.params.upper);
    std::string err;
    auto die = crown::makePonticBase(ps->bvh(), layout[self].center, A, mesial, tpl.mesioDistal * 0.5 * 0.55, tpl.buccoLingual * 0.5 * 0.45,
                                     r.params.ridgeOffset, &err);
    if (!die) {
        r.dieError = err;
        return false;
    }
    r.dieError.clear();
    r.die = std::make_shared<const crown::DieRegion>(std::move(*die));
    r.insertionAxis = A;
    r.marginAxis = A;
    crown::CrownFrame f;
    f.origin = crown::centroid(outlinePoints(app, r));
    f.axis = A;
    f.mesial = glm::normalize(mesial - glm::dot(mesial, A) * A);
    f.buccal = glm::cross(A, f.mesial);
    if (glm::dot(f.buccal, abutmentBuccal) < 0.0)
        f.buccal = -f.buccal;
    r.orientation = f;
    r.prepPoint = f.origin;
    return true;
}

// Contacts of a bridge unit ignore the other units of its bridge (they are joined, not touching).
std::vector<std::pair<glm::dvec3, double>> bridgeExclusions(DesignerApp& app, const RestorationDesign& r)
{
    std::vector<std::pair<glm::dvec3, double>> out;
    if (BridgeDesign* b = app.doc().bridgeOf(r.tooth))
        for (int t : b->teeth) {
            RestorationDesign* u = findRestoration(app, t);
            if (!u || t == r.tooth)
                continue;
            const auto pts = outlinePoints(app, *u);
            glm::dvec3 c;
            double radius = 0.0;
            if (!pts.empty()) {
                c = crown::centroid(pts);
                for (const auto& p : pts)
                    radius = std::max(radius, glm::length(glm::dvec3(p) - c));
            } else {
                continue;
            }
            const crown::ToothTemplate& tpl = crown::toothTemplate(u->params.kind, u->params.upper);
            out.emplace_back(c, std::max(radius + 1.5, tpl.mesioDistal * 0.5));
        }
    return out;
}

bool ensureBase(DesignerApp& app, RestorationDesign& r)
{
    if (r.base && r.contacts)
        return true;
    if (!ensureAxis(app, r))
        return false;
    const crown::CrownFrame frame = currentFrame(app, r);
    if (!r.base)
        r.base = std::make_shared<const crown::CrownBase>(crown::makeCrownBase(*r.die, frame, r.effectiveParams()));
    if (!r.contacts) {
        ScanObject* prep = scanById(app, r.prepScanId);
        ScanObject* ant = scanById(app, r.antagonistScanId);
        const glm::dmat4 antToPrep = ant ? glm::inverse(prep->transform) * ant->transform : glm::dmat4(1.0);
        r.contacts = std::make_shared<const crown::ContactScene>(
            crown::makeContactScene(*prep->mesh, *r.base, ant ? ant->mesh.get() : nullptr, antToPrep, bridgeExclusions(app, r)));
    }
    return true;
}

// ---------------------------------------------------------------------------
// Bridges: connectors and merging
// ---------------------------------------------------------------------------

struct UnitSnapshot {
    int tooth = 0;
    bool pontic = false;
    std::shared_ptr<const crown::CrownBase> base;
    std::shared_ptr<const crown::CrownMesh> crown; // the shell (connector placement)
    std::shared_ptr<const Mesh> solid;             // the merged unit (bridge union)
};

// Connectors between neighbouring units: above the margins / ridge plus the embrasure clearance,
// below the lower of the two crowns' cusp tips, then the technician's edits. `warnings` gets one
// entry per connector (empty when it fits).
std::vector<crown::ConnectorSpec> placeConnectors(const std::vector<UnitSnapshot>& units, const glm::dvec3& A, const BridgeDesign& b,
                                                  std::vector<std::string>& warnings, dental::Numbering numbering)
{
    warnings.clear();
    std::vector<crown::ConnectorSpec> out;
    if (units.size() < 2)
        return out;
    const glm::dvec3 ref = units.front().base->frame.origin;
    auto outlineTop = [&](const UnitSnapshot& u) {
        double h = -1e9;
        for (const auto& p : u.base->margin)
            h = std::max(h, glm::dot(glm::dvec3(p) - ref, A));
        return h;
    };
    auto crownTop = [&](const UnitSnapshot& u) {
        double h = -1e9;
        for (const auto& p : u.crown->mesh.positions)
            h = std::max(h, glm::dot(glm::dvec3(p) - ref, A));
        return h;
    };
    for (std::size_t i = 0; i + 1 < units.size(); ++i) {
        const auto& a = units[i];
        const auto& c = units[i + 1];
        const double bottom = std::max(outlineTop(a), outlineTop(c)) + b.embrasure;
        const double top = std::min(crownTop(a), crownTop(c)) - 1.5;
        crown::ConnectorSpec spec = crown::placeConnector(a.base->frame.origin, c.base->frame.origin, A, ref, bottom, top, b.connectorArea,
                                                          b.connectorHeightRatio);
        spec = crown::applyConnectorEdit(spec, b.editAt(i));
        const auto fit = crown::checkConnectorFit(spec, ref, bottom, top);
        warnings.push_back(fit ? std::format("Connector {}-{} {}.", dental::toothLabel(a.tooth, numbering), dental::toothLabel(c.tooth, numbering), *fit)
                               : std::string());
        out.push_back(spec);
    }
    return out;
}

std::shared_ptr<const crown::BridgeUnion> mergeBridge(const std::vector<UnitSnapshot>& units, const std::vector<crown::ConnectorSpec>& connectors)
{
    std::vector<crown::BridgeUnionPart> parts;
    std::vector<Mesh> cavities;
    for (const auto& u : units) {
        crown::BridgeUnionPart part;
        part.mesh = u.solid ? u.solid.get() : &u.crown->mesh;
        if (!u.pontic) {
            part.ownCavity = static_cast<int>(cavities.size());
            cavities.push_back(crown::makeCavity(*u.base));
        }
        parts.push_back(part);
    }
    return std::make_shared<const crown::BridgeUnion>(crown::uniteBridge(parts, connectors, cavities));
}

// Units of a bridge with their current geometry (empty if any unit is not designed yet).
std::vector<UnitSnapshot> bridgeUnits(DesignerApp& app, const BridgeDesign& b)
{
    std::vector<UnitSnapshot> units;
    for (int t : b.teeth) {
        RestorationDesign* u = findRestoration(app, t);
        if (!u || !u->base || !u->crown || !u->adapted)
            return {};
        units.push_back({t, u->isPontic(), u->base, u->crown, nullptr});
    }
    return units;
}

void updateConnectorOverlays(DesignerApp& app, const BridgeDesign& b)
{
    Document& doc = app.doc();
    doc.removeOverlaysWithPrefix("connector:" + b.label() + ":");
    RestorationDesign* first = findRestoration(app, b.teeth.front());
    if (!first)
        return;
    const bool visible = app.currentStep() == StepId::Connectors;
    for (std::size_t i = 0; i < b.connectors.size(); ++i) {
        DisplayMesh dm;
        dm.gpu = std::make_unique<gfx::GpuMesh>(crown::makeConnectorMesh(b.connectors[i]));
        dm.scanId = first->prepScanId;
        const bool warn = i < b.warnings.size() && !b.warnings[i].empty();
        dm.color = static_cast<int>(i) == b.selectedConnector ? (warn ? glm::vec3(0.95f, 0.35f, 0.25f) : glm::vec3(1.0f, 0.62f, 0.25f))
                                                               : (warn ? glm::vec3(0.95f, 0.65f, 0.55f) : kCrownColor);
        dm.opacity = static_cast<int>(i) == b.selectedConnector ? 0.85f : 1.0f;
        dm.visible = visible;
        doc.setOverlay(std::format("connector:{}:{}", b.label(), i), std::move(dm));
    }
}

// A unit of the bridge changed: re-place the connectors; the merged bridge is out of date.
void bridgeChanged(DesignerApp& app, BridgeDesign& b)
{
    b.result.reset();
    b.warnings.clear();
    const auto units = bridgeUnits(app, b);
    b.connectors = (units.empty() || !b.axis) ? std::vector<crown::ConnectorSpec>{} : placeConnectors(units, *b.axis, b, b.warnings, app.numbering());
    updateConnectorOverlays(app, b);
}

// Rebuild what the restoration currently is: the adapted shell (with its free-forming) or the
// free library tooth. The merged crown is out of date afterwards.
void regenerate(DesignerApp& app, RestorationDesign& r, bool distanceMap)
{
    if (!ensureBase(app, r))
        return;
    const crown::CrownParameters params = r.effectiveParams();
    r.merged.reset();
    if (r.adapted) {
        auto c = std::make_shared<crown::CrownMesh>(crown::buildCrown(*r.base, params, r.displacement, true));
        if (!r.displacement.empty() && r.displacement.size() != c->mesh.vertexCount()) {
            r.displacement.clear();
            c = std::make_shared<crown::CrownMesh>(crown::buildCrown(*r.base, params, r.displacement, true));
        }
        r.crown = c;
        r.libraryTooth.reset();
    } else {
        r.crown.reset();
        r.libraryTooth = r.placed ? std::make_shared<const Mesh>(crown::placedLibraryTooth(*r.base, params)) : nullptr;
    }
    updateCrownOverlay(app, r, distanceMap);
    if (BridgeDesign* b = app.doc().bridgeOf(r.tooth))
        bridgeChanged(app, *b);
}

// The crown bottom solid (cement gap, border, minimum thickness); pontics have none.
bool ensureBottom(DesignerApp& app, RestorationDesign& r)
{
    if (r.isPontic())
        return true;
    if (r.bottom)
        return true;
    if (!ensureBase(app, r))
        return false;
    r.bottom = std::make_shared<const Mesh>(crown::makeCrownBottom(*r.base, r.effectiveParams()));
    return true;
}

void marginChanged(DesignerApp& app, RestorationDesign& r)
{
    r.invalidateDie();
    r.insertionAxis.reset();
    r.orientation.reset();
    r.displacement.clear();
    r.placed = r.adapted = false;
    app.doc().removeOverlay(key("crown", r.tooth));
    app.doc().removeOverlay(key("die", r.tooth));
    if (BridgeDesign* b = app.doc().bridgeOf(r.tooth)) {
        // The common axis and the pontic positions depend on every abutment's margin.
        b->axis.reset();
        b->result.reset();
        b->connectors.clear();
        app.doc().removeOverlaysWithPrefix("connector:" + b->label() + ":");
        for (int t : b->teeth)
            if (RestorationDesign* u = findRestoration(app, t); u && u != &r) {
                if (u->isPontic())
                    u->invalidateDie();
                else
                    u->invalidateBase();
                u->insertionAxis.reset();
                u->crown.reset();
                app.doc().removeOverlay(key("crown", t));
            }
    }
    refreshMarginOverlays(app);
    app.markModified();
}

void startMarginDetection(DesignerApp& app, int tooth, const glm::vec3& local)
{
    RestorationDesign* r = findRestoration(app, tooth);
    if (!r)
        return;
    auto ps = prepScanFor(app, r->prepScanId);
    if (!ps)
        return;
    app.tasks().start("Detecting margin", [&app, tooth, ps, local](const ProgressFn& progress) -> ui::TaskRunner::Continuation {
        reportProgress(progress, 0.3f, "Following the preparation edge");
        std::string err;
        auto det = crown::detectMargin(*ps, local, {}, &err);
        return [&app, tooth, det, err, local] {
            RestorationDesign* r = findRestoration(app, tooth);
            if (!r)
                return;
            if (!det) {
                ui::toast(ui::ToastKind::Warning, err);
                log::warn("Margin detection failed for tooth {}: {}", app.toothText(tooth), err);
                return;
            }
            r->controls = det->controls;
            r->margin = det->line;
            r->prepPoint = glm::dvec3(local);
            marginChanged(app, *r);
            r->marginAxis = det->occlusalDirection;
            viewOcclusal(app, *r, 35.0);
            ensureDie(app, *r);
            if (!r->dieError.empty())
                ui::toast(ui::ToastKind::Warning, r->dieError);
            if (ScanObject* s = scanById(app, r->prepScanId))
                log::info("Margin of tooth {} detected: {} points, {:.1f} mm", app.toothText(tooth), r->margin.vertices.size(),
                          crown::polylineLength(r->margin.points(*s->mesh), true));
        };
    });
}

void startBridgeAutoDesign(DesignerApp& app, BridgeDesign& b, bool distanceMap);

// Auto position (tooth placement): the library tooth is sized between the neighbours and its
// height fitted to the antagonist. It stays free (not attached to the margin).
void startAutoDesign(DesignerApp& app, int tooth, bool distanceMap)
{
    if (BridgeDesign* b = app.doc().bridgeOf(tooth)) {
        startBridgeAutoDesign(app, *b, distanceMap);
        return;
    }
    RestorationDesign* r = findRestoration(app, tooth);
    if (!r || !ensureBase(app, *r))
        return;
    auto base = r->base;
    auto contacts = r->contacts;
    crown::CrownParameters params = r->params;
    params.crownHeight = 0.0;
    params.halfMesial = params.halfDistal = params.halfBuccal = params.halfLingual = 0.0;
    params.rotationDeg = params.tiltMesialDeg = params.tiltBuccalDeg = 0.0;
    params.shiftMesial = params.shiftBuccal = params.shiftOcclusal = 0.0;
    app.tasks().start("Positioning the tooth", [&app, tooth, base, contacts, params, distanceMap](const ProgressFn& progress) -> ui::TaskRunner::Continuation {
        crown::CrownParameters p = params;
        reportProgress(progress, 0.1f, "Fitting to the neighbouring teeth");
        crown::fitProximal(*base, *contacts, p);
        reportProgress(progress, 0.5f, "Fitting to the antagonist");
        crown::fitOcclusalHeight(*base, *contacts, p, {}, true);
        return [&app, tooth, p, distanceMap] {
            RestorationDesign* r = findRestoration(app, tooth);
            if (!r)
                return;
            r->params = p;
            r->placed = true;
            regenerate(app, *r, distanceMap);
            app.markModified();
            log::info("Tooth {} positioned", app.toothText(tooth));
        };
    });
}

// Positions all units of a bridge together: outer contacts from the scan, inner sides meeting at
// the neighbours, heights from the antagonist.
void startBridgeAutoDesign(DesignerApp& app, BridgeDesign& b, bool distanceMap)
{
    struct Unit {
        int tooth;
        std::shared_ptr<const crown::CrownBase> base;
        std::shared_ptr<const crown::ContactScene> contacts;
        crown::CrownParameters params;
    };
    std::vector<Unit> units;
    for (int t : b.teeth) {
        RestorationDesign* u = findRestoration(app, t);
        if (!u || !ensureBase(app, *u)) {
            ui::toast(ui::ToastKind::Warning, std::format("Bridge {}: tooth {} is not ready ({}).", bridgeName(app, b), app.toothText(t),
                                                          u && !u->dieError.empty() ? u->dieError : "margin and axis missing"));
            return;
        }
        crown::CrownParameters p = u->effectiveParams();
        p.crownHeight = 0.0;
        p.halfMesial = p.halfDistal = p.halfBuccal = p.halfLingual = 0.0;
        p.rotationDeg = p.tiltMesialDeg = p.tiltBuccalDeg = 0.0;
        p.shiftMesial = p.shiftBuccal = p.shiftOcclusal = 0.0;
        units.push_back({t, u->base, u->contacts, p});
    }
    const std::string label = bridgeName(app, b);
    app.tasks().start("Positioning bridge " + label, [&app, units, label, distanceMap](const ProgressFn& progress) mutable -> ui::TaskRunner::Continuation {
        reportProgress(progress, 0.05f, "Fitting the units");
        for (auto& u : units)
            crown::fitProximal(*u.base, *u.contacts, u.params);
        std::vector<crown::BridgeFitUnit> fit;
        for (auto& u : units)
            fit.push_back({u.base->frame, &u.params});
        crown::fitBridgeWidths(fit);
        reportProgress(progress, 0.5f, "Fitting to the antagonist");
        for (auto& u : units)
            crown::fitOcclusalHeight(*u.base, *u.contacts, u.params, {}, true);
        return [&app, units, label, distanceMap] {
            for (const auto& u : units)
                if (RestorationDesign* r = findRestoration(app, u.tooth)) {
                    // Effective parameters of pontics are derived; keep the technician's own settings.
                    const crown::CrownParameters own = r->params;
                    r->params = u.params;
                    if (r->isPontic()) {
                        r->params.cementGap = own.cementGap;
                        r->params.borderHorizontal = own.borderHorizontal;
                        r->params.anticipateMilling = own.anticipateMilling;
                    }
                    r->placed = true;
                    regenerate(app, *r, distanceMap);
                }
            app.markModified();
            log::info("Bridge {} positioned", label);
        };
    });
}

// Adapt to margin: the positioned library tooth becomes a zero-thickness shell attached to the
// margin (all units of a bridge together). Free-forming starts from the library shape.
void adaptToMargin(DesignerApp& app, int tooth, bool distanceMap)
{
    std::vector<int> teeth = {tooth};
    if (BridgeDesign* b = app.doc().bridgeOf(tooth))
        teeth = b->teeth;
    for (int t : teeth)
        if (RestorationDesign* r = findRestoration(app, t); r && ensureBase(app, *r)) {
            r->placed = true;
            r->adapted = true;
            r->displacement.clear();
            regenerate(app, *r, distanceMap);
        }
    app.markModified();
}

// Merge: every crown united with its crown bottom minus the preparation space; bridges joined
// with their connectors (on a worker thread).
void startMergeAll(DesignerApp& app)
{
    struct Job {
        int tooth;
        bool pontic;
        std::shared_ptr<const crown::CrownBase> base;
        std::shared_ptr<const crown::CrownMesh> shell;
        std::shared_ptr<const Mesh> bottom;
    };
    struct BridgeJob {
        int firstTooth;
        std::vector<int> teeth;
        std::vector<crown::ConnectorSpec> connectors;
    };
    std::vector<Job> jobs;
    std::vector<BridgeJob> bridges;
    for (auto& r : app.doc().restorations) {
        if (!r.supported() || !r.adapted || !r.crown || !ensureBottom(app, r))
            continue;
        jobs.push_back({r.tooth, r.isPontic(), r.base, r.crown, r.isPontic() ? std::make_shared<const Mesh>() : r.bottom});
    }
    for (auto& b : app.doc().bridges) {
        if (b.connectors.empty())
            bridgeChanged(app, b);
        bridges.push_back({b.teeth.front(), b.teeth, b.connectors});
    }
    if (jobs.empty())
        return;
    app.tasks().start("Merging", [&app, jobs, bridges](const ProgressFn& progress) -> ui::TaskRunner::Continuation {
        std::map<int, std::shared_ptr<const crown::CrownMerge>> merged;
        for (std::size_t i = 0; i < jobs.size(); ++i) {
            const auto& j = jobs[i];
            reportProgress(progress, 0.8f * static_cast<float>(i) / static_cast<float>(jobs.size()), "Uniting the crown with its bottom");
            merged[j.tooth] = std::make_shared<const crown::CrownMerge>(crown::mergeCrown(*j.base, *j.shell, *j.bottom));
        }
        std::map<int, std::shared_ptr<const crown::BridgeUnion>> unions;
        for (const auto& b : bridges) {
            reportProgress(progress, 0.85f, "Joining the bridge");
            std::vector<UnitSnapshot> units;
            bool ok = true;
            for (int t : b.teeth) {
                auto it = merged.find(t);
                const Job* job = nullptr;
                for (const auto& j : jobs)
                    if (j.tooth == t)
                        job = &j;
                if (it == merged.end() || !it->second->ok || !job) {
                    ok = false;
                    break;
                }
                UnitSnapshot u{t, job->pontic, job->base, job->shell, nullptr};
                u.solid = std::shared_ptr<const Mesh>(it->second, &it->second->mesh);
                units.push_back(u);
            }
            if (ok && !b.connectors.empty())
                unions[b.firstTooth] = mergeBridge(units, b.connectors);
        }
        return [&app, merged, unions] {
            for (const auto& [tooth, m] : merged)
                if (RestorationDesign* r = findRestoration(app, tooth)) {
                    r->merged = m;
                    updateCrownOverlay(app, *r, false);
                    if (m->ok)
                        log::info("{} {} merged: {:.0f} mm3, min. thickness {:.2f} mm, {}", r->isPontic() ? "Pontic" : "Crown", app.toothText(tooth), m->volume,
                                  m->minThickness, m->watertight ? "watertight" : "NOT watertight");
                    else
                        log::warn("{} could not be merged: {}", app.toothText(tooth), m->error);
                }
            for (const auto& [first, u] : unions)
                if (BridgeDesign* b = app.doc().bridgeOf(first)) {
                    b->result = u;
                    if (u->ok)
                        log::info("Bridge {} merged: {:.0f} mm3, {}", bridgeName(app, *b), u->volume, u->watertight ? "watertight" : "NOT watertight");
                    else
                        log::warn("Bridge {} could not be merged: {}", bridgeName(app, *b), u->error);
                }
            app.doc().redraw();
        };
    });
}

void drawHint(ImDrawList* dl, const Projector& proj, const char* text)
{
    const float s = ImGui::GetStyle().FontScaleDpi;
    const ImVec2 ts = ImGui::CalcTextSize(text);
    const ImVec2 pos((proj.min.x + proj.max.x - ts.x) * 0.5f, proj.max.y - ts.y - 16 * s);
    dl->AddRectFilled(ImVec2(pos.x - 10 * s, pos.y - 5 * s), ImVec2(pos.x + ts.x + 10 * s, pos.y + ts.y + 5 * s), IM_COL32(15, 18, 22, 200), 6 * s);
    dl->AddRect(ImVec2(pos.x - 10 * s, pos.y - 5 * s), ImVec2(pos.x + ts.x + 10 * s, pos.y + ts.y + 5 * s), kControlColor, 6 * s, 1.5f * s);
    dl->AddText(pos, IM_COL32(240, 242, 245, 255), text);
}

std::string restorationLabel(const DesignerApp& app, const RestorationDesign& r)
{
    const auto* t = dental::findRestorationType(r.type);
    return std::format("{}  {}", app.toothText(r.tooth), t ? t->label : r.type.c_str());
}

// Restoration picker plus scan assignment. Returns true when the active restoration changed.
bool drawRestorationCard(DesignerApp& app, bool showScans)
{
    Document& doc = app.doc();
    const ui::Palette& pal = ui::palette();
    bool changed = false;
    ui::beginCard("##restoration");
    ui::subheading("Restoration");
    if (doc.restorations.empty()) {
        ui::wrappedMutedText(app.caseMode() ? "The case has no crowns or copings. Add a tooth to design one."
                                            : "No case is open. Choose the tooth to design.");
        static int number = 0;
        static int typeIdx = 0;
        const bool universal = app.numbering() == dental::Numbering::Universal;
        if (number == 0)
            number = universal ? 19 : 36;
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4.5f);
        ImGui::InputInt(universal ? "Tooth (Universal)" : "Tooth (FDI)", &number, 0, 0);
        const int fdi = dental::parseTooth(number, app.numbering());
        ImGui::SetNextItemWidth(-FLT_MIN);
        ui::segmented("newtype", {"Anatomic crown", "Coping"}, typeIdx);
        if (ui::primaryButton("Add restoration", ImVec2(-FLT_MIN, 0), dental::isValidFdi(fdi))) {
            RestorationDesign r;
            r.tooth = fdi;
            r.type = typeIdx == 0 ? "anatomic_crown" : "coping";
            initParams(r);
            assignDefaultScans(app, r);
            doc.restorations.push_back(std::move(r));
            doc.activeRestoration = static_cast<int>(doc.restorations.size()) - 1;
            changed = true;
        }
        ui::endCard();
        return changed;
    }
    RestorationDesign* act = doc.active();
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginCombo("##rest", act ? restorationLabel(app, *act).c_str() : "")) {
        for (std::size_t i = 0; i < doc.restorations.size(); ++i) {
            const auto& r = doc.restorations[i];
            std::string label = restorationLabel(app, r);
            if (const BridgeDesign* b = doc.bridgeOf(r.tooth))
                label += "  (bridge " + bridgeName(app, *b) + ")";
            if (!r.supported())
                label += "  (not supported yet)";
            else if (r.merged && r.merged->ok)
                label += "  - merged";
            else if (r.adapted)
                label += "  - adapted";
            else if (r.placed)
                label += "  - placed";
            else if (r.marginClosed())
                label += "  - margin set";
            if (ImGui::Selectable(label.c_str(), static_cast<int>(i) == doc.activeRestoration)) {
                doc.activeRestoration = static_cast<int>(i);
                changed = true;
            }
        }
        ImGui::EndCombo();
    }
    act = doc.active();
    if (act) {
        ui::mutedText("%s", dental::toothName(act->tooth).c_str());
        if (const BridgeDesign* b = doc.bridgeOf(act->tooth))
            ui::mutedText("Part of bridge %s (%zu units)", bridgeName(app, *b).c_str(), b->teeth.size());
        if (!act->supported()) {
            ImGui::Spacing();
            ImGui::TextColored(pal.warning, "This restoration type is not designed in this version.");
        } else if (act->isPontic() && !doc.bridgeOf(act->tooth)) {
            ImGui::Spacing();
            ImGui::TextColored(pal.warning, "A pontic needs a crown or coping on a neighbouring tooth.");
        }
    }
    if (act && showScans && !doc.scans.empty() && !act->isPontic()) {
        ImGui::Spacing();
        auto scanCombo = [&](const char* label, int& id, bool allowNone) {
            ScanObject* cur = scanById(app, id);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(label);
            ImGui::SameLine(ImGui::GetFontSize() * 6.5f);
            ImGui::SetNextItemWidth(-FLT_MIN);
            bool c = false;
            if (ImGui::BeginCombo(std::format("##{}", label).c_str(), cur ? cur->label.c_str() : "None")) {
                if (allowNone && ImGui::Selectable("None", id == 0)) {
                    id = 0;
                    c = true;
                }
                for (const auto& s : doc.scans)
                    if (ImGui::Selectable(s->label.c_str(), s->id == id)) {
                        id = s->id;
                        c = true;
                    }
                ImGui::EndCombo();
            }
            return c;
        };
        int prep = act->prepScanId;
        if (scanCombo("Preparation", prep, false) && prep != act->prepScanId) {
            act->prepScanId = prep;
            act->controls.clear();
            act->margin = {};
            act->prepPoint.reset();
            marginChanged(app, *act);
        }
        int ant = act->antagonistScanId;
        if (scanCombo("Antagonist", ant, true) && ant != act->antagonistScanId) {
            act->antagonistScanId = ant == act->prepScanId ? 0 : ant;
            act->contacts.reset();
            app.markModified();
        }
    }
    ui::endCard();
    if (changed)
        refreshMarginOverlays(app);
    return changed;
}

std::optional<std::string> firstMissing(const DesignerApp& appC, bool needMargin, bool needAxis, bool needCrown)
{
    auto& app = const_cast<DesignerApp&>(appC);
    const Document& doc = app.doc();
    if (doc.scans.empty())
        return "Load the scan with the preparation first.";
    if (doc.restorations.empty())
        return "Add the tooth to design.";
    for (const auto& r : doc.restorations) {
        if (!r.supported())
            continue;
        if (r.isPontic()) {
            if (!doc.bridgeOf(r.tooth))
                return std::format("Pontic {} needs a crown on a neighbouring tooth (bridge abutment).", app.toothText(r.tooth));
            if (needCrown && !r.adapted)
                return std::format("Adapt pontic {} (tooth placement).", app.toothText(r.tooth));
            continue;
        }
        if (needMargin && !r.marginClosed())
            return std::format("Define the margin line of tooth {}.", app.toothText(r.tooth));
        if (needMargin && !r.dieError.empty())
            return std::format("Tooth {}: {}", app.toothText(r.tooth), r.dieError);
        if (needAxis && !r.insertionAxis)
            return std::format("Set the insertion axis of tooth {}.", app.toothText(r.tooth));
        if (needCrown && !r.adapted)
            return std::format("Position tooth {} and adapt it to the margin.", app.toothText(r.tooth));
    }
    if (needCrown)
        for (const auto& b : doc.bridges)
            if (b.result && !b.result->ok)
                return std::format("Bridge {} could not be merged: {}", bridgeName(app, b), b.result->error);
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// Margin line
// ---------------------------------------------------------------------------

class MarginLineStep final : public Step {
public:
    MarginLineStep() : Step(StepId::MarginLine) {}

    ViewLayout preferredLayout() const override { return ViewLayout::Single3D; }

    std::optional<std::string> blocker(const DesignerApp& app) const override { return firstMissing(app, true, false, false); }

    void onEnter(DesignerApp& app) override
    {
        syncRestorations(app);
        app.view3D(ViewId::Main3D).pickCursor = true;
        setRestorationsVisible(app, false);
        setOverlaysVisible(app, "die:", false);
        refreshMarginOverlays(app);
        setAntagonistVisible(app, false);
        viewPending_ = true;
    }

    void onLeave(DesignerApp& app) override
    {
        if (RestorationDesign* r = app.doc().active(); r && dragging_ >= 0)
            finishDrag(app, *r);
        dragging_ = hovered_ = -1;
        app.view3D(ViewId::Main3D).pickCursor = false;
        setAntagonistVisible(app, true);
        setRestorationsVisible(app, true);
        for (auto& r : app.doc().restorations)
            if (r.marginClosed())
                ensureDie(app, r);
    }

    void update(DesignerApp& app) override
    {
        if (viewPending_ && !app.loading() && !restorationsPending(app)) {
            viewPending_ = false;
            setAntagonistVisible(app, false);
            if (RestorationDesign* r = app.doc().active()) {
                requestPrepScan(app, r->prepScanId);
                if (r->marginClosed())
                    viewOcclusal(app, *r, 35.0);
                else
                    viewScanOcclusal(app, r->prepScanId);
                mode_ = r->marginClosed() ? 2 : 0;
            }
        }
        if (RestorationDesign* r = app.doc().active(); r && r->supported() && !r->isPontic()) {
            requestPrepScan(app, r->prepScanId);
            // A detected margin is ready to edit.
            if (r->marginClosed() && !wasClosed_ && mode_ == 0)
                mode_ = 2;
            wasClosed_ = r->marginClosed();
        }
    }

    void drawPanel(DesignerApp& app) override
    {
        const ui::Palette& pal = ui::palette();
        if (drawRestorationCard(app, true))
            if (RestorationDesign* r = app.doc().active()) {
                requestPrepScan(app, r->prepScanId);
                focusOn(app, *r);
            }
        RestorationDesign* r = app.doc().active();
        if (!r || !r->supported())
            return;
        if (r->isPontic()) {
            ImGui::Spacing();
            ui::beginCard("##pontic");
            ui::subheading("Pontic");
            ui::wrappedMutedText("A pontic replaces a missing tooth and has no margin. It is placed between the abutments of its "
                                 "bridge and rests on the ridge; choose an abutment to define its margin.");
            ui::endCard();
            return;
        }
        requestPrepScan(app, r->prepScanId);
        const bool ready = prepScanFor(app, r->prepScanId) != nullptr;
        ImGui::Spacing();
        ui::beginCard("##margin");
        ui::subheading("Margin line");
        if (!ready) {
            ui::wrappedMutedText("Analysing the scan...");
        } else if (r->marginClosed()) {
            ScanObject* s = scanById(app, r->prepScanId);
            ImGui::TextColored(pal.success, "Closed");
            ImGui::SameLine();
            ui::mutedText("%.1f mm, %zu control points", crown::polylineLength(r->margin.points(*s->mesh), true), r->controls.size());
            if (!r->dieError.empty())
                ImGui::TextColored(pal.danger, "%s", r->dieError.c_str());
        } else if (!r->controls.empty()) {
            ui::mutedText("Drawing: %zu points", r->controls.size());
        } else {
            ui::mutedText("Not defined yet");
        }
        ImGui::Spacing();
        ImGui::SetNextItemWidth(-FLT_MIN);
        ui::segmented("marginmode", {"Detect", "Draw", "Edit"}, mode_);
        ImGui::Spacing();
        switch (mode_) {
        case 0: ui::wrappedMutedText("Click on the top of the preparation. The margin is found automatically by following the "
                                     "preparation edge all the way round."); break;
        case 1: ui::wrappedMutedText("Click points along the margin. Each segment follows the preparation edge. Click the first "
                                     "point again (or press Close) to finish. Drag a point to move it; point at one and press "
                                     "Delete to remove it."); break;
        default:
            ui::wrappedMutedText("Drag a point to move it along the preparation. Click on the margin line to add a point there. "
                                 "Point at a point and press Delete to remove it. The margin re-traces between the points.");
            break;
        }
        ImGui::Spacing();
        const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (mode_ == 1) {
            if (ui::button("Undo point", ImVec2(half, 0), !r->controls.empty())) {
                r->controls.pop_back();
                retrace(app, *r, false);
            }
            ImGui::SameLine();
            if (ui::button("Close", ImVec2(half, 0), r->controls.size() >= 3 && !r->margin.closed))
                retrace(app, *r, true);
        }
        if (ui::button("Clear margin", ImVec2(half, 0), !r->controls.empty())) {
            r->controls.clear();
            r->margin = {};
            r->prepPoint.reset();
            marginChanged(app, *r);
        }
        ImGui::SameLine();
        if (ui::button("Focus", ImVec2(half, 0), r->marginClosed()))
            focusOn(app, *r);
        ui::endCard();

        if (scanById(app, r->antagonistScanId)) {
            ImGui::Spacing();
            bool vis = antagonistVisible(app);
            if (ImGui::Checkbox("Show antagonist", &vis))
                setAntagonistVisible(app, vis);
        }
    }

    OverlayFn overlay(DesignerApp& app, ViewId view) override
    {
        if (view != ViewId::Main3D)
            return {};
        return [this, &app](ImDrawList* dl, const Projector& proj) {
            RestorationDesign* r = app.doc().active();
            if (!r) {
                dragging_ = hovered_ = -1;
                app.view3D(ViewId::Main3D).blockOrbit = false;
                return;
            }
            ScanObject* s = scanById(app, r->prepScanId);
            auto ps = prepScanFor(app, r->prepScanId);
            const bool editable = s && ps && mode_ != 0 && !app.tasks().busy() && r->supported() && !r->isPontic();
            const ImGuiIO& io = ImGui::GetIO();
            const ImVec2 mouse = io.MousePos;
            const bool inView = proj.contains(mouse) && ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
            const glm::dvec3 fwd = app.view3D(ViewId::Main3D).camera.forward();
            const float sc = ImGui::GetStyle().FontScaleDpi;

            // Dragging a point: it follows the mouse over the preparation; the margin re-traces live.
            if (dragging_ >= 0) {
                if (!editable || dragging_ >= static_cast<int>(r->controls.size()) || !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                    finishDrag(app, *r);
                } else if (auto ray = proj.rayAt(mouse)) {
                    if (auto hit = pickOnScan(app, r->prepScanId, *ray)) {
                        const std::uint32_t v = ps->nearestVertex(*hit);
                        auto& c = r->controls[static_cast<std::size_t>(dragging_)];
                        if (v != c) {
                            c = v;
                            const auto prepPoint = r->prepPoint;
                            r->margin = crown::traceMargin(*ps, r->controls, r->margin.closed);
                            marginChanged(app, *r);
                            r->prepPoint = prepPoint;
                        }
                    }
                }
            }

            // The point under the mouse (facing the camera).
            hovered_ = -1;
            if (s && editable && inView && dragging_ < 0) {
                float best = 9.0f * sc;
                for (std::size_t i = 0; i < r->controls.size(); ++i) {
                    const std::uint32_t v = r->controls[i];
                    if (glm::dot(transformVector(s->transform, glm::dvec3(s->mesh->normals[v])), fwd) > 0.25)
                        continue;
                    if (auto p = proj(transformPoint(s->transform, glm::dvec3(s->mesh->positions[v])))) {
                        const float d = std::hypot(p->x - mouse.x, p->y - mouse.y);
                        if (d <= best) {
                            best = d;
                            hovered_ = static_cast<int>(i);
                        }
                    }
                }
            }
            if (hovered_ >= 0) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    dragging_ = hovered_;
                    app.view3D(ViewId::Main3D).blockOrbit = true; // no orbit and no click while dragging
                } else if (!io.WantTextInput && (ImGui::IsKeyPressed(ImGuiKey_Delete, false) || ImGui::IsKeyPressed(ImGuiKey_Backspace, false))) {
                    removeControl(app, *r, static_cast<std::size_t>(hovered_));
                    hovered_ = -1;
                }
            } else if (editable && inView && dragging_ < 0 && mode_ == 2 && nearMarginLine(app, *r, proj, mouse)) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
            }

            if (s && !r->controls.empty()) {
                for (std::size_t i = 0; i < r->controls.size(); ++i) {
                    const std::uint32_t v = r->controls[i];
                    const glm::dvec3 n = transformVector(s->transform, glm::dvec3(s->mesh->normals[v]));
                    if (glm::dot(n, fwd) > 0.25)
                        continue; // facing away (behind the tooth)
                    if (auto p = proj(transformPoint(s->transform, glm::dvec3(s->mesh->positions[v])))) {
                        const bool active = static_cast<int>(i) == hovered_ || static_cast<int>(i) == dragging_;
                        const float rad = (active ? 6.0f : 4.5f) * sc;
                        dl->AddCircleFilled(*p, rad + 1.0f * sc, IM_COL32(0, 0, 0, 150));
                        dl->AddCircleFilled(*p, rad, active ? IM_COL32(255, 255, 255, 255)
                                                            : (i == 0 && !r->margin.closed ? IM_COL32(255, 255, 255, 255) : kControlColor));
                    }
                }
            }
            if (!prepScanFor(app, r->prepScanId))
                drawHint(dl, proj, "Analysing the scan...");
            else if (mode_ == 0 && !r->marginClosed())
                drawHint(dl, proj, "Click on the top of the preparation");
            else if (mode_ == 1 && !r->margin.closed)
                drawHint(dl, proj, r->controls.empty() ? "Click on the margin to start" : "Click the next margin point");
            else if (mode_ == 2 && hovered_ >= 0 && dragging_ < 0)
                drawHint(dl, proj, "Drag to move the point, Delete to remove it");
        };
    }

    void onViewEvent(DesignerApp& app, ViewId view, const ViewEvents& ev) override
    {
        if (view != ViewId::Main3D || !ev.click || app.tasks().busy() || dragging_ >= 0 || hovered_ >= 0)
            return; // a click on a point selects it for dragging; it does not add one
        RestorationDesign* r = app.doc().active();
        if (!r || !r->supported() || r->isPontic())
            return;
        auto ps = prepScanFor(app, r->prepScanId);
        if (!ps)
            return;
        const auto hit = pickOnScan(app, r->prepScanId, *ev.click);
        if (!hit)
            return;
        const std::uint32_t v = ps->nearestVertex(*hit);
        if (mode_ == 0) {
            startMarginDetection(app, r->tooth, *hit);
        } else if (mode_ == 1) {
            if (r->margin.closed) {
                r->controls.clear();
                r->margin = {};
                r->prepPoint.reset();
            }
            const auto& pos = ps->mesh().positions;
            if (r->controls.size() >= 3 && glm::length(pos[r->controls.front()] - pos[v]) < 0.8f) {
                retrace(app, *r, true);
            } else {
                r->controls.push_back(v);
                retrace(app, *r, false);
            }
        } else if (r->controls.size() >= 2 && nearMarginLine(app, *r, ev.projector, ImGui::GetIO().MousePos)) {
            insertControl(app, *r, v);
        }
    }

private:
    void retrace(DesignerApp& app, RestorationDesign& r, bool close)
    {
        auto ps = prepScanFor(app, r.prepScanId);
        if (!ps)
            return;
        const auto prepPoint = r.prepPoint;
        r.margin = crown::traceMargin(*ps, r.controls, close);
        marginChanged(app, r);
        if (r.margin.closed) {
            r.prepPoint = prepPoint; // keep a point from detection if there was one; otherwise inferred
            ensureDie(app, r);
            if (!r.dieError.empty())
                ui::toast(ui::ToastKind::Warning, r.dieError);
            mode_ = 2;
        }
    }

    // True when `screen` is within a few pixels of the drawn margin line.
    bool nearMarginLine(DesignerApp& app, const RestorationDesign& r, const Projector& proj, const ImVec2& screen) const
    {
        ScanObject* s = scanById(app, r.prepScanId);
        if (!s || r.margin.vertices.size() < 2)
            return false;
        const float limit = 10.0f * ImGui::GetStyle().FontScaleDpi;
        const glm::dvec3 fwd = app.view3D(ViewId::Main3D).camera.forward();
        std::optional<ImVec2> prev;
        const std::size_t n = r.margin.vertices.size();
        for (std::size_t i = 0; i <= n; ++i) {
            if (i == n && !r.margin.closed)
                break;
            const std::uint32_t v = r.margin.vertices[i % n];
            const auto p = proj(transformPoint(s->transform, glm::dvec3(s->mesh->positions[v])));
            // Only the visible part of the margin: a click over a hidden part would land on the surface in front.
            bool visible = p && glm::dot(transformVector(s->transform, glm::dvec3(s->mesh->normals[v])), fwd) <= 0.25;
            if (visible && (i % 3 == 0 || !prev)) { // occlusion is checked on every third vertex
                const auto ray = proj.rayAt(*p);
                const auto hit = ray ? pickOnScan(app, r.prepScanId, *ray) : std::nullopt;
                visible = hit && glm::length(*hit - s->mesh->positions[v]) < 0.3f;
            }
            if (!visible) {
                prev.reset();
                continue;
            }
            if (p && prev) {
                const glm::vec2 a(prev->x, prev->y), b(p->x, p->y), m(screen.x, screen.y);
                const glm::vec2 ab = b - a;
                const float t = glm::dot(ab, ab) > 0.0f ? std::clamp(glm::dot(m - a, ab) / glm::dot(ab, ab), 0.0f, 1.0f) : 0.0f;
                if (glm::length(m - (a + ab * t)) <= limit)
                    return true;
            }
            prev = p;
        }
        return false;
    }

    // Adds a control point where it lengthens the control polygon least (between the two points
    // it lies between along the margin).
    void insertControl(DesignerApp& app, RestorationDesign& r, std::uint32_t v)
    {
        auto ps = prepScanFor(app, r.prepScanId);
        if (!ps || std::find(r.controls.begin(), r.controls.end(), v) != r.controls.end())
            return;
        const auto& pos = ps->mesh().positions;
        float toLine = std::numeric_limits<float>::max();
        for (std::uint32_t m : r.margin.vertices)
            toLine = std::min(toLine, glm::length(pos[m] - pos[v]));
        if (toLine > 1.0f)
            return; // the click landed on another part of the scan
        const std::size_t n = r.controls.size();
        const std::size_t segments = r.margin.closed ? n : n - 1;
        std::size_t best = 0;
        float bestCost = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < segments; ++i) {
            const glm::vec3 a = pos[r.controls[i]], b = pos[r.controls[(i + 1) % n]], p = pos[v];
            const float cost = glm::length(p - a) + glm::length(b - p) - glm::length(b - a);
            if (cost < bestCost) {
                bestCost = cost;
                best = i;
            }
        }
        r.controls.insert(r.controls.begin() + static_cast<std::ptrdiff_t>(best + 1), v);
        retrace(app, r, r.margin.closed);
    }

    void removeControl(DesignerApp& app, RestorationDesign& r, std::size_t index)
    {
        if (index >= r.controls.size())
            return;
        if (r.margin.closed && r.controls.size() <= 3) {
            ui::toast(ui::ToastKind::Warning, "A closed margin needs at least three points.");
            return;
        }
        r.controls.erase(r.controls.begin() + static_cast<std::ptrdiff_t>(index));
        if (r.controls.empty()) {
            r.margin = {};
            r.prepPoint.reset();
            marginChanged(app, r);
            return;
        }
        retrace(app, r, r.margin.closed);
    }

    void finishDrag(DesignerApp& app, RestorationDesign& r)
    {
        dragging_ = -1;
        app.view3D(ViewId::Main3D).blockOrbit = false;
        if (r.marginClosed()) {
            ensureDie(app, r);
            if (!r.dieError.empty())
                ui::toast(ui::ToastKind::Warning, r.dieError);
        }
    }

    int mode_ = 0;
    bool viewPending_ = false;
    bool wasClosed_ = false;
    int hovered_ = -1;  // control point under the mouse
    int dragging_ = -1; // control point being dragged
};

// ---------------------------------------------------------------------------
// Insertion axis
// ---------------------------------------------------------------------------

class InsertionAxisStep final : public Step {
public:
    InsertionAxisStep() : Step(StepId::InsertionAxis) {}

    ViewLayout preferredLayout() const override { return ViewLayout::Single3D; }

    std::optional<std::string> blocker(const DesignerApp& app) const override { return firstMissing(app, true, true, false); }

    void onEnter(DesignerApp& app) override
    {
        syncRestorations(app);
        setRestorationsVisible(app, false);
        setAntagonistVisible(app, false);
        ready_ = false;
        prepare(app);
    }

    void onLeave(DesignerApp& app) override
    {
        setAntagonistVisible(app, true);
        setRestorationsVisible(app, true);
        setOverlaysVisible(app, "die:", false);
    }

    void update(DesignerApp& app) override
    {
        if (!ready_)
            prepare(app);
    }

    void drawPanel(DesignerApp& app) override
    {
        const ui::Palette& pal = ui::palette();
        if (drawRestorationCard(app, false)) {
            setOverlaysVisible(app, "die:", false);
            if (RestorationDesign* r = app.doc().active())
                if (ensureAxis(app, *r))
                    updateDieOverlay(app, *r, true);
        }
        RestorationDesign* r = app.doc().active();
        if (!r || !r->supported())
            return;
        BridgeDesign* bridge = app.doc().bridgeOf(r->tooth);
        if (r->isPontic()) {
            ImGui::Spacing();
            ui::beginCard("##axis");
            ui::subheading("Insertion axis");
            ui::wrappedMutedText(bridge ? "Pontics follow the common insertion axis of their bridge. Choose an abutment to change it."
                                        : "A pontic needs abutments on a neighbouring tooth.");
            ui::endCard();
            return;
        }
        ImGui::Spacing();
        ui::beginCard("##axis");
        ui::subheading(bridge ? "Common insertion axis" : "Insertion axis");
        if (!ensureAxis(app, *r)) {
            ui::wrappedMutedText(r->dieError.empty() ? "Define the margin line first." : r->dieError.c_str());
            ui::endCard();
            return;
        }
        if (!app.doc().overlays.count(key("die", r->tooth)))
            updateDieOverlay(app, *r, true);
        auto report = [&](RestorationDesign& u) {
            const glm::dvec3 cen = crown::centroid(outlinePoints(app, u));
            const crown::UndercutReport rep = crown::undercutReport(u.die->mesh, crown::computeBlockout(u.die->mesh, *u.insertionAxis, cen));
            if (rep.undercutArea < 0.5)
                ImGui::TextColored(pal.success, "No undercuts");
            else
                ImGui::TextColored(pal.warning, "Undercuts: %.1f mm2, up to %.2f mm deep", rep.undercutArea, rep.maxDepth);
        };
        if (bridge) {
            ui::wrappedMutedText(std::format("All abutments of bridge {} are seated along one axis.", bridgeName(app, *bridge)).c_str());
            for (int t : bridge->teeth) {
                RestorationDesign* u = findRestoration(app, t);
                if (!u || u->isPontic() || !ensureAxis(app, *u))
                    continue;
                // Divergence from the abutment's own best direction.
                auto& own = ownAxis_[t];
                if (!own.second || own.first != u->die.get())
                    own = {u->die.get(), std::make_optional(crown::optimizeInsertionAxis(u->die->mesh, u->marginAxis))};
                ImGui::TextUnformatted(app.toothText(t).c_str());
                ImGui::SameLine();
                report(*u);
                ImGui::SameLine();
                ui::mutedText("(%.1f deg from its own axis)", crown::axisDivergence(*own.second, *bridge->axis));
            }
        } else {
            report(*r);
        }
        ui::wrappedMutedText("Undercuts (yellow to red) are blocked out on the inside of the crown so it can be seated along "
                             "the axis.");
        ImGui::Spacing();

        // Tilt relative to the margin normal, towards mesial and buccal.
        const glm::dvec3 ref = r->marginAxis;
        glm::dvec3 m = r->orientation->mesial - glm::dot(r->orientation->mesial, ref) * ref;
        m = glm::normalize(m);
        const glm::dvec3 b = glm::cross(ref, m) * (glm::dot(glm::cross(ref, m), r->orientation->buccal) < 0 ? -1.0 : 1.0);
        const glm::dvec3 a = *r->insertionAxis;
        float tiltM = static_cast<float>(glm::degrees(std::atan2(glm::dot(a, m), glm::dot(a, ref))));
        float tiltB = static_cast<float>(glm::degrees(std::atan2(glm::dot(a, b), glm::dot(a, ref))));
        bool changed = false;
        ImGui::SetNextItemWidth(-FLT_MIN);
        changed |= ImGui::SliderFloat("##tm", &tiltM, -30.0f, 30.0f, "Mesial / distal  %.1f deg");
        ImGui::SetNextItemWidth(-FLT_MIN);
        changed |= ImGui::SliderFloat("##tb", &tiltB, -30.0f, 30.0f, "Buccal / lingual  %.1f deg");
        if (changed)
            setAxis(app, *r, glm::normalize(ref + m * std::tan(glm::radians(static_cast<double>(tiltM))) + b * std::tan(glm::radians(static_cast<double>(tiltB)))));
        ImGui::Spacing();
        const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (ui::button("Optimise", ImVec2(half, 0))) {
            if (bridge) {
                bridge->axis.reset();
                ensureBridgeAxis(app, *bridge);
                setAxis(app, *r, *bridge->axis);
            } else {
                setAxis(app, *r, crown::optimizeInsertionAxis(r->die->mesh, r->marginAxis));
            }
        }
        ImGui::SameLine();
        if (ui::button("Margin normal", ImVec2(half, 0)))
            setAxis(app, *r, r->marginAxis);
        if (ui::button("From view", ImVec2(half, 0))) {
            const glm::dmat4 inv = glm::inverse(scanTransform(app, r->prepScanId));
            glm::dvec3 d = glm::normalize(transformVector(inv, -app.view3D(ViewId::Main3D).camera.forward()));
            if (glm::dot(d, r->marginAxis) < 0.0)
                d = -d;
            setAxis(app, *r, d);
        }
        ImGui::SameLine();
        if (ui::button("Look along", ImVec2(half, 0)))
            lookAlong(app, *r, *r->insertionAxis);
        ui::endCard();
    }

    OverlayFn overlay(DesignerApp& app, ViewId view) override
    {
        if (view != ViewId::Main3D)
            return {};
        return [&app](ImDrawList* dl, const Projector& proj) {
            RestorationDesign* r = app.doc().active();
            if (!r || !r->insertionAxis || !r->die)
                return;
            const glm::dmat4 t = scanTransform(app, r->prepScanId);
            const glm::dvec3 a = *r->insertionAxis;
            const glm::dvec3 c = crown::centroid(outlinePoints(app, *r));
            double top = 0.0;
            for (const auto& p : r->die->mesh.positions)
                top = std::max(top, glm::dot(glm::dvec3(p) - c, a));
            const auto p0 = proj(transformPoint(t, c + a * (top + 1.0)));
            const auto p1 = proj(transformPoint(t, c + a * (top + 7.0)));
            if (!p0 || !p1)
                return;
            const float s = ImGui::GetStyle().FontScaleDpi;
            dl->AddLine(*p0, *p1, IM_COL32(0, 0, 0, 120), 5.0f * s);
            dl->AddLine(*p0, *p1, IM_COL32(80, 170, 255, 255), 3.0f * s);
            ImVec2 dir(p1->x - p0->x, p1->y - p0->y);
            const float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
            if (len > 1.0f) {
                dir = ImVec2(dir.x / len, dir.y / len);
                const ImVec2 perp(-dir.y, dir.x);
                const float h = 10.0f * s;
                dl->AddTriangleFilled(ImVec2(p1->x + dir.x * h, p1->y + dir.y * h), ImVec2(p1->x + perp.x * h * 0.6f, p1->y + perp.y * h * 0.6f),
                                      ImVec2(p1->x - perp.x * h * 0.6f, p1->y - perp.y * h * 0.6f), IM_COL32(80, 170, 255, 255));
            }
        };
    }

private:
    // Runs once the data and the scan analysis are available.
    void prepare(DesignerApp& app)
    {
        if (app.loading() || restorationsPending(app))
            return;
        RestorationDesign* active = app.doc().active();
        if (!active || !active->supported())
            return;
        if (!ensureAxis(app, *active))
            return;
        const BridgeDesign* bridge = app.doc().bridgeOf(active->tooth);
        for (auto& r : app.doc().restorations)
            if (r.supported() && !r.isPontic() && ensureAxis(app, r))
                updateDieOverlay(app, r, &r == active || (bridge && bridge->contains(r.tooth)));
        setAntagonistVisible(app, false);
        viewOcclusal(app, *active, 40.0);
        ready_ = true;
    }

    bool ready_ = false;
    std::map<int, std::pair<const void*, std::optional<glm::dvec3>>> ownAxis_; // abutment -> best own axis (cache)

    void setAxis(DesignerApp& app, RestorationDesign& r, const glm::dvec3& axisIn)
    {
        const glm::dvec3 axis = glm::normalize(axisIn);
        std::vector<RestorationDesign*> units{&r};
        if (BridgeDesign* b = app.doc().bridgeOf(r.tooth)) {
            b->axis = axis;
            b->result.reset();
            b->connectors.clear();
            app.doc().removeOverlaysWithPrefix("connector:" + b->label() + ":");
            units.clear();
            for (int t : b->teeth)
                if (RestorationDesign* u = findRestoration(app, t))
                    units.push_back(u);
        }
        for (RestorationDesign* u : units) {
            if (u->isPontic()) {
                u->invalidateDie(); // placed on the ridge along the axis
            } else {
                u->insertionAxis = axis;
                u->invalidateBase(); // rebuilt on the new axis when it is next shown
                updateDieOverlay(app, *u, true);
            }
            u->crown.reset();
            app.doc().removeOverlay(key("crown", u->tooth));
        }
        app.markModified();
    }
};

// ---------------------------------------------------------------------------
// Crown design
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// Crown bottom (the inside of the crown)
// ---------------------------------------------------------------------------

// Colours of the crown bottom map on the die.
glm::vec3 bottomColor(const crown::CrownBase& b, std::size_t i)
{
    if (b.blockoutDepth[i] > 0.02f)
        return glm::mix(glm::vec3(0.95f, 0.55f, 0.20f), glm::vec3(0.90f, 0.20f, 0.20f), std::clamp(b.blockoutDepth[i] / 0.5f, 0.0f, 1.0f));
    if (!b.toolCompensation.empty() && b.toolCompensation[i] > 0.01f)
        return {0.80f, 0.35f, 0.85f};
    return b.gap[i] > 0.005f ? glm::vec3(0.95f, 0.85f, 0.20f) : glm::vec3(0.35f, 0.80f, 0.35f);
}

class CrownBottomStep final : public Step {
public:
    CrownBottomStep() : Step(StepId::CrownBottom) {}

    ViewLayout preferredLayout() const override { return ViewLayout::Single3D; }
    std::optional<std::string> blocker(const DesignerApp& app) const override { return firstMissing(app, true, true, false); }

    void onEnter(DesignerApp& app) override
    {
        syncRestorations(app);
        setAntagonistVisible(app, false);
        setOverlaysVisible(app, "die:", false);
        setRestorationsVisible(app, false);
        viewPending_ = true;
    }

    void onLeave(DesignerApp& app) override
    {
        app.doc().removeOverlaysWithPrefix("bottommap:");
        app.doc().removeOverlaysWithPrefix("bottom:");
        setAntagonistVisible(app, true);
        setRestorationsVisible(app, true);
    }

    void update(DesignerApp& app) override
    {
        if (!viewPending_ || app.loading() || restorationsPending(app))
            return;
        RestorationDesign* r = app.doc().active();
        if (r && r->supported() && !r->isPontic()) {
            if (!prepScanFor(app, r->prepScanId)) {
                requestPrepScan(app, r->prepScanId);
                return;
            }
            viewPending_ = false;
            setAntagonistVisible(app, false); // the scans may have loaded after onEnter
            refresh(app);
            viewOcclusal(app, *r, 35.0);
        } else {
            viewPending_ = false;
        }
    }

    void drawPanel(DesignerApp& app) override
    {
        const ui::Palette& pal = ui::palette();
        if (drawRestorationCard(app, false)) {
            refresh(app);
            if (RestorationDesign* r = app.doc().active())
                focusOn(app, *r);
        }
        RestorationDesign* r = app.doc().active();
        if (!r || !r->supported())
            return;
        if (r->isPontic()) {
            ImGui::Spacing();
            ui::wrappedMutedText("A pontic rests on the ridge and has no crown bottom.");
            return;
        }
        if (!ensureBase(app, *r)) {
            ImGui::Spacing();
            ui::wrappedMutedText(r->dieError.empty() ? "Define the margin line and insertion axis first." : r->dieError.c_str());
            return;
        }
        crown::CrownParameters& p = r->params;
        bool changed = false;
        const float full = -FLT_MIN;
        // Sliders apply when released (the crown bottom takes a moment to rebuild).
        auto mm = [&](const char* id, double& value, float lo, float hi, const char* fmt) {
            float v = static_cast<float>(value);
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat(id, &v, lo, hi, fmt))
                value = v;
            changed |= ImGui::IsItemDeactivatedAfterEdit();
        };
        ImGui::Spacing();
        ImGui::SetNextItemWidth(full);
        ui::segmented("bottomtab", {"Gap", "Border", "Advanced"}, tab_);
        ImGui::Spacing();
        ui::beginCard("##bottom");
        if (tab_ == 0) {
            ui::subheading("Cement gap");
            mm("##gap", p.cementGap, 0.0f, 0.2f, "Gap  %.3f mm");
            mm("##from", p.distanceToMargin, 0.0f, 3.0f, "No gap from margin  %.2f mm");
            ui::mutedText("Yellow: cement gap. Green: no cement gap.");
            ImGui::Spacing();
            ui::subheading("Additional spacing");
            const double ax0 = p.axialSpacing, rad0 = p.radialSpacing;
            mm("##ax", p.axialSpacing, 0.0f, 0.2f, "Axial  %.3f mm");
            mm("##rad", p.radialSpacing, 0.0f, 0.2f, "Radial  %.3f mm");
            if (linkSpacing_) {
                if (p.axialSpacing != ax0)
                    p.radialSpacing = p.axialSpacing;
                else if (p.radialSpacing != rad0)
                    p.axialSpacing = p.radialSpacing;
            }
            ImGui::Checkbox("Link axial and radial", &linkSpacing_);
        } else if (tab_ == 1) {
            ui::subheading("Crown border");
            mm("##bh", p.borderHorizontal, 0.0f, 1.0f, "1. Horizontal  %.2f mm");
            mm("##ba", p.borderAngled, 0.0f, 1.5f, "2. Angled  %.2f mm");
            mm("##bang", p.borderAngleDeg, 0.0f, 90.0f, "3. Angle  %.0f\xC2\xB0");
            mm("##bv", p.borderVertical, 0.0f, 1.5f, "4. Vertical  %.2f mm");
            mm("##bm", p.belowMargin, -0.5f, 0.5f, "5. Below margin  %.2f mm");
            drawBorderDiagram(p);
            ui::wrappedMutedText("Zirconia commonly needs at least 0.2 mm horizontally. Angled and vertical are often 0.");
        } else {
            ui::subheading("Undercuts");
            bool keep = !p.blockOutUndercuts;
            if (ImGui::Checkbox("Don't block out undercuts", &keep)) {
                p.blockOutUndercuts = !keep;
                changed = true;
            }
            if (p.blockOutUndercuts || p.bullnoseTool) {
                mm("##boa", p.blockoutAngleDeg, 0.0f, 15.0f, "Angle  %.1f\xC2\xB0");
                mm("##bou", p.blockoutUntouchable, 0.0f, 2.0f, "Untouchable zone at margin  %.2f mm");
            }
            ImGui::Spacing();
            ui::subheading("Milling");
            if (ImGui::Checkbox("Anticipate milling (tool compensation)", &p.anticipateMilling))
                changed = true;
            if (p.anticipateMilling) {
                mm("##tool", p.toolDiameter, 0.3f, 2.5f, "Tool diameter  %.2f mm");
                ui::mutedText("Choose slightly above the smallest tool used.");
                if (ImGui::Checkbox("Bullnose / flat tool", &p.bullnoseTool))
                    changed = true;
                if (p.bullnoseTool)
                    mm("##flat", p.toolFlatPercent, 0.0f, 100.0f, "Tool tip flat  %.0f %%");
            } else {
                ui::mutedText("3D printing: no tool compensation.");
            }
            ImGui::Spacing();
            mm("##minth", p.minThickness, 0.2f, 2.0f, "Minimum thickness  %.2f mm");
        }
        ui::endCard();

        if (r->base) {
            const auto& b = *r->base;
            float bo = 0.0f, tc = 0.0f;
            for (std::size_t i = 0; i < b.blockoutDepth.size(); ++i) {
                bo = std::max(bo, b.blockoutDepth[i]);
                tc = std::max(tc, b.toolCompensation.empty() ? 0.0f : b.toolCompensation[i]);
            }
            ImGui::Spacing();
            ui::mutedText("Blocked out up to %.2f mm, tool compensation up to %.2f mm", bo, tc);
            auto legend = [&](glm::vec3 c, const char* t) {
                ImGui::ColorButton(t, ImVec4(c.r, c.g, c.b, 1.0f), ImGuiColorEditFlags_NoTooltip, ImVec2(ImGui::GetFontSize(), ImGui::GetFontSize()));
                ImGui::SameLine();
                ui::mutedText("%s", t);
            };
            legend({0.35f, 0.80f, 0.35f}, "No gap");
            ImGui::SameLine();
            legend({0.95f, 0.85f, 0.20f}, "Gap");
            ImGui::SameLine();
            legend({0.90f, 0.20f, 0.20f}, "Blocked out");
            ImGui::SameLine();
            legend({0.80f, 0.35f, 0.85f}, "Tool");
        }
        ImGui::Spacing();
        if (ImGui::Checkbox("Show crown bottom solid", &showSolid_))
            refresh(app);
        if (ui::button("Apply to all crowns", ImVec2(full, 0), app.doc().restorations.size() > 1)) {
            for (auto& u : app.doc().restorations)
                if (&u != r && !u.isPontic()) {
                    const crown::CrownParameters own = u.params;
                    u.params = p;
                    // Keep everything that is not about the crown bottom.
                    u.params.library = own.library;
                    u.params.kind = own.kind;
                    u.params.upper = own.upper;
                    u.params.coping = own.coping;
                    u.params.copingThickness = own.copingThickness;
                    u.params.crownHeight = own.crownHeight;
                    u.params.halfMesial = own.halfMesial;
                    u.params.halfDistal = own.halfDistal;
                    u.params.halfBuccal = own.halfBuccal;
                    u.params.halfLingual = own.halfLingual;
                    u.params.rotationDeg = own.rotationDeg;
                    u.params.tiltMesialDeg = own.tiltMesialDeg;
                    u.params.tiltBuccalDeg = own.tiltBuccalDeg;
                    u.params.shiftMesial = own.shiftMesial;
                    u.params.shiftBuccal = own.shiftBuccal;
                    u.params.shiftOcclusal = own.shiftOcclusal;
                    u.params.cuspScale = own.cuspScale;
                    u.params.flipMesioDistal = own.flipMesioDistal;
                    u.params.flipBuccoLingual = own.flipBuccoLingual;
                    u.params.proximalTarget = own.proximalTarget;
                    u.params.occlusalTarget = own.occlusalTarget;
                    rebuildBase(u);
                }
            ui::toast(ui::ToastKind::Success, "Crown bottom settings applied to all crowns.");
            app.markModified();
        }
        (void)pal;
        if (changed) {
            rebuildBase(*r);
            refresh(app);
            app.markModified();
        }
    }

private:
    // The base (intaglio) and everything built on it; the contact scene does not depend on it.
    static void rebuildBase(RestorationDesign& r)
    {
        r.base.reset();
        r.bottom.reset();
        r.crown.reset();
        r.libraryTooth.reset();
        r.merged.reset();
    }

    void refresh(DesignerApp& app)
    {
        Document& doc = app.doc();
        doc.removeOverlaysWithPrefix("bottommap:");
        doc.removeOverlaysWithPrefix("bottom:");
        for (std::size_t i = 0; i < doc.restorations.size(); ++i) {
            RestorationDesign& r = doc.restorations[i];
            if (!r.supported() || r.isPontic() || !ensureBase(app, r))
                continue;
            const bool active = static_cast<int>(i) == doc.activeRestoration;
            Mesh m = *r.base->intaglio;
            m.colors.resize(m.positions.size());
            for (std::size_t v = 0; v < m.positions.size(); ++v)
                m.colors[v] = bottomColor(*r.base, v);
            DisplayMesh dm;
            dm.gpu = std::make_unique<gfx::GpuMesh>(m);
            dm.scanId = r.prepScanId;
            dm.vertexColors = true;
            dm.depthBias = true;
            dm.opacity = active ? 1.0f : 0.6f;
            doc.setOverlay(key("bottommap", r.tooth), std::move(dm));
            if (showSolid_ && active && ensureBottom(app, r)) {
                DisplayMesh sm;
                sm.gpu = std::make_unique<gfx::GpuMesh>(*r.bottom);
                sm.scanId = r.prepScanId;
                sm.color = glm::vec3(0.85f, 0.88f, 0.95f);
                sm.opacity = 0.35f;
                doc.setOverlay(key("bottom", r.tooth), std::move(sm));
            }
        }
    }

    // The border profile as in exocad's dialog: preparation, margin, and the numbered parts.
    static void drawBorderDiagram(const crown::CrownParameters& p)
    {
        const float s = ImGui::GetFontSize();
        const ImVec2 size(ImGui::GetContentRegionAvail().x, s * 6.0f);
        const ImVec2 o = ImGui::GetCursorScreenPos();
        ImGui::Dummy(size);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const ui::Palette& pal = ui::palette();
        // Scale so the border fills about half the width (at least 0.6 mm shown).
        const double ang0 = glm::radians(p.borderAngleDeg);
        const double reach = std::max(0.6, p.borderHorizontal + std::cos(ang0) * p.borderAngled + 0.2);
        const double rise = std::max(0.4, std::sin(ang0) * p.borderAngled + p.borderVertical + 0.2);
        const float scale = static_cast<float>(std::min(size.x * 0.5 / reach, size.y * 0.6 / rise)); // pixels per mm
        const ImVec2 m(o.x + size.x * 0.3f, o.y + size.y * 0.8f + static_cast<float>(std::clamp(p.belowMargin, -0.2, 0.2)) * scale);
        // Preparation (grey) and its shoulder.
        dl->AddRectFilled(ImVec2(o.x + 4, o.y + 4), ImVec2(m.x, o.y + size.y - 4), IM_COL32(150, 150, 150, 120));
        dl->AddRectFilled(ImVec2(o.x + 4, m.y), ImVec2(m.x + 4, o.y + size.y - 4), IM_COL32(60, 60, 60, 160));
        const double ang = glm::radians(p.borderAngleDeg);
        const ImVec2 p1(m.x + static_cast<float>(p.borderHorizontal) * scale, m.y);
        const ImVec2 p2(p1.x + static_cast<float>(std::cos(ang) * p.borderAngled) * scale, p1.y - static_cast<float>(std::sin(ang) * p.borderAngled) * scale);
        const ImVec2 p3(p2.x, p2.y - static_cast<float>(p.borderVertical) * scale);
        const ImU32 col = ImGui::GetColorU32(pal.accent);
        const float w = 2.5f;
        dl->AddLine(m, p1, col, w);
        dl->AddLine(p1, p2, col, w);
        dl->AddLine(p2, p3, col, w);
        dl->AddLine(p3, ImVec2(p3.x - s * 0.8f, o.y + 6), col, w); // on to the crown wall
        dl->AddCircleFilled(m, s * 0.22f, IM_COL32(40, 190, 70, 255));
        auto label = [&](ImVec2 at, const char* t) { dl->AddText(ImVec2(at.x + 3, at.y - s * 1.1f), ImGui::GetColorU32(pal.textMuted), t); };
        label(ImVec2((m.x + p1.x) * 0.5f, m.y), "1");
        if (p.borderAngled > 0.0)
            label(ImVec2((p1.x + p2.x) * 0.5f + 4, (p1.y + p2.y) * 0.5f), "2");
        if (p.borderVertical > 0.0)
            label(ImVec2(p3.x + 4, (p2.y + p3.y) * 0.5f + s * 0.5f), "4");
    }

    int tab_ = 0;
    bool linkSpacing_ = true;
    bool showSolid_ = false;
    bool viewPending_ = false;
};

// ---------------------------------------------------------------------------
// Tooth placement, free-forming and connectors
// ---------------------------------------------------------------------------

// Folder of the technician's own free-forming tools.
std::filesystem::path brushFolder()
{
    return platform::configDir() / "brushes";
}

std::string brushSlug(const std::string& name)
{
    std::string out;
    for (char ch : name) {
        if (std::isalnum(static_cast<unsigned char>(ch)))
            out += static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        else if (!out.empty() && out.back() != '-')
            out += '-';
    }
    while (!out.empty() && out.back() == '-')
        out.pop_back();
    return out.empty() ? "tool" : out;
}

class CrownToolStep final : public Step {
public:
    explicit CrownToolStep(StepId id) : Step(id) {}

    ViewLayout preferredLayout() const override { return ViewLayout::Single3D; }

    std::optional<std::string> blocker(const DesignerApp& app) const override
    {
        if (id() == StepId::ToothPlacement)
            return firstMissing(app, true, true, true);
        return firstMissing(app, true, true, true);
    }

    void onEnter(DesignerApp& app) override
    {
        syncRestorations(app);
        app.view3D(ViewId::Main3D).pickCursor = id() == StepId::Freeforming && brushOn_;
        setOverlaysVisible(app, "die:", false);
        setAntagonistVisible(app, id() == StepId::ToothPlacement);
        setOverlaysVisible(app, "connector:", id() == StepId::Connectors);
        ready_ = false;
        loadCustomTools();
        prepare(app);
    }

    void onLeave(DesignerApp& app) override
    {
        View3D& v = app.view3D(ViewId::Main3D);
        v.pickCursor = false;
        v.gizmoPivot.reset();
        v.blockOrbit = false;
        dragging_ = painting_ = false;
        for (auto& b : app.doc().bridges)
            if (b.selectedConnector >= 0) {
                b.selectedConnector = -1;
                updateConnectorOverlays(app, b);
            }
        setAntagonistVisible(app, true);
        setOverlaysVisible(app, "connector:", true);
    }

    void update(DesignerApp& app) override
    {
        if (!ready_)
            prepare(app);
    }

    void drawPanel(DesignerApp& app) override
    {
        if (drawRestorationCard(app, false))
            if (RestorationDesign* r = app.doc().active())
                focusOn(app, *r);
        RestorationDesign* r = app.doc().active();
        if (id() != StepId::Connectors && (!r || !r->supported()))
            return;
        if (!ready_) {
            ImGui::Spacing();
            ui::wrappedMutedText("Preparing the design...");
            return;
        }
        if (r && r->supported() && !ensureBase(app, *r)) {
            ImGui::Spacing();
            ui::wrappedMutedText(r->dieError.empty() ? "Define the margin line and insertion axis first." : r->dieError.c_str());
            return;
        }
        switch (id()) {
        case StepId::ToothPlacement: drawPlacement(app, *r); break;
        case StepId::Freeforming: drawFreeforming(app, *r); break;
        default: drawConnectors(app); break;
        }
    }

    void onViewEvent(DesignerApp& app, ViewId view, const ViewEvents& ev) override
    {
        if (view != ViewId::Main3D || id() != StepId::Connectors)
            return;
        View3D& v = app.view3D(ViewId::Main3D);
        // Dragging the selected connector with the gizmo.
        if (auto [b, i] = selected(app); b && v.gizmoDelta) {
            if (!gizmoWasUsing_)
                pushConnectorUndo(*b);
            RestorationDesign* first = findRestoration(app, b->teeth.front());
            ScanObject* s = first ? scanById(app, first->prepScanId) : nullptr;
            if (s) {
                const glm::dvec3 moveLocal = transformVector(glm::inverse(s->transform), glm::dvec3((*v.gizmoDelta)[3]));
                glm::dvec3 x, y, z;
                crown::connectorFrame(b->connectors[static_cast<std::size_t>(i)], x, y, z);
                b->editFor(static_cast<std::size_t>(i)).offset += glm::dvec3(glm::dot(moveLocal, x), glm::dot(moveLocal, y), glm::dot(moveLocal, z));
                bridgeChanged(app, *b);
                app.markModified();
            }
        }
        gizmoWasUsing_ = v.gizmoDelta.has_value() || (gizmoWasUsing_ && ImGui::IsMouseDown(ImGuiMouseButton_Left));
        if (!ev.click || app.tasks().busy())
            return;
        if (!pickConnector(app, *ev.click))
            if (auto [b, i] = selected(app); b)
                selectConnector(app, *b, -1);
    }

    OverlayFn overlay(DesignerApp& app, ViewId view) override
    {
        if (view != ViewId::Main3D)
            return {};
        View3D& v = app.view3D(ViewId::Main3D);
        if (id() == StepId::Connectors) {
            // The gizmo sits on the selected connector (set before the view draws).
            v.gizmoPivot.reset();
            if (auto [b, i] = selected(app); b && moveInView_) {
                RestorationDesign* first = findRestoration(app, b->teeth.front());
                if (ScanObject* s = first ? scanById(app, first->prepScanId) : nullptr)
                    v.gizmoPivot = transformPoint(s->transform, b->connectors[static_cast<std::size_t>(i)].center);
            }
            return {};
        }
        if (id() == StepId::ToothPlacement)
            return [this, &app](ImDrawList* dl, const Projector& proj) { placementInteraction(app, dl, proj); };
        return [this, &app](ImDrawList* dl, const Projector& proj) { brushInteraction(app, dl, proj); };
    }

private:
    // ---- Tooth placement ---------------------------------------------------

    void drawPlacement(DesignerApp& app, RestorationDesign& r)
    {
        const ui::Palette& pal = ui::palette();
        const bool busy = app.tasks().busy();
        const float full = -FLT_MIN;
        bool changed = false;
        BridgeDesign* bridge = app.doc().bridgeOf(r.tooth);
        crown::CrownParameters before = r.params;

        ImGui::Spacing();
        ui::beginCard("##tooth");
        ui::subheading(r.isPontic() ? "Pontic" : "Tooth");
        if (drawLibraryPicker(app, r))
            changed = true;
        int kind = static_cast<int>(r.params.kind);
        ImGui::SetNextItemWidth(full);
        if (ImGui::BeginCombo("##kind", crown::toothTemplate(r.params.kind, r.params.upper).name)) {
            for (int k = 0; k <= static_cast<int>(crown::ToothKind::SecondMolar); ++k)
                if (ImGui::Selectable(crown::toothTemplate(static_cast<crown::ToothKind>(k), r.params.upper).name, k == kind)) {
                    pushUndo(r);
                    r.params.kind = static_cast<crown::ToothKind>(k);
                    changed = true;
                }
            ImGui::EndCombo();
        }
        if (!r.isPontic()) {
            int coping = r.params.coping ? 1 : 0;
            ImGui::SetNextItemWidth(full);
            if (ui::segmented("crowntype", {"Anatomic", "Coping"}, coping)) {
                r.params.coping = coping == 1;
                changed = true;
            }
        }
        ui::endCard();

        ImGui::Spacing();
        ui::beginCard("##place");
        ui::subheading("Position");
        ImGui::SetNextItemWidth(full);
        ui::segmented("placemode", {"Move", "Rotate", "Scale"}, placeMode_);
        ui::wrappedMutedText("Drag the tooth in the 3D view. Ctrl: rotate, Shift: scale, Ctrl+Shift: scale in the drag direction.");
        if (bridge)
            ImGui::Checkbox("Move all units together", &moveAll_);
        if (ui::button(bridge ? "Auto position bridge" : "Auto position", ImVec2(full, 0), !busy && !app.readOnly())) {
            pushUndo(r);
            startAutoDesign(app, r.tooth, distanceMap_);
        }
        if (ImGui::CollapsingHeader("Fine adjustment")) {
            auto slider = [&](const char* sid, double& value, double fallback, float lo, float hi, const char* fmt) {
                float v = static_cast<float>(value > 0 ? value : fallback);
                ImGui::SetNextItemWidth(full);
                if (ImGui::SliderFloat(sid, &v, lo, hi, fmt)) {
                    value = v;
                    changed = true;
                }
                if (ImGui::IsItemActivated())
                    pushUndo(r);
            };
            auto signedSlider = [&](const char* sid, double& value, float lo, float hi, const char* fmt) {
                float v = static_cast<float>(value);
                ImGui::SetNextItemWidth(full);
                if (ImGui::SliderFloat(sid, &v, lo, hi, fmt)) {
                    value = v;
                    changed = true;
                }
                if (ImGui::IsItemActivated())
                    pushUndo(r);
            };
            const auto libShape = crown::toothShape(r.params.library, r.params.kind, r.params.upper);
            slider("##h", r.params.crownHeight, libShape->height, 3.0f, 14.0f, "Height  %.2f mm");
            slider("##hm", r.params.halfMesial, libShape->halfMesial, 2.0f, 8.0f, "Mesial  %.2f mm");
            slider("##hd", r.params.halfDistal, libShape->halfDistal, 2.0f, 8.0f, "Distal  %.2f mm");
            slider("##hb", r.params.halfBuccal, libShape->halfBuccal, 2.0f, 8.0f, "Buccal  %.2f mm");
            slider("##hl", r.params.halfLingual, libShape->halfLingual, 2.0f, 8.0f, "Lingual  %.2f mm");
            signedSlider("##rot", r.params.rotationDeg, -45.0f, 45.0f, "Rotation  %.1f\xC2\xB0");
            signedSlider("##tm", r.params.tiltMesialDeg, -25.0f, 25.0f, "Tilt mesial  %.1f\xC2\xB0");
            signedSlider("##tb", r.params.tiltBuccalDeg, -25.0f, 25.0f, "Tilt buccal  %.1f\xC2\xB0");
            signedSlider("##sm", r.params.shiftMesial, -3.0f, 3.0f, "Shift mesial  %+.2f mm");
            signedSlider("##sb", r.params.shiftBuccal, -3.0f, 3.0f, "Shift buccal  %+.2f mm");
            signedSlider("##so", r.params.shiftOcclusal, -4.0f, 4.0f, "Shift occlusal  %+.2f mm");
            signedSlider("##cusp", r.params.cuspScale, 0.0f, 2.0f, "Cusp height  x%.2f");
            const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
            if (ui::button("Flip mesial/distal", ImVec2(half, 0))) {
                pushUndo(r);
                r.params.flipMesioDistal = !r.params.flipMesioDistal;
                changed = true;
            }
            ImGui::SameLine();
            if (ui::button("Flip buccal/lingual", ImVec2(half, 0))) {
                pushUndo(r);
                r.params.flipBuccoLingual = !r.params.flipBuccoLingual;
                changed = true;
            }
            if (r.isPontic()) {
                float ridge = static_cast<float>(r.params.ridgeOffset);
                ImGui::SetNextItemWidth(full);
                if (ImGui::SliderFloat("##ridge", &ridge, -0.5f, 1.0f, "Ridge contact  %.2f mm")) {
                    r.params.ridgeOffset = ridge;
                    r.invalidateDie(); // the basal surface is re-projected onto the ridge
                    changed = true;
                }
                ui::mutedText("Negative values press into the tissue (ovate pontic).");
            }
        }
        ui::endCard();

        ImGui::Spacing();
        ui::beginCard("##adapt");
        ui::subheading("Adapt to margin");
        if (r.adapted) {
            ImGui::TextColored(pal.success, "Attached to the %s", r.isPontic() ? "ridge" : "margin");
            ui::wrappedMutedText("Changing the position re-adapts the shell and keeps the free-forming.");
            if (ui::button("Detach (place freely again)", ImVec2(full, 0), !busy)) {
                pushUndo(r);
                std::vector<int> teeth = {r.tooth};
                if (bridge)
                    teeth = bridge->teeth;
                for (int t : teeth)
                    if (RestorationDesign* u = findRestoration(app, t)) {
                        u->adapted = false;
                        regenerate(app, *u, distanceMap_);
                    }
                app.markModified();
            }
        } else {
            ui::wrappedMutedText("When the tooth sits right, attach it to the margin. It becomes a thin shell for free-forming; the "
                                 "crown bottom adds the thickness when merging.");
            if (ui::primaryButton(bridge ? "Adapt bridge to margins" : "Adapt to margin", ImVec2(full, 0), !busy && r.placed)) {
                pushUndo(r);
                adaptToMargin(app, r.tooth, distanceMap_);
            }
        }
        ui::endCard();
        if (scanById(app, r.antagonistScanId)) {
            ImGui::Spacing();
            bool vis = antagonistVisible(app);
            if (ImGui::Checkbox("Show antagonist (transparent)", &vis))
                setAntagonistVisible(app, vis, 0.45f);
        }
        drawUndo(app, r);
        if (changed && !busy) {
            if (r.params.kind != before.kind || r.params.coping != before.coping || r.params.library != before.library)
                r.displacement.clear();
            r.placed = true;
            regenerate(app, r, distanceMap_);
            app.markModified();
        }
    }

    // The restorations moved by a drag: the hovered one, or its whole bridge with "move all".
    std::vector<RestorationDesign*> dragTargets(DesignerApp& app, int tooth)
    {
        std::vector<RestorationDesign*> out;
        BridgeDesign* b = app.doc().bridgeOf(tooth);
        if (b && moveAll_) {
            for (int t : b->teeth)
                if (RestorationDesign* u = findRestoration(app, t))
                    out.push_back(u);
        } else if (RestorationDesign* u = findRestoration(app, tooth)) {
            out.push_back(u);
        }
        return out;
    }

    const Mesh* displayedMesh(const RestorationDesign& r) const
    {
        if (r.adapted && r.crown)
            return &r.crown->mesh;
        if (r.placed && r.libraryTooth)
            return r.libraryTooth.get();
        return nullptr;
    }

    // Move (drag), rotate (Ctrl), scale (Shift) or scale in the drag direction (Ctrl+Shift) the tooth.
    void placementInteraction(DesignerApp& app, ImDrawList* dl, const Projector& proj)
    {
        View3D& view = app.view3D(ViewId::Main3D);
        const ImGuiIO& io = ImGui::GetIO();
        const ImVec2 mouse = io.MousePos;
        const bool inView = proj.contains(mouse) && ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
        if (dragging_) {
            if (!ImGui::IsMouseDown(ImGuiMouseButton_Left) || app.tasks().busy()) {
                dragging_ = false;
                view.blockOrbit = false;
                for (int t : dragTeeth_)
                    if (RestorationDesign* u = findRestoration(app, t))
                        updateCrownOverlay(app, *u, distanceMap_);
                app.markModified();
                return;
            }
            const ImVec2 d(mouse.x - lastMouse_.x, mouse.y - lastMouse_.y);
            lastMouse_ = mouse;
            if (d.x != 0.0f || d.y != 0.0f)
                for (int t : dragTeeth_)
                    if (RestorationDesign* u = findRestoration(app, t))
                        applyDrag(app, *u, d, proj, io.KeyCtrl, io.KeyShift);
            return;
        }
        if (!inView || app.tasks().busy())
            return;
        auto ray = proj.rayAt(mouse);
        if (!ray)
            return;
        // The tooth under the mouse.
        int hovered = 0;
        double best = 1e30;
        for (const auto& r : app.doc().restorations) {
            const Mesh* m = displayedMesh(r);
            ScanObject* s = m ? scanById(app, r.prepScanId) : nullptr;
            if (!s)
                continue;
            if (auto hit = raycastMesh(*m, s->transform, *ray); hit && hit->distance < best) {
                best = hit->distance;
                hovered = r.tooth;
            }
        }
        if (!hovered)
            return;
        const bool ctrl = io.KeyCtrl, shift = io.KeyShift;
        const int mode = ctrl && shift ? 3 : ctrl ? 1 : shift ? 2 : placeMode_;
        ImGui::SetMouseCursor(mode == 0 ? ImGuiMouseCursor_ResizeAll : mode == 1 ? ImGuiMouseCursor_Hand : ImGuiMouseCursor_ResizeNWSE);
        const char* what[] = {"Drag to move", "Drag to rotate (sideways) and tilt (up/down)", "Drag up/down to scale", "Drag to scale in that direction"};
        const float sc = ImGui::GetStyle().FontScaleDpi;
        dl->AddText(ImVec2(mouse.x + 16 * sc, mouse.y + 8 * sc), IM_COL32(240, 242, 245, 230), what[mode]);
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            dragging_ = true;
            view.blockOrbit = true;
            lastMouse_ = mouse;
            dragTeeth_.clear();
            for (RestorationDesign* u : dragTargets(app, hovered)) {
                pushUndo(*u);
                dragTeeth_.push_back(u->tooth);
            }
            if (RestorationDesign* u = findRestoration(app, hovered))
                app.doc().activeRestoration = static_cast<int>(u - app.doc().restorations.data());
        }
    }

    void applyDrag(DesignerApp& app, RestorationDesign& r, const ImVec2& d, const Projector& proj, bool ctrl, bool shift)
    {
        ScanObject* s = scanById(app, r.prepScanId);
        if (!s || !r.base)
            return;
        crown::CrownParameters& p = r.params;
        const gfx::Camera& cam = app.view3D(ViewId::Main3D).camera;
        const double wpp = cam.worldPerPixel(proj.max.y - proj.min.y);
        const glm::dmat4 inv = glm::inverse(s->transform);
        const glm::dvec3 right = glm::normalize(transformVector(inv, cam.right())), up = glm::normalize(transformVector(inv, cam.up()));
        crown::CrownParameters flat = p;
        flat.tiltMesialDeg = flat.tiltBuccalDeg = 0.0;
        const crown::CrownAxes ax = crown::crownAxes(r.base->frame, flat);
        const int mode = ctrl && shift ? 3 : ctrl ? 1 : shift ? 2 : placeMode_;
        const auto shape = crown::toothShape(p.library, p.kind, p.upper);
        auto materialise = [&] {
            if (p.crownHeight <= 0)
                p.crownHeight = shape->height;
            if (p.halfMesial <= 0)
                p.halfMesial = shape->halfMesial;
            if (p.halfDistal <= 0)
                p.halfDistal = shape->halfDistal;
            if (p.halfBuccal <= 0)
                p.halfBuccal = shape->halfBuccal;
            if (p.halfLingual <= 0)
                p.halfLingual = shape->halfLingual;
        };
        if (mode == 0) {
            const glm::dvec3 move = (right * static_cast<double>(d.x) - up * static_cast<double>(d.y)) * wpp;
            p.shiftMesial += glm::dot(move, ax.M);
            p.shiftBuccal += glm::dot(move, ax.B);
            p.shiftOcclusal += glm::dot(move, ax.A);
        } else if (mode == 1) {
            p.rotationDeg = std::clamp(p.rotationDeg + d.x * 0.4, -90.0, 90.0);
            // Up/down tilts the tooth axis towards the viewer's up direction.
            glm::dvec3 tiltDir = up - ax.A * glm::dot(up, ax.A);
            if (glm::length(tiltDir) > 1e-6) {
                tiltDir = glm::normalize(tiltDir);
                const double deg = -d.y * 0.25;
                p.tiltMesialDeg = std::clamp(p.tiltMesialDeg + deg * glm::dot(tiltDir, ax.M), -30.0, 30.0);
                p.tiltBuccalDeg = std::clamp(p.tiltBuccalDeg + deg * glm::dot(tiltDir, ax.B), -30.0, 30.0);
            }
        } else if (mode == 2) {
            materialise();
            const double f = std::exp(-d.y * 0.004);
            p.crownHeight *= f;
            p.halfMesial *= f;
            p.halfDistal *= f;
            p.halfBuccal *= f;
            p.halfLingual *= f;
        } else {
            materialise();
            const glm::dvec3 dir = right * static_cast<double>(d.x) - up * static_cast<double>(d.y);
            const double len = glm::length(dir);
            if (len < 1e-9)
                return;
            // Dragging away from the tooth centre enlarges it along the drag direction.
            const glm::dvec3 u = dir / len;
            const double sign = d.x - d.y >= 0.0f ? 1.0 : -1.0; // right / up enlarges
            const double f = std::exp(sign * len * 0.004);
            const double wm = std::abs(glm::dot(u, ax.M)), wb = std::abs(glm::dot(u, ax.B)), wa = std::abs(glm::dot(u, ax.A));
            p.halfMesial *= std::pow(f, wm);
            p.halfDistal *= std::pow(f, wm);
            p.halfBuccal *= std::pow(f, wb);
            p.halfLingual *= std::pow(f, wb);
            p.crownHeight *= std::pow(f, wa);
        }
        p.crownHeight = std::clamp(p.crownHeight, 2.0, 16.0);
        for (double* h : {&p.halfMesial, &p.halfDistal, &p.halfBuccal, &p.halfLingual})
            if (*h > 0)
                *h = std::clamp(*h, 1.5, 9.0);
        r.placed = true;
        regenerate(app, r, false);
    }

    // ---- Free-forming --------------------------------------------------------

    void loadCustomTools()
    {
        tools_ = crown::builtinBrushTools();
        builtinCount_ = tools_.size();
        for (auto& t : crown::loadBrushTools(brushFolder()))
            tools_.push_back(std::move(t));
        if (tool_ >= static_cast<int>(tools_.size()))
            tool_ = 0;
    }

    void drawFreeforming(DesignerApp& app, RestorationDesign& r)
    {
        const ui::Palette& pal = ui::palette();
        const bool busy = app.tasks().busy();
        const float full = -FLT_MIN;
        if (!r.adapted || !r.crown) {
            ImGui::Spacing();
            ui::wrappedMutedText("Adapt the tooth to the margin first (tooth placement).");
            return;
        }
        ImGui::Spacing();
        ui::beginCard("##knife");
        ui::subheading("Wax knife");
        int mode = brushOn_ ? brushMode_ + 1 : 0;
        ImGui::SetNextItemWidth(full);
        if (ui::segmented("brush", {"Off", "Add / Remove", "Smooth / Flatten"}, mode)) {
            brushOn_ = mode != 0;
            if (brushOn_)
                brushMode_ = mode - 1;
            app.view3D(ViewId::Main3D).pickCursor = brushOn_;
        }
        ui::mutedText(brushMode_ == 0 ? "Hold to add, Shift: remove, Ctrl+Shift: smooth." : "Hold to smooth, Shift: flatten.");
        ImGui::SetNextItemWidth(full);
        if (ImGui::BeginCombo("##tool", tools_[static_cast<std::size_t>(tool_)].name.c_str())) {
            for (std::size_t i = 0; i < tools_.size(); ++i) {
                if (i == builtinCount_)
                    ImGui::Separator();
                if (ImGui::Selectable(std::format("{}##t{}", tools_[i].name, i).c_str(), static_cast<int>(i) == tool_))
                    tool_ = static_cast<int>(i);
            }
            ImGui::EndCombo();
        }
        ImGui::SetNextItemWidth(full);
        ImGui::SliderFloat("##br", &brushRadius_, 0.3f, 5.0f, "Size  %.1f mm");
        ImGui::SetNextItemWidth(full);
        ImGui::SliderFloat("##bs", &brushStrength_, 0.02f, 1.0f, "Strength  %.2f mm/s");
        crown::BrushTool& tool = tools_[static_cast<std::size_t>(tool_)];
        if (tool.kind == crown::BrushKind::Texture) {
            float ts = static_cast<float>(tool.textureScale), tc = static_cast<float>(tool.textureContrast);
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat("##tsc", &ts, 0.05f, 1.5f, "Texture size  %.2f mm"))
                tool.textureScale = ts;
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat("##tct", &tc, 0.2f, 3.0f, "Texture contrast  %.1f"))
                tool.textureContrast = tc;
        }
        if (ImGui::CollapsingHeader("Custom tools"))
            drawToolEditor();
        ui::endCard();

        ImGui::Spacing();
        ui::beginCard("##adaptcontacts");
        ui::subheading("Adapt");
        auto target = [&](const char* sid, double& value, const char* fmt) {
            float v = static_cast<float>(value);
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat(sid, &v, -0.2f, 0.4f, fmt))
                value = v;
        };
        target("##ot", r.params.occlusalTarget, "Occlusal distance  %.2f mm");
        target("##pt", r.params.proximalTarget, "Approximal distance  %.2f mm");
        ui::mutedText("Negative values overlap (tight contact).");
        const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (ui::button("Occlusion", ImVec2(half, 0), !busy && r.contacts->antagonist != nullptr))
            adapt(app, r, false, true, 0.6);
        ImGui::SameLine();
        if (ui::button("Approximal", ImVec2(half, 0), !busy && r.contacts->neighbors != nullptr))
            adapt(app, r, true, false, 0.6);
        if (ui::button("Cut all intersections", ImVec2(full, 0), !busy))
            adapt(app, r, true, r.contacts->antagonist != nullptr, 0.0);
        if (ImGui::Checkbox("Show distance map", &distanceMap_))
            updateCrownOverlay(app, r, distanceMap_);
        if (scanById(app, r.antagonistScanId)) {
            bool vis = antagonistVisible(app);
            if (ImGui::Checkbox("Show antagonist (transparent)", &vis))
                setAntagonistVisible(app, vis, 0.45f);
        }
        if (distanceMap_) {
            const auto legend = [&](glm::vec3 c, const char* t) {
                ImGui::ColorButton(t, ImVec4(c.r, c.g, c.b, 1.0f), ImGuiColorEditFlags_NoTooltip, ImVec2(ImGui::GetFontSize(), ImGui::GetFontSize()));
                ImGui::SameLine();
                ui::mutedText("%s", t);
            };
            legend(contactColor(-0.1f), "< -0.05 mm");
            ImGui::SameLine();
            legend(contactColor(0.05f), "0 - 0.1");
            ImGui::SameLine();
            legend(contactColor(0.2f), "0.1 - 0.3");
        }
        ui::endCard();
        (void)pal;
        drawUndo(app, r);
    }

    // Custom tool editor: the cross-section of the tool (drag points, double-click adds one,
    // right-click removes one), the falloff, and saving / importing / exporting tools.
    void drawToolEditor()
    {
        crown::BrushTool& t = editTool_;
        const float s = ImGui::GetFontSize();
        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::InputTextWithHint("##tname", "Tool name", &t.name);
        // Profile canvas: x = distance from the tool centre (0..1), y = material (0..1).
        const ImVec2 size(ImGui::GetContentRegionAvail().x, s * 7.0f);
        const ImVec2 o = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("##profile", size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        const bool hovered = ImGui::IsItemHovered();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const ui::Palette& pal = ui::palette();
        const float pad = s * 0.5f;
        auto toScreen = [&](double x, double y) { return ImVec2(o.x + pad + static_cast<float>(x) * (size.x - 2 * pad), o.y + size.y - pad - static_cast<float>(y) * (size.y - 2 * pad)); };
        auto fromScreen = [&](ImVec2 p) {
            return glm::dvec2(std::clamp((p.x - o.x - pad) / (size.x - 2 * pad), 0.0f, 1.0f), std::clamp((o.y + size.y - pad - p.y) / (size.y - 2 * pad), 0.0f, 1.0f));
        };
        dl->AddRectFilled(o, ImVec2(o.x + size.x, o.y + size.y), ImGui::GetColorU32(pal.surfaceAlt), 4.0f);
        // Effective tool (profile x falloff) as a filled curve, and the editable profile on top.
        for (int i = 0; i < 60; ++i) {
            const double x0 = i / 60.0, x1 = (i + 1) / 60.0;
            const ImVec2 a = toScreen(x0, t.weight(x0, glm::vec3(0))), b = toScreen(x1, t.weight(x1, glm::vec3(0)));
            dl->AddQuadFilled(toScreen(x0, 0), a, b, toScreen(x1, 0), IM_COL32(120, 160, 230, 90));
        }
        for (std::size_t i = 1; i < t.profile.size(); ++i)
            dl->AddLine(toScreen(t.profile[i - 1].x, t.profile[i - 1].y), toScreen(t.profile[i].x, t.profile[i].y), ImGui::GetColorU32(pal.accent), 2.0f);
        const ImVec2 fs = toScreen(t.falloffStart, 0.0);
        dl->AddLine(ImVec2(fs.x, o.y + pad), ImVec2(fs.x, o.y + size.y - pad), IM_COL32(230, 160, 60, 160), 1.0f);
        int hoverPt = -1;
        for (std::size_t i = 0; i < t.profile.size(); ++i) {
            const ImVec2 q = toScreen(t.profile[i].x, t.profile[i].y);
            const bool h = hovered && std::hypot(q.x - ImGui::GetIO().MousePos.x, q.y - ImGui::GetIO().MousePos.y) < s * 0.5f;
            if (h)
                hoverPt = static_cast<int>(i);
            dl->AddCircleFilled(q, s * (h || static_cast<int>(i) == dragPt_ ? 0.32f : 0.24f), ImGui::GetColorU32(pal.accent));
        }
        if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            dragPt_ = hoverPt;
        if (dragPt_ >= 0 && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            glm::dvec2 q = fromScreen(ImGui::GetIO().MousePos);
            const auto i = static_cast<std::size_t>(dragPt_);
            if (i == 0)
                q.x = 0.0; // the centre stays at the centre
            if (i > 0)
                q.x = std::max(q.x, t.profile[i - 1].x + 0.01);
            if (i + 1 < t.profile.size())
                q.x = std::min(q.x, t.profile[i + 1].x - 0.01);
            t.profile[i] = q;
        } else {
            dragPt_ = -1;
        }
        if (hovered && hoverPt < 0 && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
            const glm::dvec2 q = fromScreen(ImGui::GetIO().MousePos);
            t.profile.push_back(q);
            std::sort(t.profile.begin(), t.profile.end(), [](const glm::dvec2& a, const glm::dvec2& b) { return a.x < b.x; });
        }
        if (hovered && hoverPt > 0 && t.profile.size() > 2 && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
            t.profile.erase(t.profile.begin() + hoverPt);
        ui::mutedText("Drag points; double-click adds, right-click removes.");
        float f0 = static_cast<float>(t.falloffStart), fp = static_cast<float>(t.falloffPower);
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::SliderFloat("##f0", &f0, 0.0f, 1.0f, "Falloff starts at  %.2f"))
            t.falloffStart = f0;
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::SliderFloat("##fp", &fp, 0.2f, 4.0f, "Falloff curve  %.2f"))
            t.falloffPower = fp;
        t.kind = crown::BrushKind::Custom;
        const float third = (ImGui::GetContentRegionAvail().x - 2 * ImGui::GetStyle().ItemSpacing.x) / 3.0f;
        if (ui::button("Save tool", ImVec2(third, 0), !t.name.empty())) {
            try {
                crown::saveBrushTool(brushFolder() / (brushSlug(t.name) + ".json"), t);
                loadCustomTools();
                for (std::size_t i = 0; i < tools_.size(); ++i)
                    if (tools_[i].name == t.name && i >= builtinCount_)
                        tool_ = static_cast<int>(i);
                ui::toast(ui::ToastKind::Success, "Tool \"" + t.name + "\" saved.");
            } catch (const std::exception& e) {
                ui::showError("Save tool", e.what());
            }
        }
        ImGui::SameLine();
        if (ui::button("Import...", ImVec2(third, 0)))
            if (auto f = ui::dialogs::openFile({{"OcclusaCAD tools", "json"}})) {
                try {
                    const crown::BrushTool in = crown::loadBrushTool(*f);
                    crown::saveBrushTool(brushFolder() / (brushSlug(in.name) + ".json"), in);
                    loadCustomTools();
                    editTool_ = in;
                    ui::toast(ui::ToastKind::Success, "Tool \"" + in.name + "\" imported.");
                } catch (const std::exception& e) {
                    ui::showError("Import tool", e.what());
                }
            }
        ImGui::SameLine();
        if (ui::button("Export...", ImVec2(third, 0), !t.name.empty()))
            if (auto f = ui::dialogs::saveFile({{"OcclusaCAD tools", "json"}}, brushSlug(t.name) + ".json")) {
                try {
                    crown::saveBrushTool(*f, t);
                } catch (const std::exception& e) {
                    ui::showError("Export tool", e.what());
                }
            }
        if (tool_ >= static_cast<int>(builtinCount_) && ui::button("Edit selected tool", ImVec2(-FLT_MIN, 0)))
            editTool_ = tools_[static_cast<std::size_t>(tool_)];
    }

    // Hold-to-sculpt: while the mouse is held over the crown, the tool works continuously.
    void brushInteraction(DesignerApp& app, ImDrawList* dl, const Projector& proj)
    {
        View3D& view = app.view3D(ViewId::Main3D);
        const ImGuiIO& io = ImGui::GetIO();
        const ImVec2 mouse = io.MousePos;
        RestorationDesign* r = app.doc().active();
        if (!brushOn_ || !r || !r->adapted || !r->crown || app.tasks().busy()) {
            if (painting_)
                endStroke(app);
            return;
        }
        const bool inView = proj.contains(mouse) && ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
        const float sc = ImGui::GetStyle().FontScaleDpi;
        ScanObject* s = scanById(app, r->prepScanId);
        if (!s)
            return;
        std::optional<glm::vec3> local;
        if (auto ray = proj.rayAt(mouse))
            if (auto hit = raycastMesh(r->crown->mesh, s->transform, *ray))
                local = glm::vec3(transformPoint(glm::inverse(s->transform), hit->point));
        if (local && inView) {
            // Brush outline: a circle of the brush size around the surface point.
            const glm::dvec3 w = transformPoint(s->transform, glm::dvec3(*local));
            if (auto c = proj(w)) {
                const glm::dvec3 rightW = app.view3D(ViewId::Main3D).camera.right();
                if (auto e = proj(w + rightW * static_cast<double>(brushRadius_)))
                    dl->AddCircle(*c, std::hypot(e->x - c->x, e->y - c->y), IM_COL32(255, 150, 40, 220), 40, 1.5f * sc);
            }
        }
        if (!painting_ && inView && local && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            painting_ = true;
            view.blockOrbit = true;
            pushUndo(*r);
        }
        if (painting_) {
            if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                endStroke(app);
                return;
            }
            if (!local)
                return;
            crown::BrushMode mode;
            if (brushMode_ == 0)
                mode = io.KeyCtrl && io.KeyShift ? crown::BrushMode::Smooth : io.KeyShift ? crown::BrushMode::Remove : crown::BrushMode::Add;
            else
                mode = io.KeyShift ? crown::BrushMode::Flatten : crown::BrushMode::Smooth;
            const float dt = std::clamp(io.DeltaTime, 0.004f, 0.05f);
            const float amount = mode == crown::BrushMode::Smooth || mode == crown::BrushMode::Flatten ? brushStrength_ * 0.5f : brushStrength_ * dt;
            crown::applyBrushTool(*r->crown, r->displacement, *local, brushRadius_, amount, mode, tools_[static_cast<std::size_t>(tool_)]);
            regenerateStroke(app, *r);
        }
    }

    // During a stroke only the shell is rebuilt (no distance map, no bridge update).
    void regenerateStroke(DesignerApp& app, RestorationDesign& r)
    {
        r.crown = std::make_shared<crown::CrownMesh>(crown::buildCrown(*r.base, r.effectiveParams(), r.displacement, true));
        r.merged.reset();
        updateCrownOverlay(app, r, false);
    }

    void endStroke(DesignerApp& app)
    {
        painting_ = false;
        app.view3D(ViewId::Main3D).blockOrbit = false;
        if (RestorationDesign* r = app.doc().active()) {
            regenerate(app, *r, distanceMap_);
            app.markModified();
        }
    }

    void adapt(DesignerApp& app, RestorationDesign& r, bool proximal, bool occlusal, double reach)
    {
        if (!ensureBase(app, r))
            return;
        pushUndo(r);
        auto base = r.base;
        auto contacts = r.contacts;
        const crown::CrownParameters p = r.effectiveParams();
        std::vector<float> disp = r.displacement;
        const int tooth = r.tooth;
        const bool map = distanceMap_;
        app.tasks().start("Adapting", [&app, base, contacts, p, disp, tooth, proximal, occlusal, reach, map](const ProgressFn& progress) mutable
                          -> ui::TaskRunner::Continuation {
            reportProgress(progress, 0.2f, "Adapting the crown surface");
            crown::adaptContacts(*base, p, *contacts, disp, proximal, occlusal, reach, true);
            return [&app, disp, tooth, map] {
                if (RestorationDesign* r = findRestoration(app, tooth)) {
                    r->displacement = disp;
                    regenerate(app, *r, map);
                    app.markModified();
                }
            };
        });
    }

    // ---- Connectors ----------------------------------------------------------

    void drawConnectors(DesignerApp& app)
    {
        const bool busy = app.tasks().busy();
        if (app.doc().bridges.empty()) {
            ImGui::Spacing();
            ui::wrappedMutedText("This case has no bridges: there are no connectors to set. Continue to merge.");
            return;
        }
        for (auto& b : app.doc().bridges) {
            if (b.connectors.empty())
                bridgeChanged(app, b);
            if (b.connectors.empty()) {
                ImGui::Spacing();
                ui::wrappedMutedText(std::format("Bridge {}: adapt all units to the margins first.", bridgeName(app, b)).c_str());
                continue;
            }
            drawBridgeCard(app, b, busy);
        }
    }

    // ---- Shared ----------------------------------------------------------------

    void drawUndo(DesignerApp& app, RestorationDesign& r)
    {
        const bool busy = app.tasks().busy();
        ImGui::Spacing();
        const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (ui::button("Undo", ImVec2(half, 0), !undo_.empty() && !busy)) {
            const auto& u = undo_.back();
            if (RestorationDesign* t = findRestoration(app, u.tooth)) {
                t->params = u.params;
                t->displacement = u.displacement;
                t->placed = u.placed;
                t->adapted = u.adapted;
                t->invalidateShape();
                regenerate(app, *t, distanceMap_);
            }
            undo_.pop_back();
            app.markModified();
        }
        ImGui::SameLine();
        if (ui::button("Reset free-forming", ImVec2(half, 0), !r.displacement.empty() && !busy)) {
            pushUndo(r);
            r.displacement.clear();
            regenerate(app, r, distanceMap_);
            app.markModified();
        }
    }

    // Tooth library choice and import. Returns true when the library changed.
    bool drawLibraryPicker(DesignerApp& app, RestorationDesign& r)
    {
        auto& registry = crown::ToothLibraryRegistry::instance();
        const auto libs = registry.libraries();
        const std::string current = r.params.library.empty() ? crown::ToothLibraryRegistry::kDefaultLibrary : r.params.library;
        const auto info = registry.find(current);
        bool changed = false;
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::BeginCombo("##library", info ? info->name.c_str() : current.c_str())) {
            for (const auto& lib : libs) {
                const std::string label = lib.builtIn ? lib.name : lib.name + "  (" + lib.license + ")";
                if (ImGui::Selectable(std::format("{}##{}", label, lib.id).c_str(), lib.id == current)) {
                    if (lib.id != current) {
                        pushUndo(r);
                        r.params.library = lib.id;
                        r.params.crownHeight = 0.0;
                        r.params.halfMesial = r.params.halfDistal = r.params.halfBuccal = r.params.halfLingual = 0.0;
                        changed = true;
                    }
                }
                if (ImGui::IsItemHovered() && !lib.description.empty())
                    ImGui::SetTooltip("%s", lib.description.c_str());
            }
            ImGui::EndCombo();
        }
        if (info && !info->builtIn) {
            bool has = false;
            for (int t : info->teeth)
                has |= dental::isUpper(t) == r.params.upper && t % 10 == crown::fdiPosition(r.params.kind);
            if (!has)
                ui::mutedText("No %s in this library: the default is used.", std::string(crown::toString(r.params.kind)).c_str());
            if (!info->author.empty() || !info->license.empty())
                ui::mutedText("%s%s%s", info->author.c_str(), info->author.empty() || info->license.empty() ? "" : "  -  ", info->license.c_str());
        }
        if (ui::button("Import library...", ImVec2(-FLT_MIN, 0), !app.headless() && !app.tasks().busy()))
            if (auto folder = ui::dialogs::pickFolder())
                if (auto id = importLibrary(app, *folder)) {
                    pushUndo(r);
                    r.params.library = *id;
                    changed = true;
                }
        return changed;
    }

    // Copy a library folder into the lab's shared libraries (or the user's own without a data folder).
    // A folder without library.json is accepted when its STL files are named by FDI number and
    // already in the tooth frame.
    std::optional<std::string> importLibrary(DesignerApp& app, const std::filesystem::path& source)
    {
        try {
            crown::ToothLibraryManifest m = std::filesystem::exists(source / "library.json")
                                                ? crown::readToothLibraryManifest(source)
                                                : crown::manifestFromStlFolder(source, platform::pathToUtf8(source.filename()));
            const std::filesystem::path root = app.config().dataRoot.empty() ? platform::configDir() / "libraries" : app.config().librariesRoot();
            const std::filesystem::path dest = root / platform::pathFromUtf8(m.id);
            std::filesystem::create_directories(dest);
            for (const auto& t : m.teeth)
                std::filesystem::copy_file(source / platform::pathFromUtf8(t.file), dest / platform::pathFromUtf8(t.file),
                                           std::filesystem::copy_options::overwrite_existing);
            crown::writeToothLibraryManifest(dest, m);
            // Check every tooth before offering the library.
            for (const auto& t : m.teeth)
                crown::sampleToothShape(crown::loadLibraryTooth(dest, t), t.cervicalZ);
            const std::string id = crown::ToothLibraryRegistry::instance().addFolder(dest);
            ui::toast(ui::ToastKind::Success, std::format("Tooth library \"{}\" imported ({} teeth)", m.name, m.teeth.size()));
            log::info("Tooth library {} imported to {}", m.name, platform::pathToUtf8(dest));
            return id;
        } catch (const std::exception& e) {
            ui::showError("Import tooth library", e.what());
            return std::nullopt;
        }
    }

    void drawBridgeCard(DesignerApp& app, BridgeDesign& b, bool busy)
    {
        const ui::Palette& pal = ui::palette();
        const float full = -FLT_MIN;
        ImGui::Spacing();
        ui::beginCard("##bridge");
        ui::subheading(std::format("Bridge {}", bridgeName(app, b)).c_str());
        int pontics = 0;
        for (int t : b.teeth)
            if (RestorationDesign* u = findRestoration(app, t); u && u->isPontic())
                ++pontics;
        ui::mutedText("%zu units: %zu abutments, %d pontic%s", b.teeth.size(), b.teeth.size() - static_cast<std::size_t>(pontics), pontics,
                      pontics == 1 ? "" : "s");
        bool changed = false;
        float area = static_cast<float>(b.connectorArea);
        ImGui::SetNextItemWidth(full);
        if (ImGui::SliderFloat("##carea", &area, 4.0f, 20.0f, "Connector area  %.1f mm2")) {
            b.connectorArea = area;
            changed = true;
        }
        float ratio = static_cast<float>(b.connectorHeightRatio);
        ImGui::SetNextItemWidth(full);
        if (ImGui::SliderFloat("##cratio", &ratio, 0.8f, 2.0f, "Height / width  %.2f")) {
            b.connectorHeightRatio = ratio;
            changed = true;
        }
        float emb = static_cast<float>(b.embrasure);
        ImGui::SetNextItemWidth(full);
        if (ImGui::SliderFloat("##cemb", &emb, 0.0f, 3.0f, "Embrasure clearance  %.1f mm")) {
            b.embrasure = emb;
            changed = true;
        }
        ui::mutedText("Zirconia: about 9 mm2 posterior, 7 mm2 anterior.");
        if (changed) {
            bridgeChanged(app, b);
            app.markModified();
        }
        for (const auto& w : b.warnings)
            if (!w.empty()) {
                ImGui::PushStyleColor(ImGuiCol_Text, pal.warning);
                ImGui::TextWrapped("%s", w.c_str());
                ImGui::PopStyleColor();
            }
        drawConnectorEditor(app, b);
        ui::endCard();
    }

    // List of connectors plus position / size controls for the selected one.
    void drawConnectorEditor(DesignerApp& app, BridgeDesign& b)
    {
        const ui::Palette& pal = ui::palette();
        const float full = -FLT_MIN;
        if (b.connectors.empty())
            return;
        ImGui::Spacing();
        ImGui::TextUnformatted("Connectors");
        for (std::size_t i = 0; i < b.connectors.size(); ++i) {
            const auto& c = b.connectors[i];
            const bool warn = i < b.warnings.size() && !b.warnings[i].empty();
            std::string label = std::format("{}-{}   {:.1f} mm2,  {:.1f} x {:.1f} mm", app.toothText(b.teeth[i]), app.toothText(b.teeth[i + 1]), c.area, c.height(), c.width());
            if (!b.editAt(i).isDefault())
                label += "  (edited)";
            if (warn)
                ImGui::PushStyleColor(ImGuiCol_Text, pal.warning);
            if (ImGui::Selectable(std::format("{}##con{}", label, i).c_str(), b.selectedConnector == static_cast<int>(i)))
                selectConnector(app, b, b.selectedConnector == static_cast<int>(i) ? -1 : static_cast<int>(i));
            if (warn)
                ImGui::PopStyleColor();
        }
        if (b.selectedConnector < 0)
            ui::mutedText("Click a connector here or in the 3D view to edit it.");
        if (b.selectedConnector < 0 || b.selectedConnector >= static_cast<int>(b.connectors.size()))
            return;
        const auto i = static_cast<std::size_t>(b.selectedConnector);
        crown::ConnectorEdit& e = b.editFor(i);
        const crown::ConnectorSpec& spec = b.connectors[i];
        bool changed = false;
        auto track = [&](bool edited) {
            if (ImGui::IsItemActivated())
                pushConnectorUndo(b);
            changed |= edited;
        };
        float area = static_cast<float>(spec.area);
        ImGui::SetNextItemWidth(full);
        track(ImGui::SliderFloat("##earea", &area, 4.0f, 25.0f, "Area  %.1f mm2"));
        if (std::abs(area - static_cast<float>(spec.area)) > 1e-4f)
            e.area = area;
        float ratio = static_cast<float>(spec.heightRatio);
        ImGui::SetNextItemWidth(full);
        track(ImGui::SliderFloat("##eratio", &ratio, 0.6f, 2.5f, "Height / width  %.2f"));
        if (std::abs(ratio - static_cast<float>(spec.heightRatio)) > 1e-4f)
            e.heightRatio = ratio;
        float length = static_cast<float>(spec.length);
        ImGui::SetNextItemWidth(full);
        track(ImGui::SliderFloat("##elen", &length, 2.0f, 12.0f, "Length  %.1f mm"));
        if (std::abs(length - static_cast<float>(spec.length)) > 1e-4f)
            e.length = length;
        float off[3] = {static_cast<float>(e.offset.z), static_cast<float>(e.offset.y), static_cast<float>(e.offset.x)};
        ImGui::SetNextItemWidth(full);
        track(ImGui::SliderFloat("##eoz", &off[0], -3.0f, 3.0f, "Occlusal / gingival  %+.2f mm"));
        ImGui::SetNextItemWidth(full);
        track(ImGui::SliderFloat("##eoy", &off[1], -3.0f, 3.0f, "Bucco-lingual  %+.2f mm"));
        ImGui::SetNextItemWidth(full);
        track(ImGui::SliderFloat("##eox", &off[2], -3.0f, 3.0f, "Mesio-distal  %+.2f mm"));
        e.offset = glm::dvec3(off[2], off[1], off[0]);
        ImGui::Checkbox("Move with the gizmo in the 3D view", &moveInView_);
        const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (ui::button("Undo", ImVec2(half, 0), !connectorUndo_.empty())) {
            const auto& [label, edits] = connectorUndo_.back();
            if (label == b.label())
                b.edits = edits;
            connectorUndo_.pop_back();
            changed = true;
        }
        ImGui::SameLine();
        if (ui::button("Reset connector", ImVec2(half, 0), !e.isDefault())) {
            pushConnectorUndo(b);
            e = {};
            changed = true;
        }
        if (changed) {
            bridgeChanged(app, b);
            app.markModified();
        }
    }

    void selectConnector(DesignerApp& app, BridgeDesign& b, int index)
    {
        for (auto& other : app.doc().bridges)
            if (&other != &b && other.selectedConnector >= 0) {
                other.selectedConnector = -1;
                updateConnectorOverlays(app, other);
            }
        b.selectedConnector = index;
        updateConnectorOverlays(app, b);
    }

    void pushConnectorUndo(const BridgeDesign& b)
    {
        connectorUndo_.emplace_back(b.label(), b.edits);
        if (connectorUndo_.size() > 50)
            connectorUndo_.erase(connectorUndo_.begin());
    }

    // The selected connector (if any) and its bridge.
    std::pair<BridgeDesign*, int> selected(DesignerApp& app)
    {
        for (auto& b : app.doc().bridges)
            if (b.selectedConnector >= 0 && b.selectedConnector < static_cast<int>(b.connectors.size()))
                return {&b, b.selectedConnector};
        return {nullptr, -1};
    }

    // Click on a connector in the 3D view: select it.
    bool pickConnector(DesignerApp& app, const Ray& ray)
    {
        double bestT = 1e30;
        BridgeDesign* bestB = nullptr;
        int bestI = -1;
        for (auto& b : app.doc().bridges) {
            RestorationDesign* first = findRestoration(app, b.teeth.front());
            ScanObject* s = first ? scanById(app, first->prepScanId) : nullptr;
            if (!s)
                continue;
            for (std::size_t i = 0; i < b.connectors.size(); ++i)
                if (auto hit = raycastMesh(crown::makeConnectorMesh(b.connectors[i], 16), s->transform, ray); hit && hit->distance < bestT) {
                    bestT = hit->distance;
                    bestB = &b;
                    bestI = static_cast<int>(i);
                }
        }
        // A crown clearly in front of the connector wins (connectors are mostly inside the crowns'
        // contact area, so a crown surface just in front of one still selects it).
        for (const auto& r : app.doc().restorations)
            if (r.crown)
                if (ScanObject* s = scanById(app, r.prepScanId))
                    if (auto hit = raycastMesh(r.crown->mesh, s->transform, ray); hit && hit->distance < bestT - 1.5)
                        return false;
        if (!bestB)
            return false;
        selectConnector(app, *bestB, bestI);
        return true;
    }

    // Runs once the data and the scan analysis are available: positions new teeth automatically
    // (tooth placement) and rebuilds saved designs.
    void prepare(DesignerApp& app)
    {
        if (app.loading() || restorationsPending(app))
            return;
        RestorationDesign* active = app.doc().active();
        if (!active || !active->supported() || !ensureBase(app, *active)) {
            if (id() == StepId::Connectors && (!active || !active->supported()))
                ready_ = true;
            return;
        }
        std::set<std::string> bridgesQueued;
        for (auto& r : app.doc().restorations) {
            if (!r.supported() || !ensureBase(app, r))
                continue;
            BridgeDesign* b = app.doc().bridgeOf(r.tooth);
            if (!r.placed && id() == StepId::ToothPlacement) {
                // One positioning task per bridge or single tooth, run one after another.
                if (b && !bridgesQueued.insert(b->label()).second)
                    continue;
                const int tooth = r.tooth;
                const bool map = distanceMap_;
                app.enqueue([&app, tooth, map] { startAutoDesign(app, tooth, map); });
            } else if ((r.adapted && !r.crown) || (!r.adapted && r.placed && !r.libraryTooth)) {
                regenerate(app, r, distanceMap_);
            } else {
                updateCrownOverlay(app, r, distanceMap_);
            }
        }
        for (auto& b : app.doc().bridges)
            if (b.connectors.empty())
                bridgeChanged(app, b);
        setAntagonistVisible(app, id() == StepId::ToothPlacement);
        viewOcclusal(app, *active, 35.0);
        ready_ = true;
    }

    struct UndoEntry {
        int tooth = 0;
        crown::CrownParameters params;
        std::vector<float> displacement;
        bool placed = false, adapted = false;
    };

    void pushUndo(const RestorationDesign& r)
    {
        undo_.push_back({r.tooth, r.params, r.displacement, r.placed, r.adapted});
        if (undo_.size() > 40)
            undo_.erase(undo_.begin());
    }

    bool ready_ = false;
    // Tooth placement
    int placeMode_ = 0; // move, rotate, scale
    bool moveAll_ = true;
    bool dragging_ = false;
    ImVec2 lastMouse_{0, 0};
    std::vector<int> dragTeeth_;
    // Free-forming
    bool brushOn_ = true;
    int brushMode_ = 0; // add/remove, smooth/flatten
    int tool_ = 0;
    std::vector<crown::BrushTool> tools_;
    std::size_t builtinCount_ = 0;
    crown::BrushTool editTool_ = [] {
        crown::BrushTool t;
        t.name = "My tool";
        t.kind = crown::BrushKind::Custom;
        return t;
    }();
    int dragPt_ = -1;
    bool painting_ = false;
    float brushRadius_ = 1.5f;
    float brushStrength_ = 0.3f;
    bool distanceMap_ = true;
    // Connectors
    bool moveInView_ = true;
    bool gizmoWasUsing_ = false;
    std::vector<std::pair<std::string, std::vector<crown::ConnectorEdit>>> connectorUndo_;
    std::vector<UndoEntry> undo_;
};

// ---------------------------------------------------------------------------
// Merge
// ---------------------------------------------------------------------------

class MergeStep final : public Step {
public:
    MergeStep() : Step(StepId::Merge) {}

    ViewLayout preferredLayout() const override { return ViewLayout::Single3D; }

    std::optional<std::string> blocker(const DesignerApp& appC) const override
    {
        if (auto m = firstMissing(appC, true, true, true))
            return m;
        auto& app = const_cast<DesignerApp&>(appC);
        if (app.tasks().busy())
            return std::string("Merging...");
        for (const auto& r : app.doc().restorations) {
            if (!r.supported() || app.doc().bridgeOf(r.tooth))
                continue;
            if (!r.merged)
                return std::format("Merge {} first.", app.toothText(r.tooth));
            if (!r.merged->ok)
                return std::format("{} could not be merged: {}", app.toothText(r.tooth), r.merged->error);
        }
        for (const auto& b : app.doc().bridges)
            if (!b.result || !b.result->ok)
                return std::format("Bridge {} is not merged{}", bridgeName(app, b), b.result ? ": " + b.result->error : ".");
        return std::nullopt;
    }

    void onEnter(DesignerApp& app) override
    {
        syncRestorations(app);
        setOverlaysVisible(app, "connector:", false);
        setOverlaysVisible(app, "die:", false);
        started_ = false;
    }

    void onLeave(DesignerApp& app) override
    {
        app.doc().removeOverlaysWithPrefix("bridge:");
        for (auto& r : app.doc().restorations)
            updateCrownOverlay(app, r, false);
    }

    void update(DesignerApp& app) override
    {
        if (!started_ && !app.loading() && !restorationsPending(app) && !app.tasks().busy()) {
            bool rebuilt = false;
            for (auto& r : app.doc().restorations)
                if (r.supported() && r.adapted && !r.crown) {
                    regenerate(app, r, false);
                    rebuilt = true;
                }
            if (!rebuilt) {
                started_ = true;
                if (RestorationDesign* a = app.doc().active(); a && a->supported() && (a->isPontic() || a->marginClosed()))
                    viewOcclusal(app, *a, 35.0);
                bool needed = false;
                for (const auto& r : app.doc().restorations)
                    needed |= r.supported() && r.adapted && !r.merged;
                for (const auto& b : app.doc().bridges)
                    needed |= !b.result;
                if (needed)
                    startMergeAll(app);
            }
        }
        // Show the finished parts once they are there.
        std::size_t done = 0;
        for (const auto& r : app.doc().restorations)
            done += r.merged ? 1 : 0;
        for (const auto& b : app.doc().bridges)
            done += b.result ? 100 : 0;
        if (done != shown_) {
            shown_ = done;
            refreshOverlays(app);
        }
    }

    void drawPanel(DesignerApp& app) override
    {
        const ui::Palette& pal = ui::palette();
        const bool busy = app.tasks().busy();
        ImGui::Spacing();
        ui::beginCard("##merge");
        ui::subheading("Merged restorations");
        bool any = false;
        for (const auto& r : app.doc().restorations) {
            if (!r.supported() || app.doc().bridgeOf(r.tooth))
                continue;
            any = true;
            ImGui::TextUnformatted(restorationLabel(app, r).c_str());
            if (!r.merged)
                ui::mutedText(busy ? "  merging..." : "  not merged");
            else if (!r.merged->ok)
                ImGui::TextColored(pal.danger, "  %s", r.merged->error.c_str());
            else {
                ImGui::TextColored(r.merged->watertight ? pal.success : pal.danger, "  %s", r.merged->watertight ? "watertight" : "NOT watertight");
                ImGui::SameLine();
                ui::mutedText("%.0f mm3, min. thickness %.2f mm", r.merged->volume, r.merged->minThickness);
            }
        }
        for (const auto& b : app.doc().bridges) {
            any = true;
            ImGui::TextUnformatted(std::format("Bridge {}", bridgeName(app, b)).c_str());
            if (!b.result)
                ui::mutedText(busy ? "  merging..." : "  not merged");
            else if (!b.result->ok)
                ImGui::TextColored(pal.danger, "  %s", b.result->error.c_str());
            else {
                ImGui::TextColored(b.result->watertight ? pal.success : pal.danger, "  %s", b.result->watertight ? "watertight" : "NOT watertight");
                ImGui::SameLine();
                ui::mutedText("%.0f mm3", b.result->volume);
                for (std::size_t i = 0; i < b.result->connectorAreas.size() && i + 1 < b.teeth.size(); ++i) {
                    const double a = b.result->connectorAreas[i];
                    ImGui::TextColored(a + 1e-3 >= b.connectorArea ? pal.success : pal.warning, "  %s-%s", app.toothText(b.teeth[i]).c_str(),
                                       app.toothText(b.teeth[i + 1]).c_str());
                    ImGui::SameLine();
                    ui::mutedText("connector %.1f mm2", a);
                }
            }
        }
        if (!any)
            ui::mutedText("Nothing to merge.");
        ui::endCard();
        ImGui::Spacing();
        ui::wrappedMutedText("Each crown is its free-formed shell united with the crown bottom (cement gap, border, minimum thickness), "
                             "minus the preparation space. Bridges are joined with their connectors.");
        const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (ui::button("Merge again", ImVec2(half, 0), !busy)) {
            clear(app);
            started_ = false;
        }
        ImGui::SameLine();
        if (ui::button("Remove merged parts", ImVec2(half, 0), !busy)) {
            clear(app);
            app.goToStep(StepId::Freeforming);
        }
    }

private:
    static void clear(DesignerApp& app)
    {
        for (auto& r : app.doc().restorations)
            r.merged.reset();
        for (auto& b : app.doc().bridges)
            b.result.reset();
        app.doc().removeOverlaysWithPrefix("bridge:");
    }

    void refreshOverlays(DesignerApp& app)
    {
        Document& doc = app.doc();
        doc.removeOverlaysWithPrefix("bridge:");
        for (auto& b : doc.bridges) {
            RestorationDesign* first = findRestoration(app, b.teeth.front());
            if (!b.result || !b.result->ok || !first)
                continue;
            DisplayMesh dm;
            dm.gpu = std::make_unique<gfx::GpuMesh>(b.result->mesh);
            dm.scanId = first->prepScanId;
            dm.color = kCrownColor;
            doc.setOverlay("bridge:" + b.label(), std::move(dm));
        }
        for (auto& r : doc.restorations)
            updateCrownOverlay(app, r, false);
    }

    bool started_ = false;
    std::size_t shown_ = ~std::size_t{0};
};

} // namespace

// ---------------------------------------------------------------------------
// Public functions
// ---------------------------------------------------------------------------

bool RestorationDesign::supported() const
{
    return type == "anatomic_crown" || type == "coping" || type == "pontic";
}

std::unique_ptr<Step> makeMarginLineStep()
{
    return std::make_unique<MarginLineStep>();
}
std::unique_ptr<Step> makeInsertionAxisStep()
{
    return std::make_unique<InsertionAxisStep>();
}
std::unique_ptr<Step> makeCrownBottomStep()
{
    return std::make_unique<CrownBottomStep>();
}
std::unique_ptr<Step> makeToothPlacementStep()
{
    return std::make_unique<CrownToolStep>(StepId::ToothPlacement);
}
std::unique_ptr<Step> makeFreeformingStep()
{
    return std::make_unique<CrownToolStep>(StepId::Freeforming);
}
std::unique_ptr<Step> makeConnectorsStep()
{
    return std::make_unique<CrownToolStep>(StepId::Connectors);
}
std::unique_ptr<Step> makeMergeStep()
{
    return std::make_unique<MergeStep>();
}

bool restorationsPending(const DesignerApp& app)
{
    return !app.doc().pendingRestorations.empty();
}

void syncRestorations(DesignerApp& app)
{
    Document& doc = app.doc();
    // Saved designs: wait until every referenced scan has been loaded.
    if (!doc.pendingRestorations.empty()) {
        if (app.loading())
            return;
        auto bysource = [&](const std::string& src) -> ScanObject* {
            for (const auto& s : doc.scans)
                if (s->source == src)
                    return s.get();
            return nullptr;
        };
        for (const auto& saved : doc.pendingRestorations)
            if (!saved.prepScanSource.empty() && !bysource(saved.prepScanSource))
                return; // still loading
        for (const auto& saved : doc.pendingRestorations) {
            RestorationDesign r;
            r.tooth = saved.tooth;
            r.type = saved.type;
            ScanObject* prep = bysource(saved.prepScanSource);
            ScanObject* ant = bysource(saved.antagonistSource);
            r.prepScanId = prep ? prep->id : 0;
            r.antagonistScanId = ant ? ant->id : 0;
            r.params = saved.params;
            r.orientation = saved.orientation;
            r.scansAssigned = prep != nullptr;
            const bool valid = prep && saved.prepVertexCount == prep->mesh->vertexCount();
            if (valid) {
                r.controls = saved.controls;
                r.margin.vertices = saved.margin;
                r.margin.closed = saved.marginClosed;
                r.prepPoint = saved.prepPoint;
                r.insertionAxis = saved.insertionAxis;
                r.displacement = saved.displacement;
                r.placed = saved.placed;
                r.adapted = saved.adapted;
            } else if (prep) {
                log::warn("The scan of tooth {} changed since the design was saved; the margin must be redrawn.", app.toothText(saved.tooth));
            }
            doc.restorations.push_back(std::move(r));
        }
        doc.pendingRestorations.clear();
        for (auto& r : doc.restorations)
            if (r.marginClosed())
                requestPrepScan(app, r.prepScanId);
        refreshMarginOverlays(app);
    }
    // Restorations from the case record.
    if (auto* rec = app.caseRecord()) {
        for (const auto& cr : rec->restorations) {
            if (!isToothBorne(cr.type) || findRestoration(app, cr.tooth))
                continue;
            RestorationDesign r;
            r.tooth = cr.tooth;
            r.type = cr.type;
            initParams(r);
            doc.restorations.push_back(std::move(r));
        }
    }
    for (auto& r : doc.restorations) {
        if (!r.scansAssigned && !app.loading() && !doc.scans.empty()) {
            assignDefaultScans(app, r);
            r.scansAssigned = true;
        } else if (!scanById(app, r.prepScanId) || (r.antagonistScanId && !scanById(app, r.antagonistScanId))) {
            assignDefaultScans(app, r);
        }
    }
    // Bridges: runs of adjacent abutments and pontics. Settings survive regrouping by matching teeth.
    {
        std::vector<std::pair<int, std::string>> list;
        for (const auto& r : doc.restorations)
            list.emplace_back(r.tooth, r.type);
        const auto groups = crown::findBridges(list);
        bool same = groups.size() == doc.bridges.size();
        for (std::size_t i = 0; same && i < groups.size(); ++i)
            same = groups[i] == doc.bridges[i].teeth;
        if (!same) {
            std::vector<BridgeDesign> next;
            for (const auto& g : groups) {
                BridgeDesign b;
                b.teeth = g;
                for (auto& old : doc.bridges)
                    if (old.teeth == g)
                        b = std::move(old);
                for (const auto& saved : doc.pendingBridges)
                    if (saved.teeth == g) {
                        b.connectorArea = saved.connectorArea;
                        b.connectorHeightRatio = saved.connectorHeightRatio;
                        b.embrasure = saved.embrasure;
                        b.axis = saved.axis;
                        b.edits = saved.edits;
                    }
                next.push_back(std::move(b));
            }
            for (const auto& old : doc.bridges)
                doc.removeOverlaysWithPrefix("connector:" + old.label() + ":");
            doc.bridges = std::move(next);
        }
        doc.pendingBridges.clear();
        // All units of a bridge are designed on the scan of its abutments.
        for (const auto& b : doc.bridges) {
            const RestorationDesign* abutment = nullptr;
            for (int t : b.teeth)
                if (const RestorationDesign* u = findRestoration(app, t); u && !u->isPontic() && u->prepScanId) {
                    abutment = u;
                    break;
                }
            if (!abutment)
                continue;
            for (int t : b.teeth)
                if (RestorationDesign* u = findRestoration(app, t); u && u->isPontic()) {
                    u->prepScanId = abutment->prepScanId;
                    u->antagonistScanId = abutment->antagonistScanId;
                }
        }
    }
    if (doc.activeRestoration >= static_cast<int>(doc.restorations.size()))
        doc.activeRestoration = 0;
    // Prefer a restoration that can actually be designed.
    if (RestorationDesign* a = doc.active(); a && !a->supported())
        for (std::size_t i = 0; i < doc.restorations.size(); ++i)
            if (doc.restorations[i].supported()) {
                doc.activeRestoration = static_cast<int>(i);
                break;
            }
    // Saved crowns: rebuild the geometry once the scan analysis is available.
    for (auto& r : doc.restorations)
        if ((r.placed || r.adapted) && !r.crown && !r.libraryTooth && (r.isPontic() || r.marginClosed()) && prepScanFor(app, r.prepScanId) &&
            !app.tasks().busy())
            regenerate(app, r, false);
}

std::vector<SavedBridge> captureBridges(const DesignerApp& appC)
{
    std::vector<SavedBridge> out = appC.doc().pendingBridges;
    for (const auto& b : appC.doc().bridges)
        out.push_back({b.teeth, b.connectorArea, b.connectorHeightRatio, b.embrasure, b.axis, b.edits});
    return out;
}

std::vector<SavedRestoration> captureRestorations(const DesignerApp& appC)
{
    auto& app = const_cast<DesignerApp&>(appC);
    std::vector<SavedRestoration> out = app.doc().pendingRestorations;
    for (const auto& r : app.doc().restorations) {
        SavedRestoration s;
        s.tooth = r.tooth;
        s.type = r.type;
        ScanObject* prep = scanById(app, r.prepScanId);
        ScanObject* ant = scanById(app, r.antagonistScanId);
        s.prepScanSource = prep ? prep->source : "";
        s.antagonistSource = ant ? ant->source : "";
        s.prepVertexCount = prep ? prep->mesh->vertexCount() : 0;
        s.controls = r.controls;
        s.margin = r.margin.vertices;
        s.marginClosed = r.margin.closed;
        s.prepPoint = r.prepPoint;
        s.insertionAxis = r.insertionAxis;
        s.params = r.params;
        s.orientation = r.orientation;
        s.displacement = r.displacement;
        s.placed = r.placed;
        s.adapted = r.adapted;
        if (r.adapted)
            s.crownFile = app.doc().bridgeOf(r.tooth) ? "design/bridge_" + bridgeName(app, *app.doc().bridgeOf(r.tooth)) + ".stl"
                                                       : "design/crown_" + app.toothText(r.tooth) + ".stl";
        out.push_back(std::move(s));
    }
    return out;
}

// Merge one restoration now (on the calling thread).
bool mergeNow(DesignerApp& app, RestorationDesign& r)
{
    if (r.merged)
        return r.merged->ok;
    if (!r.adapted || !r.crown || !ensureBottom(app, r))
        return false;
    r.merged = std::make_shared<const crown::CrownMerge>(crown::mergeCrown(*r.base, *r.crown, r.isPontic() ? Mesh{} : *r.bottom));
    return r.merged->ok;
}

std::vector<CrownExport> crownExports(DesignerApp& app, bool merge)
{
    std::vector<CrownExport> out;
    for (auto& b : app.doc().bridges) {
        auto units = bridgeUnits(app, b);
        if (units.empty())
            continue; // not completely designed
        if (!b.result && merge) {
            if (b.connectors.empty())
                bridgeChanged(app, b);
            bool ok = true;
            for (auto& u : units) {
                RestorationDesign* r = findRestoration(app, u.tooth);
                ok = ok && r && mergeNow(app, *r);
                if (ok)
                    u.solid = std::shared_ptr<const Mesh>(r->merged, &r->merged->mesh);
            }
            if (ok)
                b.result = mergeBridge(units, b.connectors);
        }
        if (!b.result || !b.result->ok) {
            log::warn("Bridge {} is not exported: {}", bridgeName(app, b), b.result ? b.result->error : "not merged");
            continue;
        }
        auto mesh = std::make_shared<Mesh>(b.result->mesh);
        const std::string name = bridgeName(app, b);
        out.push_back({b.teeth.front(), "Bridge " + name, "bridge_" + name, mesh,
                       app.numbering() == dental::Numbering::Universal ? std::format("OcclusaCAD Bridge {} (Universal; FDI {}), scan coordinates, mm", name, b.label())
                                                                       : std::format("OcclusaCAD Bridge {} (FDI), scan coordinates, mm", name)});
    }
    for (auto& r : app.doc().restorations) {
        if (!r.adapted || app.doc().bridgeOf(r.tooth) || r.isPontic())
            continue;
        if (merge)
            mergeNow(app, r);
        if (!r.merged || !r.merged->ok) {
            log::warn("Crown {} is not exported: {}", app.toothText(r.tooth), r.merged ? r.merged->error : "not merged");
            continue;
        }
        auto mesh = std::make_shared<Mesh>(r.merged->mesh);
        mesh->colors.clear();
        const char* kind = r.params.coping ? "Coping" : "Crown";
        const std::string tooth = app.toothText(r.tooth);
        out.push_back({r.tooth, std::format("{} {}", kind, tooth), std::format("crown_{}", tooth), mesh,
                       app.numbering() == dental::Numbering::Universal ? std::format("OcclusaCAD {} {} (Universal; FDI {}), scan coordinates, mm", kind, tooth, r.tooth)
                                                                       : std::format("OcclusaCAD {} {} (FDI), scan coordinates, mm", kind, tooth)});
    }
    return out;
}

bool allRestorationsExported(const DesignerApp& app)
{
    for (const auto& r : app.doc().restorations)
        if (r.adapted && !r.crown)
            return false;
    return true;
}

bool isGeneratedRestorationFile(const std::string& rel)
{
    auto starts = [&](std::string_view p) { return rel.rfind(p, 0) == 0; };
    return (starts("design/crown_") || starts("design/bridge_") || starts("design/abutment_")) && rel.size() > 4 && rel.compare(rel.size() - 4, 4, ".stl") == 0;
}

// ---------------------------------------------------------------------------
// Headless demo
// ---------------------------------------------------------------------------

namespace {

struct DemoPrep {
    int tooth = 0;
    glm::vec3 prepPoint{0.0f};
    std::vector<glm::dvec3> margin;
    std::optional<glm::dvec3> axis;
};

// Truth files: a single preparation ({tooth, prepPoint, margin, axis}) or several ({preps: [...]}).
std::vector<DemoPrep> demoPreps(const nlohmann::json& truth)
{
    std::vector<DemoPrep> out;
    auto read = [](const nlohmann::json& j) {
        DemoPrep d;
        d.tooth = j.value("tooth", 0);
        const auto& p = j["prepPoint"];
        d.prepPoint = glm::vec3(p[0].get<float>(), p[1].get<float>(), p[2].get<float>());
        for (const auto& q : j["margin"])
            d.margin.emplace_back(q[0].get<double>(), q[1].get<double>(), q[2].get<double>());
        if (j.contains("axis")) {
            const auto& a = j["axis"];
            d.axis = glm::normalize(glm::dvec3(a[0].get<double>(), a[1].get<double>(), a[2].get<double>()));
        }
        return d;
    };
    if (truth.contains("preps"))
        for (const auto& j : truth["preps"])
            out.push_back(read(j));
    else
        out.push_back(read(truth));
    return out;
}

double polylineDistance(const glm::dvec3& p, const std::vector<glm::dvec3>& loop)
{
    double best = 1e9;
    for (std::size_t i = 0; i < loop.size(); ++i) {
        const glm::dvec3 a = loop[i], b = loop[(i + 1) % loop.size()];
        const glm::dvec3 ab = b - a;
        const double t = std::clamp(glm::dot(p - a, ab) / std::max(glm::dot(ab, ab), 1e-12), 0.0, 1.0);
        best = std::min(best, glm::length(p - (a + ab * t)));
    }
    return best;
}

} // namespace

bool runCrownDemo(DesignerApp& app, CrownDemo& demo)
{
    if (demo.state >= 100)
        return false;
    if (app.loading() || demo.waitFrames-- > 0)
        return true;
    auto finish = [&](int code) {
        if (code)
            app.setExitCode(code);
        demo.state = 100;
        return false;
    };
    std::vector<DemoPrep> preps;
    try {
        std::ifstream in(demo.truthFile);
        preps = demoPreps(nlohmann::json::parse(in));
    } catch (const std::exception& e) {
        log::error("DEMO cannot read {}: {}", demo.truthFile.string(), e.what());
        return finish(2);
    }
    if (preps.empty())
        return finish(2);
    syncRestorations(app);
    Document& doc = app.doc();
    const std::size_t k = std::min(demo.index, preps.size() - 1);
    RestorationDesign* r = findRestoration(app, preps[k].tooth);
    switch (demo.state) {
    case 0: // wait for data
        if (doc.scans.empty() || restorationsPending(app) || !r || !r->prepScanId)
            return true;
        app.goToStep(StepId::MarginLine);
        requestPrepScan(app, r->prepScanId);
        demo.state = 1;
        return true;
    case 1: // simulated click on the preparation
        if (!prepScanFor(app, r->prepScanId))
            return true;
        doc.activeRestoration = static_cast<int>(r - doc.restorations.data());
        startMarginDetection(app, preps[k].tooth, preps[k].prepPoint);
        demo.state = 2;
        return true;
    case 2: { // compare with the ground-truth margin
        if (!r->marginClosed()) {
            log::error("DEMO margin detection failed for tooth {}", preps[k].tooth);
            return finish(3);
        }
        double sum = 0.0, worst = 0.0;
        const auto pts = r->margin.points(*scanById(app, r->prepScanId)->mesh);
        for (const auto& p : pts) {
            const double d = polylineDistance(glm::dvec3(p), preps[k].margin);
            sum += d;
            worst = std::max(worst, d);
        }
        const double mean = pts.empty() ? 0.0 : sum / static_cast<double>(pts.size());
        log::info("DEMO margin error vs ground truth (tooth {}): mean {:.3f} mm, max {:.3f} mm ({} points)", preps[k].tooth, mean, worst, pts.size());
        if (demo.maxMarginError > 0.0 && worst > demo.maxMarginError) {
            log::error("DEMO margin error {:.3f} mm exceeds the limit of {:.3f} mm", worst, demo.maxMarginError);
            return finish(3);
        }
        if (++demo.index < preps.size()) {
            demo.state = 1;
            return true;
        }
        app.goToStep(StepId::InsertionAxis);
        demo.state = 3;
        demo.waitFrames = 2;
        return true;
    }
    case 3:
        for (const auto& pr : preps) {
            RestorationDesign* u = findRestoration(app, pr.tooth);
            if (!u || !u->insertionAxis) {
                log::error("DEMO insertion axis missing for tooth {}: {}", pr.tooth, u ? u->dieError : "");
                return finish(3);
            }
            if (pr.axis)
                log::info("DEMO insertion axis of {} deviates {:.1f} deg from the preparation axis", pr.tooth, crown::axisDivergence(*pr.axis, *u->insertionAxis));
        }
        if (!demo.library.empty())
            for (auto& u : doc.restorations) {
                if (!crown::ToothLibraryRegistry::instance().find(demo.library)) {
                    log::error("DEMO unknown tooth library {}", demo.library);
                    return finish(2);
                }
                u.params.library = demo.library;
            }
        app.goToStep(StepId::CrownBottom);
        demo.state = 4;
        demo.waitFrames = 3;
        return true;
    case 4: // crown bottoms exist, then tooth placement positions the teeth automatically
        for (auto& u : doc.restorations)
            if (u.supported() && !u.isPontic() && !ensureBottom(app, u)) {
                log::error("DEMO crown bottom of tooth {} failed: {}", u.tooth, u.dieError);
                return finish(3);
            }
        app.goToStep(StepId::ToothPlacement);
        demo.state = 5;
        demo.waitFrames = 3;
        return true;
    case 5: { // wait for the positioning, then adapt to the margins
        for (const auto& u : doc.restorations)
            if (u.supported() && !u.placed)
                return true;
        if (app.tasks().busy())
            return true;
        for (auto& u : doc.restorations)
            if (u.supported() && !u.adapted)
                adaptToMargin(app, u.tooth, false);
        app.goToStep(StepId::Freeforming);
        demo.state = 6;
        demo.waitFrames = 3;
        return true;
    }
    case 6: // free-forming: adapt the contacts (approximal and occlusal) on the shells
        for (auto& u : doc.restorations) {
            if (!u.supported() || u.isPontic() || !u.crown || !ensureBase(app, u))
                continue;
            if (!u.params.coping)
                crown::adaptContacts(*u.base, u.effectiveParams(), *u.contacts, u.displacement, true, u.contacts->antagonist != nullptr, 0.6, true);
            regenerate(app, u, false);
        }
        app.goToStep(StepId::Connectors);
        demo.state = 7;
        demo.waitFrames = 3;
        return true;
    case 7:
        for (const auto& b : doc.bridges)
            if (b.connectors.empty()) {
                log::error("DEMO bridge {} has no connectors", b.label());
                return finish(3);
            }
        app.goToStep(StepId::Merge); // merges everything
        demo.state = 8;
        demo.waitFrames = 3;
        return true;
    case 8: {
        if (app.tasks().busy())
            return true;
        for (const auto& u : doc.restorations) {
            if (!u.supported())
                continue;
            if (!u.merged || !u.merged->ok) {
                log::error("DEMO merge of tooth {} failed: {}", u.tooth, u.merged ? u.merged->error : std::string("missing"));
                return finish(3);
            }
            log::info("DEMO {} {} ({}): {} triangles, volume {:.0f} mm3, min thickness {:.2f} mm, {}", u.isPontic() ? "pontic" : "crown", u.tooth,
                      u.params.library.empty() ? crown::ToothLibraryRegistry::kDefaultLibrary : u.params.library, u.merged->mesh.triangleCount(),
                      u.merged->volume, u.merged->minThickness, u.merged->watertight ? "watertight" : "NOT watertight");
            if (!u.merged->watertight)
                return finish(3);
        }
        for (const auto& b : doc.bridges) {
            if (!b.result || !b.result->ok) {
                log::error("DEMO bridge {} not merged: {}", b.label(), b.result ? b.result->error : "missing");
                return finish(3);
            }
            std::string areas;
            double minArea = 1e9;
            for (double a : b.result->connectorAreas) {
                areas += std::format("{}{:.1f}", areas.empty() ? "" : ", ", a);
                minArea = std::min(minArea, a);
            }
            log::info("DEMO bridge {}: volume {:.0f} mm3, {}, connectors {} mm2 (target {:.1f})", b.label(), b.result->volume,
                      b.result->watertight ? "watertight" : "NOT watertight", areas, b.connectorArea);
            if (!b.result->watertight || minArea + 1e-3 < b.connectorArea)
                return finish(3);
        }
        // Edit the first connector of each bridge (bigger, raised), re-merge and check that the merged
        // bridge honours it.
        for (auto& b : doc.bridges) {
            if (b.connectors.empty())
                continue;
            crown::ConnectorEdit& e = b.editFor(0);
            e.area = b.connectorArea + 3.0;
            e.offset.z += 0.3;
            bridgeChanged(app, b);
            auto units = bridgeUnits(app, b);
            for (auto& u : units)
                if (RestorationDesign* r = findRestoration(app, u.tooth); r && r->merged)
                    u.solid = std::shared_ptr<const Mesh>(r->merged, &r->merged->mesh);
            b.result = mergeBridge(units, b.connectors);
            const double measured = b.result->ok && !b.result->connectorAreas.empty() ? b.result->connectorAreas[0] : 0.0;
            log::info("DEMO connector {}-{} edited to {:.1f} mm2 (+0.3 mm occlusal): merged {}, measured {:.1f} mm2{}", b.teeth[0], b.teeth[1], e.area,
                      b.result->watertight ? "watertight" : "NOT watertight", measured, b.warnings[0].empty() ? "" : " - " + b.warnings[0]);
            if (!b.result->ok || !b.result->watertight || measured + 1e-3 < e.area)
                return finish(3);
        }
        if (demo.save && app.caseMode())
            app.saveDesign(true);
        demo.state = 9;
        demo.waitFrames = 2;
        return true;
    }
    default:
        return finish(0);
    }
}

} // namespace occlusa::designer

