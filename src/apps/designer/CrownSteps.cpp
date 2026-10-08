// Crown & bridge tools: margin line, insertion axis and crown design.
#include "apps/designer/CrownSteps.h"

#include "apps/designer/DesignerApp.h"
#include "core/Dental.h"
#include "core/Geometry.h"
#include "core/Log.h"
#include "core/crown/InsertionAxis.h"
#include "core/crown/ToothLibrary.h"
#include "core/Platform.h"
#include "ui/FileDialog.h"
#include "ui/Fonts.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"

#include <imgui.h>
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

void updateCrownOverlay(DesignerApp& app, const RestorationDesign& r, bool distanceMap)
{
    // Crowns would hide the preparation while the margin or the axis is being edited.
    const bool visible = app.currentStep() != StepId::MarginLine && app.currentStep() != StepId::InsertionAxis;
    Document& doc = app.doc();
    if (!r.crown) {
        doc.removeOverlay(key("crown", r.tooth));
        return;
    }
    Mesh m = r.crown->mesh;
    if (distanceMap && r.contacts) {
        const auto d = crown::contactDistances(*r.crown, *r.contacts);
        m.colors.assign(m.positions.size(), kCrownColor);
        for (std::size_t v = r.crown->outerBegin; v < d.size(); ++v)
            m.colors[v] = contactColor(d[v]);
    }
    DisplayMesh dm;
    dm.gpu = std::make_unique<gfx::GpuMesh>(m);
    dm.scanId = r.prepScanId;
    dm.color = kCrownColor;
    dm.vertexColors = distanceMap;
    dm.visible = visible;
    doc.setOverlay(key("crown", r.tooth), std::move(dm));
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
    std::shared_ptr<const crown::CrownMesh> crown;
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
        part.mesh = &u.crown->mesh;
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
        if (!u || !u->base || !u->crown)
            return {};
        units.push_back({t, u->isPontic(), u->base, u->crown});
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
    const bool visible = app.currentStep() != StepId::MarginLine && app.currentStep() != StepId::InsertionAxis;
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

void regenerate(DesignerApp& app, RestorationDesign& r, bool distanceMap)
{
    if (!ensureBase(app, r))
        return;
    const crown::CrownParameters params = r.effectiveParams();
    auto c = std::make_shared<crown::CrownMesh>(crown::buildCrown(*r.base, params, r.displacement));
    if (!r.displacement.empty() && r.displacement.size() != c->mesh.vertexCount()) {
        r.displacement.clear();
        c = std::make_shared<crown::CrownMesh>(crown::buildCrown(*r.base, params, r.displacement));
    }
    r.crown = c;
    r.crownGenerated = true;
    updateCrownOverlay(app, r, distanceMap);
    if (BridgeDesign* b = app.doc().bridgeOf(r.tooth))
        bridgeChanged(app, *b);
}

void marginChanged(DesignerApp& app, RestorationDesign& r)
{
    r.invalidateDie();
    r.insertionAxis.reset();
    r.orientation.reset();
    r.displacement.clear();
    r.crownGenerated = false;
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
    params.rotationDeg = 0.0;
    app.tasks().start("Designing crown", [&app, tooth, base, contacts, params, distanceMap](const ProgressFn& progress) -> ui::TaskRunner::Continuation {
        crown::CrownParameters p = params;
        reportProgress(progress, 0.1f, "Fitting to the neighbouring teeth");
        crown::fitProximal(*base, *contacts, p);
        reportProgress(progress, 0.35f, "Fitting to the antagonist");
        crown::fitOcclusalHeight(*base, *contacts, p);
        reportProgress(progress, 0.6f, "Adapting contacts");
        std::vector<float> disp;
        if (!p.coping)
            crown::adaptContacts(*base, p, *contacts, disp, true, contacts->antagonist != nullptr);
        auto c = std::make_shared<crown::CrownMesh>(crown::buildCrown(*base, p, disp));
        return [&app, tooth, p, disp, c, distanceMap] {
            RestorationDesign* r = findRestoration(app, tooth);
            if (!r)
                return;
            r->params = p;
            r->displacement = disp;
            r->crown = c;
            r->crownGenerated = true;
            updateCrownOverlay(app, *r, distanceMap);
            app.markModified();
            log::info("Crown {} designed: {:.0f} mm3, min. thickness {:.2f} mm{}", app.toothText(tooth), c->volume, c->minThickness, c->watertight ? "" : " (not watertight)");
        };
    });
}

// Designs all units of a bridge together: outer contacts from the scan, inner sides meeting at
// the neighbours, heights from the antagonist, contacts adapted, then connectors and the merge.
void startBridgeAutoDesign(DesignerApp& app, BridgeDesign& b, bool distanceMap)
{
    struct Unit {
        int tooth;
        bool pontic;
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
        p.rotationDeg = 0.0;
        units.push_back({t, u->isPontic(), u->base, u->contacts, p});
    }
    const std::string label = bridgeName(app, b);
    const dental::Numbering numbering = app.numbering();
    const BridgeDesign settings = [&] {
        BridgeDesign s;
        s.teeth = b.teeth;
        s.connectorArea = b.connectorArea;
        s.connectorHeightRatio = b.connectorHeightRatio;
        s.embrasure = b.embrasure;
        s.edits = b.edits;
        return s;
    }();
    const glm::dvec3 axis = *b.axis;
    app.tasks().start("Designing bridge " + label, [&app, units, settings, axis, label, distanceMap, numbering](const ProgressFn& progress) mutable
                      -> ui::TaskRunner::Continuation {
        reportProgress(progress, 0.05f, "Fitting the units");
        for (auto& u : units)
            crown::fitProximal(*u.base, *u.contacts, u.params);
        std::vector<crown::BridgeFitUnit> fit;
        for (auto& u : units)
            fit.push_back({u.base->frame, &u.params});
        crown::fitBridgeWidths(fit);
        reportProgress(progress, 0.25f, "Fitting to the antagonist");
        for (auto& u : units)
            crown::fitOcclusalHeight(*u.base, *u.contacts, u.params);
        reportProgress(progress, 0.45f, "Adapting contacts");
        std::vector<std::vector<float>> disp(units.size());
        std::vector<UnitSnapshot> snaps;
        for (std::size_t i = 0; i < units.size(); ++i) {
            auto& u = units[i];
            if (!u.params.coping)
                crown::adaptContacts(*u.base, u.params, *u.contacts, disp[i], true, u.contacts->antagonist != nullptr);
            snaps.push_back({u.tooth, u.pontic, u.base, std::make_shared<const crown::CrownMesh>(crown::buildCrown(*u.base, u.params, disp[i]))});
        }
        reportProgress(progress, 0.8f, "Connectors and merging");
        std::vector<std::string> warnings;
        auto connectors = placeConnectors(snaps, axis, settings, warnings, numbering);
        auto merged = mergeBridge(snaps, connectors);
        return [&app, units, disp, snaps, connectors, merged, warnings, label, distanceMap] {
            for (std::size_t i = 0; i < units.size(); ++i) {
                RestorationDesign* r = findRestoration(app, units[i].tooth);
                if (!r)
                    return;
                r->params = units[i].params;
                r->displacement = disp[i];
                r->crown = snaps[i].crown;
                r->crownGenerated = true;
                updateCrownOverlay(app, *r, distanceMap);
            }
            if (BridgeDesign* b = app.doc().bridgeOf(units.front().tooth)) {
                b->connectors = connectors;
                b->warnings = warnings;
                b->result = merged;
                updateConnectorOverlays(app, *b);
            }
            app.markModified();
            if (merged->ok)
                log::info("Bridge {} designed: {:.0f} mm3, {}, connectors {}", label, merged->volume, merged->watertight ? "watertight" : "NOT watertight",
                          [&] {
                              std::string s;
                              for (double a : merged->connectorAreas)
                                  s += std::format("{}{:.1f} mm2", s.empty() ? "" : ", ", a);
                              return s;
                          }());
            else
                log::warn("Bridge {} could not be merged: {}", label, merged->error);
            for (const auto& w : warnings)
                if (!w.empty())
                    log::warn("Bridge {}: {}", label, w);
        };
    });
}

// Merge a bridge whose units are designed (on the worker thread).
void startBridgeMerge(DesignerApp& app, BridgeDesign& b)
{
    auto units = bridgeUnits(app, b);
    if (units.empty())
        return;
    auto connectors = b.connectors;
    const std::string label = bridgeName(app, b);
    const int firstTooth = b.teeth.front();
    app.tasks().start("Merging bridge " + label, [&app, units, connectors, firstTooth](const ProgressFn& progress) -> ui::TaskRunner::Continuation {
        reportProgress(progress, 0.2f, "Joining units and connectors");
        auto merged = mergeBridge(units, connectors);
        return [&app, merged, firstTooth] {
            if (BridgeDesign* b = app.doc().bridgeOf(firstTooth))
                b->result = merged;
            if (!merged->ok)
                ui::showError("Bridge", "The bridge could not be merged: " + merged->error);
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
            else if (r.crownGenerated)
                label += "  - designed";
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
            if (needCrown && !r.crownGenerated)
                return std::format("Design pontic {}.", app.toothText(r.tooth));
            continue;
        }
        if (needMargin && !r.marginClosed())
            return std::format("Define the margin line of tooth {}.", app.toothText(r.tooth));
        if (needMargin && !r.dieError.empty())
            return std::format("Tooth {}: {}", app.toothText(r.tooth), r.dieError);
        if (needAxis && !r.insertionAxis)
            return std::format("Set the insertion axis of tooth {}.", app.toothText(r.tooth));
        if (needCrown && !r.crownGenerated)
            return std::format("Design the crown of tooth {}.", app.toothText(r.tooth));
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
        if (RestorationDesign* r = app.doc().active(); r && r->supported() && !r->isPontic())
            requestPrepScan(app, r->prepScanId);
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
                                     "point again (or press Close) to finish."); break;
        default: ui::wrappedMutedText("Click on the scan to move the nearest control point there; the margin re-traces between "
                                      "its neighbours."); break;
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
            if (!r)
                return;
            ScanObject* s = scanById(app, r->prepScanId);
            if (s && !r->controls.empty()) {
                const glm::dvec3 fwd = app.view3D(ViewId::Main3D).camera.forward();
                const float sc = ImGui::GetStyle().FontScaleDpi;
                for (std::size_t i = 0; i < r->controls.size(); ++i) {
                    const std::uint32_t v = r->controls[i];
                    const glm::dvec3 n = transformVector(s->transform, glm::dvec3(s->mesh->normals[v]));
                    if (glm::dot(n, fwd) > 0.25)
                        continue; // facing away (behind the tooth)
                    if (auto p = proj(transformPoint(s->transform, glm::dvec3(s->mesh->positions[v])))) {
                        dl->AddCircleFilled(*p, 5.5f * sc, IM_COL32(0, 0, 0, 150));
                        dl->AddCircleFilled(*p, 4.5f * sc, i == 0 && !r->margin.closed ? IM_COL32(255, 255, 255, 255) : kControlColor);
                    }
                }
            }
            if (!prepScanFor(app, r->prepScanId))
                drawHint(dl, proj, "Analysing the scan...");
            else if (mode_ == 0 && !r->marginClosed())
                drawHint(dl, proj, "Click on the top of the preparation");
            else if (mode_ == 1 && !r->margin.closed)
                drawHint(dl, proj, r->controls.empty() ? "Click on the margin to start" : "Click the next margin point");
        };
    }

    void onViewEvent(DesignerApp& app, ViewId view, const ViewEvents& ev) override
    {
        if (view != ViewId::Main3D || !ev.click || app.tasks().busy())
            return;
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
        } else if (!r->controls.empty()) {
            const auto& pos = ps->mesh().positions;
            std::size_t best = 0;
            float bestD = 1e9f;
            for (std::size_t i = 0; i < r->controls.size(); ++i) {
                const float d = glm::length(pos[r->controls[i]] - pos[v]);
                if (d < bestD) {
                    bestD = d;
                    best = i;
                }
            }
            if (bestD < 4.0f) {
                r->controls[best] = v;
                retrace(app, *r, r->margin.closed);
            }
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

    int mode_ = 0;
    bool viewPending_ = false;
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

class CrownDesignStep final : public Step {
public:
    CrownDesignStep() : Step(StepId::CrownDesign) {}

    ViewLayout preferredLayout() const override { return ViewLayout::Single3D; }

    std::optional<std::string> blocker(const DesignerApp& app) const override { return firstMissing(app, true, true, true); }

    void onEnter(DesignerApp& app) override
    {
        syncRestorations(app);
        app.view3D(ViewId::Main3D).pickCursor = brush_ != 0;
        setOverlaysVisible(app, "die:", false);
        setAntagonistVisible(app, false);
        ready_ = false;
        prepare(app);
    }

    void onLeave(DesignerApp& app) override
    {
        app.view3D(ViewId::Main3D).pickCursor = false;
        app.view3D(ViewId::Main3D).gizmoPivot.reset();
        for (auto& b : app.doc().bridges)
            if (b.selectedConnector >= 0) {
                b.selectedConnector = -1;
                updateConnectorOverlays(app, b);
            }
        setAntagonistVisible(app, true);
    }

    void update(DesignerApp& app) override
    {
        if (!ready_)
            prepare(app);
    }

    void drawPanel(DesignerApp& app) override
    {
        const ui::Palette& pal = ui::palette();
        drawRestorationCard(app, false);
        RestorationDesign* r = app.doc().active();
        if (!r || !r->supported())
            return;
        if (!ready_) {
            ImGui::Spacing();
            ui::wrappedMutedText("Preparing the design...");
            return;
        }
        if (!ensureBase(app, *r)) {
            ImGui::Spacing();
            ui::wrappedMutedText(r->dieError.empty() ? "Define the margin line and insertion axis first." : r->dieError.c_str());
            return;
        }
        const bool busy = app.tasks().busy();
        crown::CrownParameters before = r->params;
        bool geometryChanged = false, baseChanged = false;
        const float full = -FLT_MIN;

        BridgeDesign* bridge = app.doc().bridgeOf(r->tooth);
        ImGui::Spacing();
        ui::beginCard("##crown");
        ui::subheading(r->isPontic() ? "Pontic" : "Crown");
        if (drawLibraryPicker(app, *r))
            geometryChanged = true;
        int kind = static_cast<int>(r->params.kind);
        ImGui::SetNextItemWidth(full);
        if (ImGui::BeginCombo("##kind", crown::toothTemplate(r->params.kind, r->params.upper).name)) {
            for (int k = 0; k <= static_cast<int>(crown::ToothKind::SecondMolar); ++k)
                if (ImGui::Selectable(crown::toothTemplate(static_cast<crown::ToothKind>(k), r->params.upper).name, k == kind)) {
                    r->params.kind = static_cast<crown::ToothKind>(k);
                    geometryChanged = true;
                }
            ImGui::EndCombo();
        }
        if (!r->isPontic()) {
            int coping = r->params.coping ? 1 : 0;
            ImGui::SetNextItemWidth(full);
            if (ui::segmented("crowntype", {"Anatomic", "Coping"}, coping)) {
                r->params.coping = coping == 1;
                geometryChanged = true;
            }
        }
        ImGui::Spacing();
        if (ui::primaryButton(bridge ? "Auto design bridge" : "Auto design", ImVec2(full, 0), !busy && !app.readOnly())) {
            pushUndo(*r);
            startAutoDesign(app, r->tooth, distanceMap_);
        }
        ui::wrappedMutedText(bridge ? "Designs all units of the bridge: outer contacts, heights from the antagonist, connectors."
                                    : "Fits the tooth between the neighbours, to the antagonist and adapts the contacts.");
        ui::endCard();

        if (bridge)
            drawBridgeCard(app, *bridge, busy);

        ImGui::Spacing();
        ui::beginCard("##place");
        ui::subheading("Shape and position");
        auto slider = [&](const char* id, double& value, double fallback, float lo, float hi, const char* fmt) {
            float v = static_cast<float>(value > 0 ? value : fallback);
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat(id, &v, lo, hi, fmt)) {
                value = v;
                geometryChanged = true;
            }
            if (ImGui::IsItemActivated())
                pushUndo(*r);
        };
        // Unset sizes show the library tooth's own dimensions.
        const auto libShape = crown::toothShape(r->params.library, r->params.kind, r->params.upper);
        slider("##h", r->params.crownHeight, libShape->height, 3.0f, 14.0f, "Height  %.2f mm");
        slider("##hm", r->params.halfMesial, libShape->halfMesial, 2.0f, 8.0f, "Mesial  %.2f mm");
        slider("##hd", r->params.halfDistal, libShape->halfDistal, 2.0f, 8.0f, "Distal  %.2f mm");
        slider("##hb", r->params.halfBuccal, libShape->halfBuccal, 2.0f, 8.0f, "Buccal  %.2f mm");
        slider("##hl", r->params.halfLingual, libShape->halfLingual, 2.0f, 8.0f, "Lingual  %.2f mm");
        auto sliderSigned = [&](const char* id, double& value, float lo, float hi, const char* fmt) {
            float v = static_cast<float>(value);
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat(id, &v, lo, hi, fmt)) {
                value = v;
                geometryChanged = true;
            }
            if (ImGui::IsItemActivated())
                pushUndo(*r);
        };
        sliderSigned("##rot", r->params.rotationDeg, -45.0f, 45.0f, "Rotation  %.1f deg");
        sliderSigned("##cusp", r->params.cuspScale, 0.0f, 2.0f, "Cusp height  x%.2f");
        const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (ui::button("Flip mesial/distal", ImVec2(half, 0))) {
            pushUndo(*r);
            r->params.flipMesioDistal = !r->params.flipMesioDistal;
            geometryChanged = true;
        }
        ImGui::SameLine();
        if (ui::button("Flip buccal/lingual", ImVec2(half, 0))) {
            pushUndo(*r);
            r->params.flipBuccoLingual = !r->params.flipBuccoLingual;
            geometryChanged = true;
        }
        if (r->isPontic()) {
            float ridge = static_cast<float>(r->params.ridgeOffset);
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat("##ridge", &ridge, -0.5f, 1.0f, "Ridge contact  %.2f mm")) {
                r->params.ridgeOffset = ridge;
                r->invalidateDie(); // the basal surface is re-projected onto the ridge
                geometryChanged = true;
            }
            if (ImGui::IsItemActivated())
                pushUndo(*r);
            ui::mutedText("Negative values press into the tissue (ovate pontic).");
        }
        ui::endCard();

        ImGui::Spacing();
        ui::beginCard("##contacts");
        ui::subheading("Contacts");
        auto target = [&](const char* id, double& value, const char* fmt) {
            float v = static_cast<float>(value);
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat(id, &v, -0.2f, 0.4f, fmt))
                value = v;
        };
        target("##pt", r->params.proximalTarget, "Proximal  %.2f mm");
        target("##ot", r->params.occlusalTarget, "Occlusal  %.2f mm");
        ui::mutedText("Negative values overlap (tight contact).");
        if (ui::button("Adapt proximal", ImVec2(half, 0), !busy && r->contacts->neighbors != nullptr))
            adapt(app, *r, true, false);
        ImGui::SameLine();
        if (ui::button("Adapt occlusal", ImVec2(half, 0), !busy && r->contacts->antagonist != nullptr))
            adapt(app, *r, false, true);
        if (ImGui::Checkbox("Show distance map", &distanceMap_))
            updateCrownOverlay(app, *r, distanceMap_);
        if (scanById(app, r->antagonistScanId)) {
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

        ImGui::Spacing();
        ui::beginCard("##freeform");
        ui::subheading("Free-form");
        ImGui::SetNextItemWidth(full);
        if (ui::segmented("brush", {"Off", "Add", "Remove", "Smooth"}, brush_))
            app.view3D(ViewId::Main3D).pickCursor = brush_ != 0;
        ImGui::SetNextItemWidth(full);
        ImGui::SliderFloat("##br", &brushRadius_, 0.5f, 5.0f, "Radius  %.1f mm");
        ImGui::SetNextItemWidth(full);
        ImGui::SliderFloat("##bs", &brushStrength_, 0.02f, 0.5f, "Strength  %.2f mm");
        ui::mutedText(brush_ ? "Click on the crown to apply." : "Choose a tool, then click on the crown.");
        ui::endCard();

        ImGui::Spacing();
        if (!r->isPontic() && ImGui::CollapsingHeader("Cement space and thickness")) {
            auto mm = [&](const char* id, double& value, float lo, float hi, const char* fmt) {
                float v = static_cast<float>(value);
                ImGui::SetNextItemWidth(full);
                if (ImGui::SliderFloat(id, &v, lo, hi, fmt)) {
                    value = v;
                    baseChanged = true;
                }
            };
            mm("##cg", r->params.cementGap, 0.0f, 0.15f, "Cement gap  %.3f mm");
            mm("##eg", r->params.extraGap, 0.0f, 0.15f, "Extra gap  %.3f mm");
            mm("##dm", r->params.distanceToMargin, 0.0f, 2.0f, "Distance to margin  %.2f mm");
            bool bo = r->params.blockOutUndercuts;
            if (ImGui::Checkbox("Block out undercuts", &bo)) {
                r->params.blockOutUndercuts = bo;
                baseChanged = true;
            }
            float mt = static_cast<float>(r->params.minThickness);
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat("##mt", &mt, 0.2f, 2.0f, "Min. thickness  %.2f mm")) {
                r->params.minThickness = mt;
                geometryChanged = true;
            }
            float mg = static_cast<float>(r->params.marginThickness);
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat("##mg", &mg, 0.0f, 0.4f, "Margin thickness  %.2f mm")) {
                r->params.marginThickness = mg;
                geometryChanged = true;
            }
            if (r->params.coping) {
                float ct = static_cast<float>(r->params.copingThickness);
                ImGui::SetNextItemWidth(full);
                if (ImGui::SliderFloat("##ct", &ct, 0.3f, 1.5f, "Coping thickness  %.2f mm")) {
                    r->params.copingThickness = ct;
                    geometryChanged = true;
                }
            }
        }

        ImGui::Spacing();
        ui::beginCard("##result");
        ui::subheading("Result");
        if (r->crown) {
            const auto& c = *r->crown;
            if (c.watertight)
                ImGui::TextColored(pal.success, "Closed, watertight");
            else
                ImGui::TextColored(pal.danger, "Not watertight");
            ui::mutedText("Volume %.0f mm3, %zu triangles", c.volume, c.mesh.triangleCount());
            ui::mutedText("Min. thickness %.2f mm%s", c.minThickness, c.thickenedVertices ? " (thickened)" : "");
        } else {
            ui::mutedText(busy ? "Designing..." : "Not designed yet");
        }
        if (ui::button("Undo", ImVec2(half, 0), !undo_.empty() && !busy)) {
            r->params = undo_.back().first;
            r->displacement = undo_.back().second;
            undo_.pop_back();
            r->invalidateBase();
            regenerate(app, *r, distanceMap_);
            app.markModified();
        }
        ImGui::SameLine();
        if (ui::button("Reset free-form", ImVec2(half, 0), !r->displacement.empty() && !busy)) {
            pushUndo(*r);
            r->displacement.clear();
            regenerate(app, *r, distanceMap_);
            app.markModified();
        }
        ui::endCard();

        if (baseChanged) {
            r->base.reset();
            r->crown.reset();
        }
        if ((geometryChanged || baseChanged) && !busy) {
            if (r->params.kind != before.kind || r->params.coping != before.coping || r->params.library != before.library)
                r->displacement.clear();
            regenerate(app, *r, distanceMap_);
            app.markModified();
        }
    }

    void onViewEvent(DesignerApp& app, ViewId view, const ViewEvents& ev) override
    {
        if (view != ViewId::Main3D)
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
        if (brush_ == 0) {
            if (!pickConnector(app, *ev.click))
                if (auto [b, i] = selected(app); b)
                    selectConnector(app, *b, -1);
            return;
        }
        RestorationDesign* r = app.doc().active();
        if (!r || !r->crown || !r->base)
            return;
        ScanObject* s = scanById(app, r->prepScanId);
        if (!s)
            return;
        auto hit = raycastMesh(r->crown->mesh, s->transform, *ev.click);
        if (!hit)
            return;
        pushUndo(*r);
        const glm::vec3 local(transformPoint(glm::inverse(s->transform), hit->point));
        const crown::BrushMode mode = brush_ == 1 ? crown::BrushMode::Add : brush_ == 2 ? crown::BrushMode::Remove : crown::BrushMode::Smooth;
        crown::applyBrush(*r->crown, r->displacement, local, brushRadius_, brushStrength_, mode);
        regenerate(app, *r, distanceMap_);
        app.markModified();
    }

    OverlayFn overlay(DesignerApp& app, ViewId view) override
    {
        if (view != ViewId::Main3D)
            return {};
        // The gizmo sits on the selected connector (set before the view draws).
        View3D& v = app.view3D(ViewId::Main3D);
        v.gizmoPivot.reset();
        if (auto [b, i] = selected(app); b && moveInView_ && brush_ == 0) {
            RestorationDesign* first = findRestoration(app, b->teeth.front());
            if (ScanObject* s = first ? scanById(app, first->prepScanId) : nullptr)
                v.gizmoPivot = transformPoint(s->transform, b->connectors[static_cast<std::size_t>(i)].center);
        }
        if (brush_ == 0)
            return {};
        return [](ImDrawList* dl, const Projector& proj) {
            const ImVec2 m = ImGui::GetIO().MousePos;
            if (proj.contains(m))
                dl->AddCircle(m, 14.0f * ImGui::GetStyle().FontScaleDpi, IM_COL32(255, 150, 40, 220), 32, 1.5f);
        };
    }

private:
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
        ImGui::Spacing();
        if (b.result) {
            if (b.result->ok) {
                ImGui::TextColored(b.result->watertight ? pal.success : pal.danger, b.result->watertight ? "Merged, watertight" : "Merged, NOT watertight");
                ui::mutedText("Volume %.0f mm3, %zu triangles", b.result->volume, b.result->mesh.triangleCount());
                for (std::size_t i = 0; i < b.result->connectorAreas.size() && i + 1 < b.teeth.size(); ++i) {
                    const double a = b.result->connectorAreas[i];
                    ImGui::TextColored(a + 1e-3 >= b.connectorArea ? pal.success : pal.warning, "%s-%s", app.toothText(b.teeth[i]).c_str(), app.toothText(b.teeth[i + 1]).c_str());
                    ImGui::SameLine();
                    ui::mutedText("connector %.1f mm2", a);
                }
            } else {
                ImGui::TextColored(pal.danger, "Merge failed: %s", b.result->error.c_str());
            }
        } else {
            ui::mutedText("Not merged yet (merged automatically when saving).");
        }
        if (ui::button("Merge bridge", ImVec2(full, 0), !busy && !b.connectors.empty() && !b.result))
            startBridgeMerge(app, b);
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

    // Runs once the data and the scan analysis are available: designs new crowns automatically.
    void prepare(DesignerApp& app)
    {
        if (app.loading() || restorationsPending(app))
            return;
        RestorationDesign* active = app.doc().active();
        if (!active || !active->supported() || !ensureBase(app, *active))
            return;
        std::set<std::string> bridgesQueued;
        for (auto& r : app.doc().restorations) {
            if (!r.supported() || !ensureBase(app, r))
                continue;
            BridgeDesign* b = app.doc().bridgeOf(r.tooth);
            if (!r.crownGenerated) {
                // One design task per bridge or single crown, run one after another.
                if (b && !bridgesQueued.insert(b->label()).second)
                    continue;
                const int tooth = r.tooth;
                const bool map = distanceMap_;
                app.enqueue([&app, tooth, map] { startAutoDesign(app, tooth, map); });
            } else if (!r.crown) {
                regenerate(app, r, distanceMap_);
            } else {
                updateCrownOverlay(app, r, distanceMap_);
            }
        }
        for (auto& b : app.doc().bridges)
            if (b.connectors.empty())
                bridgeChanged(app, b);
        setAntagonistVisible(app, false);
        viewOcclusal(app, *active, 35.0);
        ready_ = true;
    }

    bool ready_ = false;

    void pushUndo(const RestorationDesign& r)
    {
        undo_.emplace_back(r.params, r.displacement);
        if (undo_.size() > 30)
            undo_.erase(undo_.begin());
    }

    void adapt(DesignerApp& app, RestorationDesign& r, bool proximal, bool occlusal)
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
        app.tasks().start("Adapting contacts", [&app, base, contacts, p, disp, tooth, proximal, occlusal, map](const ProgressFn& progress) mutable
                          -> ui::TaskRunner::Continuation {
            reportProgress(progress, 0.2f, "Adapting the crown surface");
            crown::adaptContacts(*base, p, *contacts, disp, proximal, occlusal);
            return [&app, disp, tooth, map] {
                if (RestorationDesign* r = findRestoration(app, tooth)) {
                    r->displacement = disp;
                    regenerate(app, *r, map);
                    app.markModified();
                }
            };
        });
    }

    bool moveInView_ = true;
    bool gizmoWasUsing_ = false;
    std::vector<std::pair<std::string, std::vector<crown::ConnectorEdit>>> connectorUndo_;
    int brush_ = 0;
    float brushRadius_ = 1.5f;
    float brushStrength_ = 0.1f;
    bool distanceMap_ = true;
    std::vector<std::pair<crown::CrownParameters, std::vector<float>>> undo_;
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
std::unique_ptr<Step> makeCrownDesignStep()
{
    return std::make_unique<CrownDesignStep>();
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
                r.crownGenerated = saved.crownGenerated;
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
        if (r.crownGenerated && !r.crown && (r.isPontic() || r.marginClosed()) && prepScanFor(app, r.prepScanId) && !app.tasks().busy())
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
        s.crownGenerated = r.crownGenerated;
        if (r.crownGenerated)
            s.crownFile = app.doc().bridgeOf(r.tooth) ? "design/bridge_" + bridgeName(app, *app.doc().bridgeOf(r.tooth)) + ".stl"
                                                       : "design/crown_" + app.toothText(r.tooth) + ".stl";
        out.push_back(std::move(s));
    }
    return out;
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
    for (const auto& r : app.doc().restorations) {
        if (!r.crown || !r.crownGenerated || app.doc().bridgeOf(r.tooth) || r.isPontic())
            continue;
        auto mesh = std::make_shared<Mesh>(r.crown->mesh);
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
        if (r.crownGenerated && !r.crown)
            return false;
    return true;
}

bool isGeneratedRestorationFile(const std::string& rel)
{
    auto starts = [&](std::string_view p) { return rel.rfind(p, 0) == 0; };
    return (starts("design/crown_") || starts("design/bridge_")) && rel.size() > 4 && rel.compare(rel.size() - 4, 4, ".stl") == 0;
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
        app.goToStep(StepId::CrownDesign); // starts the automatic design
        demo.state = 4;
        demo.waitFrames = 3;
        return true;
    case 4: {
        for (const auto& u : doc.restorations) {
            if (!u.supported())
                continue;
            if (!u.crown) {
                log::error("DEMO design of tooth {} failed: {}", u.tooth, u.dieError);
                return finish(3);
            }
            log::info("DEMO {} {} ({}): {} triangles, volume {:.0f} mm3, min thickness {:.2f} mm, {}", u.isPontic() ? "pontic" : "crown", u.tooth,
                      u.params.library.empty() ? crown::ToothLibraryRegistry::kDefaultLibrary : u.params.library, u.crown->mesh.triangleCount(),
                      u.crown->volume, u.crown->minThickness, u.crown->watertight ? "watertight" : "NOT watertight");
            if (!u.crown->watertight)
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
            b.selectedConnector = 0;
            bridgeChanged(app, b);
            const auto units = bridgeUnits(app, b);
            b.result = mergeBridge(units, b.connectors);
            const double measured = b.result->ok && !b.result->connectorAreas.empty() ? b.result->connectorAreas[0] : 0.0;
            log::info("DEMO connector {}-{} edited to {:.1f} mm2 (+0.3 mm occlusal): merged {}, measured {:.1f} mm2{}", b.teeth[0], b.teeth[1], e.area,
                      b.result->watertight ? "watertight" : "NOT watertight", measured, b.warnings[0].empty() ? "" : " - " + b.warnings[0]);
            if (!b.result->ok || !b.result->watertight || measured + 1e-3 < e.area)
                return finish(3);
        }
        if (demo.save && app.caseMode())
            app.saveDesign(true);
        demo.state = 5;
        demo.waitFrames = 2;
        return true;
    }
    default:
        return finish(0);
    }
}

} // namespace occlusa::designer
