#include "apps/designer/Views.h"

#include "ui/Fonts.h"
#include "ui/Theme.h"

#include <ImGuizmo.h>

#include <cmath>
#include <cstring>
#include <format>
#include <type_traits>

namespace occlusa::designer {

namespace {

// FNV-1a over arbitrary trivially copyable values: cheap change detection for re-rendering.
class Hasher {
public:
    template <class T>
    Hasher& add(const T& v)
    {
        static_assert(std::is_trivially_copyable_v<T>);
        const auto* p = reinterpret_cast<const unsigned char*>(&v);
        for (std::size_t i = 0; i < sizeof(T); ++i) {
            h_ ^= p[i];
            h_ *= 1099511628211ull;
        }
        return *this;
    }
    std::size_t value() const { return static_cast<std::size_t>(h_); }

private:
    std::uint64_t h_ = 1469598103934665603ull;
};

void hashCamera(Hasher& h, const gfx::Camera& c)
{
    h.add(c.target).add(c.distance).add(c.orientation).add(c.orthoHeight).add(c.projection).add(c.fovYDegrees).add(c.sceneRadius);
}

void hashDisplay(Hasher& h, const VolumeDisplay& d)
{
    h.add(d.windowCenter).add(d.windowWidth).add(d.isoValue).add(d.mode).add(d.color).add(d.opacity).add(d.cropMin).add(d.cropMax).add(d.visible)
        .add(d.showThresholdOnSlices);
}

ImTextureID textureId(GLuint tex)
{
    return static_cast<ImTextureID>(static_cast<std::uintptr_t>(tex));
}

void drawLabel(ImDrawList* dl, ImVec2 pos, const char* text, ImU32 color)
{
    const ImVec2 ts = ImGui::CalcTextSize(text);
    const float pad = 4.0f * ImGui::GetStyle().FontScaleDpi;
    dl->AddRectFilled(ImVec2(pos.x - pad, pos.y - pad * 0.5f), ImVec2(pos.x + ts.x + pad, pos.y + ts.y + pad * 0.5f), IM_COL32(0, 0, 0, 110), 4.0f);
    dl->AddText(pos, color, text);
}

char directionLetter(const glm::dvec3& v)
{
    const glm::dvec3 a = glm::abs(v);
    if (a.x >= a.y && a.x >= a.z)
        return v.x > 0 ? 'L' : 'R';
    if (a.y >= a.z)
        return v.y > 0 ? 'P' : 'A';
    return v.z > 0 ? 'S' : 'I';
}

void drawAxisTriad(ImDrawList* dl, const gfx::Camera& cam, ImVec2 origin, float len)
{
    const glm::dmat3 view = glm::transpose(glm::dmat3(glm::mat3_cast(cam.orientation)));
    struct Axis {
        glm::dvec3 dir;
        ImU32 color;
        const char* label;
    };
    const Axis axes[3] = {{{1, 0, 0}, IM_COL32(230, 80, 80, 255), "L"},
                          {{0, 1, 0}, IM_COL32(90, 200, 110, 255), "P"},
                          {{0, 0, 1}, IM_COL32(90, 150, 240, 255), "S"}};
    for (const auto& a : axes) {
        const glm::dvec3 v = view * a.dir;
        const ImVec2 end(origin.x + static_cast<float>(v.x) * len, origin.y - static_cast<float>(v.y) * len);
        dl->AddLine(origin, end, a.color, 2.0f);
        dl->AddText(ImVec2(end.x - 3, end.y - 7), a.color, a.label);
    }
}

} // namespace

std::optional<ImVec2> Projector::operator()(const glm::dvec3& world) const
{
    const glm::dvec4 clip = viewProj * glm::dvec4(world, 1.0);
    if (clip.w <= 1e-9)
        return std::nullopt;
    const glm::dvec3 ndc = glm::dvec3(clip) / clip.w;
    return ImVec2(min.x + static_cast<float>((ndc.x * 0.5 + 0.5) * (max.x - min.x)),
                  min.y + static_cast<float>((0.5 - ndc.y * 0.5) * (max.y - min.y)));
}

ImU32 orientationColor(SliceOrientation o)
{
    switch (o) {
    case SliceOrientation::Axial: return IM_COL32(80, 150, 255, 255);
    case SliceOrientation::Coronal: return IM_COL32(80, 210, 120, 255);
    case SliceOrientation::Sagittal: return IM_COL32(240, 90, 90, 255);
    }
    return IM_COL32_WHITE;
}

const char* toString(SliceOrientation o)
{
    switch (o) {
    case SliceOrientation::Axial: return "Axial";
    case SliceOrientation::Coronal: return "Coronal";
    case SliceOrientation::Sagittal: return "Sagittal";
    }
    return "";
}

// ---------------------------------------------------------------------------
// View3D
// ---------------------------------------------------------------------------

View3D::View3D(std::string t, ViewContent c) : title(std::move(t)), content(c)
{
    gfx::applyViewPreset(camera, gfx::ViewPreset::Front);
}

void View3D::setPreset(gfx::ViewPreset preset)
{
    gfx::applyViewPreset(camera, preset);
    needsFit_ = true;
}

Aabb View3D::contentBounds(const Document& doc) const
{
    Aabb box;
    if (content == ViewContent::ScanOnly) {
        for (const auto& s : doc.scans)
            if (s->id == scanFilter && s->mesh)
                box = s->mesh->bounds();
        return box;
    }
    if (doc.volume && content != ViewContent::ScanOnly) {
        // Prefer the scans' region when present: the CBCT field of view is much larger than the arch.
        box.expand(doc.volume->volume->geometry.worldBounds());
    }
    if (content == ViewContent::Combined)
        for (const auto& s : doc.scans)
            if (s->visible)
                box.expand(s->worldBounds());
    return box;
}

std::size_t View3D::stateHash(const Document& doc, int w, int h) const
{
    Hasher hs;
    hashCamera(hs, camera);
    hs.add(w).add(h).add(content).add(scanFilter).add(interacting_).add(doc.revision()).add(ui::palette().dark);
    if (doc.volume)
        hashDisplay(hs, doc.volume->display);
    for (const auto& s : doc.scans)
        hs.add(s->id).add(s->revision).add(s->visible).add(s->opacity).add(s->color);
    return hs.value();
}

ViewEvents View3D::draw(RenderServices& rs, const ImVec2& sizeIn, const OverlayFn& overlay)
{
    ViewEvents ev;
    Document& doc = rs.doc;
    ImGuiIO& io = ImGui::GetIO();
    const ImVec2 size(std::max(sizeIn.x, 32.0f), std::max(sizeIn.y, 32.0f));
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const ImVec2 end(pos.x + size.x, pos.y + size.y);
    ImGui::PushID(this);
    ImGui::InvisibleButton("##view3d", size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    ev.hovered = hovered;
    const double aspect = static_cast<double>(size.x) / size.y;

    const Aabb bounds = contentBounds(doc);
    if (bounds.valid())
        camera.sceneRadius = std::max(bounds.diagonal() * 0.5, 10.0);
    if (focusBox_) {
        camera.fit(*focusBox_, aspect);
        focusBox_.reset();
        needsFit_ = false;
    } else if (needsFit_ && bounds.valid()) {
        // Frame the scans if there are any (the region of interest), otherwise the whole volume.
        Aabb focus;
        if (content == ViewContent::Combined)
            for (const auto& s : doc.scans)
                if (s->visible)
                    focus.expand(s->worldBounds());
        camera.fit(focus.valid() ? focus : bounds, aspect);
        needsFit_ = false;
    }

    // Camera interaction (unless the gizmo owns the mouse).
    const bool gizmoBusy = (gizmoScan != 0 || gizmoPivot) && (ImGuizmo::IsUsing() || ImGuizmo::IsOver());
    interacting_ = false;
    if (active && !gizmoBusy) {
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 2.0f) && !blockOrbit) {
            camera.orbit(io.MouseDelta.x, io.MouseDelta.y, size.y);
            interacting_ = true;
        }
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Right, 2.0f) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 2.0f)) {
            camera.pan(io.MouseDelta.x, io.MouseDelta.y, size.y);
            interacting_ = true;
        }
    }
    if (hovered && io.MouseWheel != 0.0f) {
        // Zoom towards the point under the mouse.
        const double ndcX = (io.MousePos.x - pos.x) / size.x * 2.0 - 1.0;
        const double ndcY = 1.0 - (io.MousePos.y - pos.y) / size.y * 2.0;
        const double factor = std::pow(0.85, io.MouseWheel);
        const double wpp = camera.worldPerPixel(size.y);
        const glm::dvec3 offset = camera.right() * (ndcX * size.x * 0.5 * wpp) + camera.up() * (ndcY * size.y * 0.5 * wpp);
        camera.target += offset * (1.0 - factor);
        camera.zoom(factor);
    }
    if (hovered && !gizmoBusy && !blockOrbit && ImGui::IsMouseReleased(ImGuiMouseButton_Left) && io.MouseDragMaxDistanceSqr[0] < 16.0f) {
        const double ndcX = (io.MousePos.x - pos.x) / size.x * 2.0 - 1.0;
        const double ndcY = 1.0 - (io.MousePos.y - pos.y) / size.y * 2.0;
        ev.click = camera.rayFromNdc(ndcX, ndcY, aspect);
    }
    if (hovered && pickCursor)
        ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);

    // Render when anything changed.
    const int w = static_cast<int>(size.x * io.DisplayFramebufferScale.x);
    const int h = static_cast<int>(size.y * io.DisplayFramebufferScale.y);
    const std::size_t hash = stateHash(doc, w, h);
    if (hash != lastHash_ || !target_.valid() || target_.width() != w || target_.height() != h) {
        target_.resize(w, h, 4);
        const gfx::FrameContext ctx = gfx::makeFrameContext(camera, w, h);
        std::vector<gfx::MeshDraw> meshes;
        for (const auto& s : doc.scans) {
            if (!s->gpu)
                continue;
            if (content == ViewContent::VolumeOnly)
                continue;
            if (content == ViewContent::ScanOnly && s->id != scanFilter)
                continue;
            if (content == ViewContent::Combined && (!s->visible || s->stepHidden))
                continue;
            gfx::MeshDraw md;
            md.mesh = s->gpu.get();
            md.model = content == ViewContent::ScanOnly ? glm::mat4(1.0f) : glm::mat4(s->transform);
            md.color = s->color;
            md.opacity = content == ViewContent::ScanOnly ? 1.0f : s->opacity;
            md.highlight = gizmoScan == s->id;
            meshes.push_back(md);
        }
        if (content == ViewContent::Combined) {
            for (const auto& [key, o] : doc.overlays) {
                if (!o.gpu || !o.visible)
                    continue;
                gfx::MeshDraw md;
                md.mesh = o.gpu.get();
                if (o.scanId != 0) {
                    const ScanObject* scan = nullptr;
                    for (const auto& s : doc.scans)
                        if (s->id == o.scanId)
                            scan = s.get();
                    if (!scan)
                        continue;
                    md.model = glm::mat4(scan->transform);
                }
                md.color = o.color;
                md.opacity = o.opacity;
                md.vertexColors = o.vertexColors;
                md.depthBias = o.depthBias;
                meshes.push_back(md);
            }
        }
        std::optional<gfx::VolumeDraw> vd;
        if (doc.volume && doc.volume->gpu && content != ViewContent::ScanOnly && (doc.volume->display.visible || content == ViewContent::VolumeOnly)) {
            const VolumeDisplay& d = doc.volume->display;
            gfx::VolumeDraw v;
            v.volume = doc.volume->gpu.get();
            v.mode = content == ViewContent::VolumeOnly ? gfx::VolumeMode::Isosurface : d.mode;
            v.isoValue = d.isoValue;
            v.windowCenter = d.windowCenter;
            v.windowWidth = d.windowWidth;
            v.color = d.color;
            v.opacity = d.opacity;
            v.clipMin = d.cropMin;
            v.clipMax = d.cropMax;
            v.stepVoxels = interacting_ ? 1.2f : 0.5f;
            vd = v;
        }
        const ui::Palette& pal = ui::palette();
        rs.renderer.render3D(target_, ctx, meshes, vd ? &*vd : nullptr, pal.viewportTop, pal.viewportBottom);
        lastHash_ = hash;
    }

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddImage(textureId(target_.colorTexture()), pos, end, ImVec2(0, 1), ImVec2(1, 0));

    ev.projector.viewProj = camera.projectionMatrix(aspect) * camera.viewMatrix();
    ev.projector.min = pos;
    ev.projector.max = end;

    dl->PushClipRect(pos, end, true);
    if (overlay)
        overlay(dl, ev.projector);
    const float s = ImGui::GetStyle().FontScaleDpi;
    drawLabel(dl, ImVec2(pos.x + 10 * s, pos.y + 8 * s), title.c_str(), IM_COL32(235, 238, 242, 255));
    if (content != ViewContent::ScanOnly)
        drawAxisTriad(dl, camera, ImVec2(pos.x + 34 * s, end.y - 34 * s), 22.0f * s);
    dl->PopClipRect();

    // Gizmo for manual alignment (applied as a world-space delta about the scan centre).
    gizmoDelta.reset();
    if (gizmoPivot && gizmoScan == 0 && content == ViewContent::Combined) {
        ImGuizmo::PushID(this);
        ImGuizmo::SetDrawlist(dl);
        ImGuizmo::SetRect(pos.x, pos.y, size.x, size.y);
        ImGuizmo::SetOrthographic(camera.projection == gfx::Camera::Projection::Orthographic);
        const glm::mat4 view(camera.viewMatrix());
        const glm::mat4 proj(camera.projectionMatrix(aspect));
        glm::mat4 pivot = glm::translate(glm::mat4(1.0f), glm::vec3(*gizmoPivot));
        glm::mat4 delta(1.0f);
        if (ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj), ImGuizmo::TRANSLATE, ImGuizmo::WORLD, glm::value_ptr(pivot), glm::value_ptr(delta)))
            gizmoDelta = glm::dmat4(delta);
        ImGuizmo::PopID();
    }
    if (gizmoScan != 0 && content == ViewContent::Combined) {
        if (ScanObject* scan = doc.findScan(gizmoScan); scan && scan->mesh) {
            ImGuizmo::PushID(this);
            ImGuizmo::SetDrawlist(dl);
            ImGuizmo::SetRect(pos.x, pos.y, size.x, size.y);
            ImGuizmo::SetOrthographic(camera.projection == gfx::Camera::Projection::Orthographic);
            const glm::mat4 view(camera.viewMatrix());
            const glm::mat4 proj(camera.projectionMatrix(aspect));
            glm::mat4 pivot = glm::translate(glm::mat4(1.0f), glm::vec3(scan->worldBounds().center()));
            glm::mat4 delta(1.0f);
            const ImGuizmo::OPERATION op = gizmoOperation == 0 ? ImGuizmo::TRANSLATE : ImGuizmo::ROTATE;
            if (ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj), op, ImGuizmo::WORLD, glm::value_ptr(pivot), glm::value_ptr(delta)))
                gizmoDelta = glm::dmat4(delta);
            ImGuizmo::PopID();
        }
    }
    ImGui::PopID();
    return ev;
}

// ---------------------------------------------------------------------------
// SliceView
// ---------------------------------------------------------------------------

SliceView::SliceView(SliceOrientation o) : orientation(o) {}

glm::dvec3 SliceView::viewForward() const
{
    switch (orientation) {
    case SliceOrientation::Axial: return {0, 0, 1};     // seen from the feet
    case SliceOrientation::Coronal: return {0, 1, 0};   // seen from the front
    case SliceOrientation::Sagittal: return {-1, 0, 0}; // seen from the patient's left
    }
    return {0, 0, 1};
}

glm::dvec3 SliceView::viewUp() const
{
    return orientation == SliceOrientation::Axial ? glm::dvec3(0, -1, 0) : glm::dvec3(0, 0, 1);
}

Plane SliceView::plane(const Document& doc) const
{
    glm::dvec3 n = orientation == SliceOrientation::Axial ? glm::dvec3(0, 0, 1)
                 : orientation == SliceOrientation::Coronal ? glm::dvec3(0, 1, 0)
                                                            : glm::dvec3(1, 0, 0);
    return Plane{doc.cursor, n};
}

gfx::Camera SliceView::makeCamera(const Document& doc, double aspect) const
{
    gfx::Camera cam;
    cam.projection = gfx::Camera::Projection::Orthographic;
    cam.setView(viewForward(), viewUp());
    const Aabb wb = doc.volume->volume->geometry.worldBounds();
    const glm::dvec3 right = viewRight(), up = viewUp(), fwd = viewForward();
    // In-plane extent of the volume.
    double rMin = 1e300, rMax = -1e300, uMin = 1e300, uMax = -1e300;
    for (const auto& c : wb.corners()) {
        const double r = glm::dot(c - doc.cursor, right), u = glm::dot(c - doc.cursor, up);
        rMin = std::min(rMin, r);
        rMax = std::max(rMax, r);
        uMin = std::min(uMin, u);
        uMax = std::max(uMax, u);
    }
    const glm::dvec3 center = doc.cursor + right * (0.5 * (rMin + rMax) + pan.x) + up * (0.5 * (uMin + uMax) + pan.y);
    cam.target = center;
    cam.sceneRadius = std::max(wb.diagonal(), 10.0);
    cam.distance = cam.sceneRadius;
    const double fitH = std::max(uMax - uMin, (rMax - rMin) / std::max(aspect, 0.1)) * 1.04;
    cam.orthoHeight = zoomHeight > 0.0 ? zoomHeight : fitH;
    (void)fwd;
    return cam;
}

const std::vector<LineSegment>& SliceView::contours(const ScanObject& scan, const Plane& pl)
{
    ContourCache& c = contourCache_[scan.id];
    const double offset = glm::dot(pl.point, pl.normal);
    if (c.revision != scan.revision || std::abs(c.offset - offset) > 1e-6) {
        c.segments = slicePlane(*scan.mesh, scan.transform, pl);
        c.revision = scan.revision;
        c.offset = offset;
    }
    return c.segments;
}

ViewEvents SliceView::draw(RenderServices& rs, const ImVec2& sizeIn, const OverlayFn& overlay)
{
    ViewEvents ev;
    Document& doc = rs.doc;
    ImGuiIO& io = ImGui::GetIO();
    const ImVec2 size(std::max(sizeIn.x, 32.0f), std::max(sizeIn.y, 32.0f));
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const ImVec2 end(pos.x + size.x, pos.y + size.y);
    const ui::Palette& pal = ui::palette();
    ImGui::PushID(this);
    ImGui::InvisibleButton("##slice", size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    ev.hovered = hovered;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float s = ImGui::GetStyle().FontScaleDpi;

    if (!doc.volume || !doc.volume->gpu) {
        dl->AddRectFilled(pos, end, ImGui::ColorConvertFloat4ToU32(ImVec4(pal.sliceBackground.r, pal.sliceBackground.g, pal.sliceBackground.b, 1.0f)));
        drawLabel(dl, ImVec2(pos.x + 10 * s, pos.y + 8 * s), toString(orientation), orientationColor(orientation));
        const char* msg = "No CBCT loaded";
        const ImVec2 ts = ImGui::CalcTextSize(msg);
        dl->AddText(ImVec2(pos.x + (size.x - ts.x) * 0.5f, pos.y + (size.y - ts.y) * 0.5f), IM_COL32(150, 155, 162, 255), msg);
        ImGui::PopID();
        return ev;
    }

    VolumeObject& vo = *doc.volume;
    const Volume& vol = *vo.volume;
    const double aspect = static_cast<double>(size.x) / size.y;
    const Aabb wb = vol.geometry.worldBounds();
    const double sliceStep = std::min({vol.geometry.spacing.x, vol.geometry.spacing.y, vol.geometry.spacing.z});

    // Interaction.
    gfx::Camera cam = makeCamera(doc, aspect);
    const auto mouseToPlane = [&](ImVec2 m) {
        const double ndcX = (m.x - pos.x) / size.x * 2.0 - 1.0;
        const double ndcY = 1.0 - (m.y - pos.y) / size.y * 2.0;
        const Ray r = cam.rayFromNdc(ndcX, ndcY, aspect);
        const Plane pl = plane(doc);
        const double denom = glm::dot(r.direction, pl.normal);
        const double t = std::abs(denom) > 1e-12 ? glm::dot(pl.point - r.origin, pl.normal) / denom : 0.0;
        return r.at(t);
    };
    if (active && ImGui::IsMouseDown(ImGuiMouseButton_Left) && !io.KeyShift) {
        glm::dvec3 p = mouseToPlane(io.MousePos);
        doc.cursor = glm::clamp(p, wb.min, wb.max);
    }
    if (active && (ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 1.0f) || (io.KeyShift && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 1.0f)))) {
        const double wpp = cam.orthoHeight / size.y;
        pan.x -= io.MouseDelta.x * wpp;
        pan.y += io.MouseDelta.y * wpp;
        if (zoomHeight <= 0.0)
            zoomHeight = cam.orthoHeight;
    }
    if (active && ImGui::IsMouseDragging(ImGuiMouseButton_Right, 1.0f)) {
        const auto [lo, hi] = vol.valueRange();
        const double scale = (hi - lo) / 600.0;
        vo.display.windowWidth = std::clamp(vo.display.windowWidth + io.MouseDelta.x * scale, 1.0, (hi - lo) * 2.0);
        vo.display.windowCenter = std::clamp(vo.display.windowCenter - io.MouseDelta.y * scale, lo, hi);
    }
    if (hovered && io.MouseWheel != 0.0f) {
        if (io.KeyCtrl) {
            if (zoomHeight <= 0.0)
                zoomHeight = cam.orthoHeight;
            zoomHeight = std::clamp(zoomHeight * std::pow(0.85, io.MouseWheel), 5.0, 2000.0);
        } else {
            const Plane pl = plane(doc);
            const double stepMm = sliceStep * (io.KeyShift ? 5.0 : 1.0) * (io.MouseWheel > 0 ? 1.0 : -1.0);
            doc.cursor = glm::clamp(doc.cursor + pl.normal * stepMm, wb.min, wb.max);
        }
    }
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
        requestFit();
    if (hovered && ImGui::IsMouseReleased(ImGuiMouseButton_Left) && io.MouseDragMaxDistanceSqr[0] < 16.0f)
        ev.clickWorld = mouseToPlane(io.MousePos);
    cam = makeCamera(doc, aspect);

    // Render.
    const Plane pl = plane(doc);
    const glm::dvec3 right = viewRight(), up = viewUp();
    double rMin = 1e300, rMax = -1e300, uMin = 1e300, uMax = -1e300;
    for (const auto& c : wb.corners()) {
        const double r = glm::dot(c - doc.cursor, right), u = glm::dot(c - doc.cursor, up);
        rMin = std::min(rMin, r);
        rMax = std::max(rMax, r);
        uMin = std::min(uMin, u);
        uMax = std::max(uMax, u);
    }
    const int w = static_cast<int>(size.x * io.DisplayFramebufferScale.x);
    const int h = static_cast<int>(size.y * io.DisplayFramebufferScale.y);
    Hasher hs;
    hashCamera(hs, cam);
    hashDisplay(hs, vo.display);
    hs.add(w).add(h).add(doc.cursor).add(pal.dark);
    if (hs.value() != lastHash_ || !target_.valid() || target_.width() != w || target_.height() != h) {
        target_.resize(w, h, 1);
        gfx::SliceDraw sd;
        sd.volume = vo.gpu.get();
        sd.corners[0] = glm::vec3(doc.cursor + right * rMin + up * uMin);
        sd.corners[1] = glm::vec3(doc.cursor + right * rMax + up * uMin);
        sd.corners[2] = glm::vec3(doc.cursor + right * rMax + up * uMax);
        sd.corners[3] = glm::vec3(doc.cursor + right * rMin + up * uMax);
        sd.windowCenter = vo.display.windowCenter;
        sd.windowWidth = vo.display.windowWidth;
        sd.showThreshold = vo.display.showThresholdOnSlices;
        sd.isoValue = vo.display.isoValue;
        rs.renderer.renderSlice(target_, gfx::makeFrameContext(cam, w, h), sd, pal.sliceBackground);
        lastHash_ = hs.value();
    }
    dl->AddImage(textureId(target_.colorTexture()), pos, end, ImVec2(0, 1), ImVec2(1, 0));

    ev.projector.viewProj = cam.projectionMatrix(aspect) * cam.viewMatrix();
    ev.projector.min = pos;
    ev.projector.max = end;
    const Projector& proj = ev.projector;

    dl->PushClipRect(pos, end, true);
    // Crosshair: intersections with the other two planes.
    for (SliceOrientation other : {SliceOrientation::Axial, SliceOrientation::Coronal, SliceOrientation::Sagittal}) {
        if (other == orientation)
            continue;
        SliceView tmp(other);
        const glm::dvec3 dir = glm::normalize(glm::cross(pl.normal, tmp.plane(doc).normal));
        const auto a = proj(doc.cursor - dir * 1000.0), b = proj(doc.cursor + dir * 1000.0);
        if (a && b) {
            const ImU32 col = (orientationColor(other) & 0x00FFFFFF) | (170u << 24);
            dl->AddLine(*a, *b, col, 1.0f * s);
        }
    }
    // Scan outlines.
    if (showContours) {
        for (const auto& scan : doc.scans) {
            if (!scan->visible || !scan->mesh)
                continue;
            const ImU32 col = ImGui::ColorConvertFloat4ToU32(ImVec4(std::min(scan->color.r * 1.15f, 1.0f), std::min(scan->color.g * 1.15f, 1.0f),
                                                                    std::min(scan->color.b * 1.15f, 1.0f), 1.0f));
            for (const auto& seg : contours(*scan, pl)) {
                const auto a = proj(glm::dvec3(seg.a)), b = proj(glm::dvec3(seg.b));
                if (a && b)
                    dl->AddLine(*a, *b, col, 1.6f * s);
            }
        }
    }
    if (overlay)
        overlay(dl, proj);

    // Labels: title, orientation letters, slice position, value under the mouse.
    drawLabel(dl, ImVec2(pos.x + 10 * s, pos.y + 8 * s), toString(orientation), orientationColor(orientation));
    const ImU32 letterCol = IM_COL32(200, 205, 212, 220);
    const char left[2] = {directionLetter(-right), 0}, rightL[2] = {directionLetter(right), 0};
    const char top[2] = {directionLetter(up), 0}, bottom[2] = {directionLetter(-up), 0};
    const float fh = ImGui::GetFontSize();
    dl->AddText(ImVec2(pos.x + 8 * s, pos.y + size.y * 0.5f - fh * 0.5f), letterCol, left);
    dl->AddText(ImVec2(end.x - 16 * s, pos.y + size.y * 0.5f - fh * 0.5f), letterCol, rightL);
    dl->AddText(ImVec2(pos.x + size.x * 0.5f - 4 * s, pos.y + 6 * s), letterCol, top);
    dl->AddText(ImVec2(pos.x + size.x * 0.5f - 4 * s, end.y - fh - 6 * s), letterCol, bottom);
    const double offset = glm::dot(doc.cursor, pl.normal);
    const std::string posText = std::format("{} {:.1f} mm", orientation == SliceOrientation::Axial ? "Z" : orientation == SliceOrientation::Coronal ? "Y" : "X", offset);
    drawLabel(dl, ImVec2(pos.x + 10 * s, end.y - fh - 10 * s), posText.c_str(), IM_COL32(200, 205, 212, 255));
    if (hovered) {
        const glm::dvec3 p = mouseToPlane(io.MousePos);
        if (vol.containsWorld(p)) {
            const std::string v = std::format("{:.0f}", vol.sampleWorld(p));
            const ImVec2 ts = ImGui::CalcTextSize(v.c_str());
            drawLabel(dl, ImVec2(end.x - ts.x - 12 * s, end.y - fh - 10 * s), v.c_str(), IM_COL32(200, 205, 212, 255));
        }
    }
    dl->PopClipRect();
    ImGui::PopID();
    return ev;
}

} // namespace occlusa::designer
