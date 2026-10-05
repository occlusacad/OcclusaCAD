#pragma once

#include "apps/designer/Document.h"
#include "core/Geometry.h"
#include "gfx/Camera.h"
#include "gfx/GlObjects.h"
#include "gfx/Renderers.h"

#include <imgui.h>

#include <functional>
#include <map>
#include <optional>
#include <string>

namespace occlusa::designer {

// Maps world coordinates to screen pixels for overlay drawing.
struct Projector {
    glm::dmat4 viewProj{1.0};
    ImVec2 min{0, 0};
    ImVec2 max{0, 0};

    std::optional<ImVec2> operator()(const glm::dvec3& world) const;
    bool contains(const ImVec2& p) const { return p.x >= min.x && p.y >= min.y && p.x <= max.x && p.y <= max.y; }
};

struct ViewEvents {
    bool hovered = false;
    std::optional<Ray> click;     // left click without drag (world ray)
    std::optional<glm::dvec3> clickWorld; // slice views: clicked point on the plane
    Projector projector;
};

using OverlayFn = std::function<void(ImDrawList*, const Projector&)>;

struct RenderServices {
    gfx::SceneRenderer& renderer;
    Document& doc;
};

// ---------------------------------------------------------------------------
// 3D view
// ---------------------------------------------------------------------------

enum class ViewContent { Combined, ScanOnly, VolumeOnly };

class View3D {
public:
    explicit View3D(std::string title, ViewContent content = ViewContent::Combined);

    std::string title;
    ViewContent content;
    int scanFilter = 0;          // ScanOnly: scan id to show (in its own coordinates)
    gfx::Camera camera;
    bool pickCursor = false;     // show a crosshair cursor (pick mode)
    int gizmoScan = 0;           // scan id manipulated with the gizmo (0 = none)
    std::optional<glm::dvec3> gizmoPivot; // or: translate-only gizmo at this world point (e.g. a bridge connector)
    int gizmoOperation = 0;      // 0 translate, 1 rotate
    std::optional<glm::dmat4> gizmoDelta; // set when the gizmo moved this frame

    // Draw into a region of the current window.
    ViewEvents draw(RenderServices& rs, const ImVec2& size, const OverlayFn& overlay = {});
    void requestFit() { needsFit_ = true; }
    // Frame a world-space box on the next draw (keeps the current viewing direction).
    void focus(const Aabb& box) { focusBox_ = box; }
    void setPreset(gfx::ViewPreset preset);

private:
    Aabb contentBounds(const Document& doc) const;
    std::size_t stateHash(const Document& doc, int w, int h) const;

    gfx::RenderTarget target_;
    bool needsFit_ = true;
    std::optional<Aabb> focusBox_;
    bool interacting_ = false;
    std::size_t lastHash_ = 0;
    std::uint64_t lastDocRevision_ = ~0ull;
};

// ---------------------------------------------------------------------------
// MPR slice view
// ---------------------------------------------------------------------------

enum class SliceOrientation { Axial, Coronal, Sagittal };
const char* toString(SliceOrientation o);

class SliceView {
public:
    explicit SliceView(SliceOrientation orientation);

    SliceOrientation orientation;
    double zoomHeight = 0.0;     // visible height in mm (0 = fit)
    glm::dvec2 pan{0.0};         // in-plane offset from the volume centre (mm)
    bool showContours = true;

    ViewEvents draw(RenderServices& rs, const ImVec2& size, const OverlayFn& overlay = {});
    Plane plane(const Document& doc) const;
    void requestFit() { zoomHeight = 0.0; pan = glm::dvec2(0.0); }

    // Camera axes for this orientation (radiological conventions).
    glm::dvec3 viewForward() const;
    glm::dvec3 viewUp() const;
    glm::dvec3 viewRight() const { return glm::cross(viewForward(), viewUp()); }

private:
    gfx::Camera makeCamera(const Document& doc, double aspect) const;
    const std::vector<LineSegment>& contours(const ScanObject& scan, const Plane& plane);

    gfx::RenderTarget target_;
    std::size_t lastHash_ = 0;
    struct ContourCache {
        std::uint64_t revision = ~0ull;
        double offset = 1e300;
        std::vector<LineSegment> segments;
    };
    std::map<int, ContourCache> contourCache_;
};

ImU32 orientationColor(SliceOrientation o);

} // namespace occlusa::designer
