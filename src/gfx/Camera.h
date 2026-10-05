#pragma once

#include "core/Math.h"

namespace occlusa::gfx {

// Orbit camera around a target point. Orientation is a free trackball (no fixed up axis),
// which suits inspecting dental models from any direction.
class Camera {
public:
    enum class Projection { Perspective, Orthographic };

    Projection projection = Projection::Perspective;
    glm::dvec3 target{0.0};
    double distance = 200.0;
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0}; // camera-to-world rotation (camera looks along -Z)
    double fovYDegrees = 30.0;
    double orthoHeight = 100.0; // visible height in mm (orthographic)
    double sceneRadius = 100.0; // for clip planes

    glm::dvec3 eye() const { return target - forward() * distance; }
    glm::dvec3 forward() const { return orientation * glm::dvec3(0, 0, -1); }
    glm::dvec3 up() const { return orientation * glm::dvec3(0, 1, 0); }
    glm::dvec3 right() const { return orientation * glm::dvec3(1, 0, 0); }

    glm::dmat4 viewMatrix() const;
    glm::dmat4 projectionMatrix(double aspect) const;

    // Look along `forward` with the given up direction (orthogonalized).
    void setView(const glm::dvec3& forward, const glm::dvec3& up);
    // Fit the camera so the box fills the view.
    void fit(const Aabb& box, double aspect);

    // Interaction (pixel deltas relative to viewport height).
    void orbit(double dxPixels, double dyPixels, double viewportHeight);
    void pan(double dxPixels, double dyPixels, double viewportHeight);
    void zoom(double factor); // < 1 zooms in

    // World-space ray through a point given in normalized device coordinates [-1, 1].
    Ray rayFromNdc(double ndcX, double ndcY, double aspect) const;
    // World units per pixel at the target depth.
    double worldPerPixel(double viewportHeight) const;
};

// Standard anatomical views in patient (LPS) coordinates.
enum class ViewPreset { Front, Back, Left, Right, Top, Bottom };
void applyViewPreset(Camera& camera, ViewPreset preset);

} // namespace occlusa::gfx
