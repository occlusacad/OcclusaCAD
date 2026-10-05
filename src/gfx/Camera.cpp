#include "gfx/Camera.h"

namespace occlusa::gfx {

glm::dmat4 Camera::viewMatrix() const
{
    return glm::lookAt(eye(), target, up());
}

glm::dmat4 Camera::projectionMatrix(double aspect) const
{
    aspect = std::max(aspect, 1e-3);
    const double r = std::max(sceneRadius, 1.0);
    if (projection == Projection::Orthographic) {
        const double h = orthoHeight * 0.5, w = h * aspect;
        return glm::ortho(-w, w, -h, h, distance - 4.0 * r, distance + 4.0 * r);
    }
    const double nearZ = std::max(distance - 2.5 * r, distance * 0.01);
    const double farZ = distance + 2.5 * r;
    return glm::perspective(glm::radians(fovYDegrees), aspect, nearZ, farZ);
}

void Camera::setView(const glm::dvec3& fwd, const glm::dvec3& upHint)
{
    const glm::dvec3 f = glm::normalize(fwd);
    glm::dvec3 u = upHint - f * glm::dot(upHint, f);
    if (glm::length(u) < 1e-9)
        u = std::abs(f.z) < 0.9 ? glm::dvec3(0, 0, 1) : glm::dvec3(0, 1, 0);
    u = glm::normalize(u);
    const glm::dvec3 r = glm::cross(f, u);
    // Columns: camera X (right), Y (up), Z (-forward).
    const glm::dmat3 m(r, u, -f);
    orientation = glm::normalize(glm::quat_cast(m));
}

void Camera::fit(const Aabb& box, double aspect)
{
    if (!box.valid())
        return;
    target = box.center();
    const double radius = std::max(box.diagonal() * 0.5, 1.0);
    sceneRadius = std::max(sceneRadius, radius);
    const double halfFov = glm::radians(fovYDegrees) * 0.5;
    const double fitH = radius / std::sin(halfFov);
    const double fitW = radius / std::sin(std::atan(std::tan(halfFov) * std::max(aspect, 0.1)));
    distance = std::max(fitH, fitW) * 1.05;
    orthoHeight = 2.0 * radius * 1.1 / std::min(1.0, std::max(aspect, 0.1));
}

void Camera::orbit(double dx, double dy, double viewportHeight)
{
    const double scale = glm::pi<double>() / std::max(viewportHeight, 1.0); // half turn per viewport height
    const glm::dquat yaw = glm::angleAxis(-dx * scale, up());
    const glm::dquat pitch = glm::angleAxis(-dy * scale, right());
    orientation = glm::normalize(yaw * pitch * orientation);
}

double Camera::worldPerPixel(double viewportHeight) const
{
    const double h = projection == Projection::Orthographic ? orthoHeight
                                                            : 2.0 * distance * std::tan(glm::radians(fovYDegrees) * 0.5);
    return h / std::max(viewportHeight, 1.0);
}

void Camera::pan(double dx, double dy, double viewportHeight)
{
    const double s = worldPerPixel(viewportHeight);
    target += -right() * (dx * s) + up() * (dy * s);
}

void Camera::zoom(double factor)
{
    factor = std::clamp(factor, 0.2, 5.0);
    if (projection == Projection::Orthographic)
        orthoHeight = std::clamp(orthoHeight * factor, 1.0, 5000.0);
    else
        distance = std::clamp(distance * factor, 5.0, 20000.0);
}

Ray Camera::rayFromNdc(double x, double y, double aspect) const
{
    const glm::dmat4 inv = glm::inverse(projectionMatrix(aspect) * viewMatrix());
    glm::dvec4 a = inv * glm::dvec4(x, y, -1.0, 1.0);
    glm::dvec4 b = inv * glm::dvec4(x, y, 1.0, 1.0);
    a /= a.w;
    b /= b.w;
    return Ray{glm::dvec3(a), glm::normalize(glm::dvec3(b - a))};
}

void applyViewPreset(Camera& camera, ViewPreset preset)
{
    // LPS: +X patient left, +Y posterior, +Z superior.
    switch (preset) {
    case ViewPreset::Front: camera.setView({0, 1, 0}, {0, 0, 1}); break;   // looking from anterior
    case ViewPreset::Back: camera.setView({0, -1, 0}, {0, 0, 1}); break;
    case ViewPreset::Left: camera.setView({-1, 0, 0}, {0, 0, 1}); break;   // looking at the patient's left side
    case ViewPreset::Right: camera.setView({1, 0, 0}, {0, 0, 1}); break;
    case ViewPreset::Top: camera.setView({0, 0, -1}, {0, -1, 0}); break;   // occlusal view of the lower jaw
    case ViewPreset::Bottom: camera.setView({0, 0, 1}, {0, -1, 0}); break; // occlusal view of the upper jaw
    }
}

} // namespace occlusa::gfx
