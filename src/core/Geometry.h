#pragma once

#include "core/Mesh.h"

#include <optional>
#include <vector>

namespace occlusa {

struct MeshHit {
    double distance = 0.0;   // along the ray, world units
    glm::dvec3 point{0.0};   // world space
    glm::dvec3 normal{0.0};  // world space, facing the ray origin
    std::uint32_t triangle = 0;
};

// Closest intersection of a world-space ray with a mesh placed by `modelMatrix`.
std::optional<MeshHit> raycastMesh(const Mesh& mesh, const glm::dmat4& modelMatrix, const Ray& worldRay);

struct LineSegment {
    glm::vec3 a, b;
};

// Intersection of a transformed mesh with a world-space plane, as unordered line segments in world space.
std::vector<LineSegment> slicePlane(const Mesh& mesh, const glm::dmat4& modelMatrix, const Plane& plane);

// Tube along a polyline (e.g. a margin line), for depth-correct display.
Mesh makeTube(const std::vector<glm::vec3>& points, bool closed, float radius, int sides = 8);

// Small sphere (icosphere-like UV sphere) centred at `center`.
Mesh makeSphere(const glm::vec3& center, float radius, int segments = 12);

} // namespace occlusa
