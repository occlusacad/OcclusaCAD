#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <limits>

namespace occlusa {

// World space throughout OcclusaCAD is the DICOM patient coordinate system (LPS, millimetres):
//   +X = patient left, +Y = patient posterior, +Z = patient superior.
// Volumes are never transformed; surface scans carry a model matrix into this space.

struct Aabb {
    glm::dvec3 min{std::numeric_limits<double>::max()};
    glm::dvec3 max{std::numeric_limits<double>::lowest()};

    bool valid() const { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }
    void expand(const glm::dvec3& p)
    {
        min = glm::min(min, p);
        max = glm::max(max, p);
    }
    void expand(const Aabb& other)
    {
        if (other.valid()) {
            expand(other.min);
            expand(other.max);
        }
    }
    glm::dvec3 center() const { return (min + max) * 0.5; }
    glm::dvec3 size() const { return max - min; }
    double diagonal() const { return valid() ? glm::length(max - min) : 0.0; }

    std::array<glm::dvec3, 8> corners() const
    {
        return {glm::dvec3{min.x, min.y, min.z}, glm::dvec3{max.x, min.y, min.z}, glm::dvec3{min.x, max.y, min.z},
                glm::dvec3{max.x, max.y, min.z}, glm::dvec3{min.x, min.y, max.z}, glm::dvec3{max.x, min.y, max.z},
                glm::dvec3{min.x, max.y, max.z}, glm::dvec3{max.x, max.y, max.z}};
    }

    Aabb transformed(const glm::dmat4& m) const
    {
        Aabb out;
        if (!valid())
            return out;
        for (const auto& c : corners())
            out.expand(glm::dvec3(m * glm::dvec4(c, 1.0)));
        return out;
    }
};

struct Ray {
    glm::dvec3 origin;
    glm::dvec3 direction; // normalized

    glm::dvec3 at(double t) const { return origin + direction * t; }
};

struct Plane {
    glm::dvec3 point;
    glm::dvec3 normal; // normalized

    double signedDistance(const glm::dvec3& p) const { return glm::dot(p - point, normal); }
};

inline glm::dvec3 transformPoint(const glm::dmat4& m, const glm::dvec3& p)
{
    return glm::dvec3(m * glm::dvec4(p, 1.0));
}

inline glm::dvec3 transformVector(const glm::dmat4& m, const glm::dvec3& v)
{
    return glm::dvec3(m * glm::dvec4(v, 0.0));
}

// Ray/box slab test. Returns false if no intersection; tNear may be negative when the origin is inside.
inline bool intersectRayAabb(const Ray& ray, const Aabb& box, double& tNear, double& tFar)
{
    tNear = -std::numeric_limits<double>::max();
    tFar = std::numeric_limits<double>::max();
    for (int i = 0; i < 3; ++i) {
        if (std::abs(ray.direction[i]) < 1e-12) {
            if (ray.origin[i] < box.min[i] || ray.origin[i] > box.max[i])
                return false;
            continue;
        }
        double t0 = (box.min[i] - ray.origin[i]) / ray.direction[i];
        double t1 = (box.max[i] - ray.origin[i]) / ray.direction[i];
        if (t0 > t1)
            std::swap(t0, t1);
        tNear = std::max(tNear, t0);
        tFar = std::min(tFar, t1);
        if (tNear > tFar)
            return false;
    }
    return tFar >= 0.0;
}

// Rigid transform helpers.
inline bool isRigid(const glm::dmat4& m, double tol = 1e-6)
{
    const glm::dmat3 r(m);
    const glm::dmat3 rtR = glm::transpose(r) * r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (std::abs(rtR[i][j] - (i == j ? 1.0 : 0.0)) > tol)
                return false;
    return glm::determinant(r) > 0.0;
}

// Re-orthonormalize the rotation part of a rigid transform (guards against drift after many incremental updates).
inline glm::dmat4 orthonormalize(const glm::dmat4& m)
{
    glm::dquat q = glm::quat_cast(glm::dmat3(m));
    q = glm::normalize(q);
    glm::dmat4 out = glm::mat4_cast(q);
    out[3] = glm::dvec4(glm::dvec3(m[3]), 1.0);
    return out;
}

} // namespace occlusa
