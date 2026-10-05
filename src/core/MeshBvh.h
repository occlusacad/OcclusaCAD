#pragma once

#include "core/Mesh.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace occlusa {

struct BvhRayHit {
    float t = 0.0f;              // distance along the (normalized) ray direction
    std::uint32_t triangle = 0;
    glm::vec3 point{0.0f};
    glm::vec3 faceNormal{0.0f};  // unit, following the triangle winding
};

struct BvhClosestPoint {
    float distance = 0.0f;       // unsigned
    float signedDistance = 0.0f; // negative behind the surface (using interpolated vertex normals)
    std::uint32_t triangle = 0;
    glm::vec3 point{0.0f};
    glm::vec3 normal{0.0f};      // interpolated vertex normal (face normal if the mesh has none)
};

// Bounding volume hierarchy over a triangle mesh, in the mesh's own coordinates. Used for
// ray casts and closest-point / signed-distance queries against scans (contacts, thickness).
// The mesh is shared, so the BVH stays valid as long as it exists. Queries are thread safe.
class MeshBvh {
public:
    MeshBvh() = default;
    explicit MeshBvh(std::shared_ptr<const Mesh> mesh);

    bool empty() const { return !mesh_ || nodes_.empty(); }
    const Mesh& mesh() const { return *mesh_; }
    std::shared_ptr<const Mesh> meshPtr() const { return mesh_; }

    std::optional<BvhRayHit> raycast(const glm::vec3& origin, const glm::vec3& direction, float tMax = 1e30f) const;
    std::optional<BvhClosestPoint> closestPoint(const glm::vec3& p, float maxDistance) const;

private:
    struct Node {
        glm::vec3 bmin, bmax;
        std::uint32_t first = 0; // leaf: first triangle in order_; inner: right child index
        std::uint32_t count = 0; // leaf triangle count (0 = inner node, left child = this + 1)
    };
    std::uint32_t build(std::uint32_t begin, std::uint32_t end, std::vector<glm::vec3>& centroids);
    glm::vec3 vertex(std::uint32_t tri, int k) const { return mesh_->positions[mesh_->indices[3 * tri + k]]; }

    std::shared_ptr<const Mesh> mesh_;
    std::vector<Node> nodes_;
    std::vector<std::uint32_t> order_;
};

// Closest point on triangle (a, b, c) to p, with barycentric weights of the result.
glm::vec3 closestPointOnTriangle(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, glm::vec3* barycentric = nullptr);

} // namespace occlusa
