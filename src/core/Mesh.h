#pragma once

#include "core/Math.h"

#include <cstdint>
#include <vector>

namespace occlusa {

// Indexed triangle mesh in local (file) coordinates. Single precision is ample for
// dental geometry (sub-micron at a 100 mm extent).
struct Mesh {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals; // per vertex, same size as positions (may be empty)
    std::vector<std::uint32_t> indices; // 3 per triangle
    std::vector<glm::vec3> colors;      // optional per-vertex colours (display only, not saved to STL)

    std::size_t vertexCount() const { return positions.size(); }
    std::size_t triangleCount() const { return indices.size() / 3; }
    bool empty() const { return indices.empty(); }

    Aabb bounds() const;
    glm::dvec3 centroid() const; // area weighted surface centroid

    // Area weighted vertex normals.
    void computeVertexNormals();

    // Merge vertices closer than `tolerance` (STL files store every triangle corner separately).
    void weldVertices(float tolerance = 1e-5f);

    // Remove triangles with repeated indices or zero area.
    void removeDegenerateTriangles();
};

// Uniformly sample up to `maxCount` vertices (deterministic stride). Returns vertex indices.
std::vector<std::uint32_t> subsampleVertices(const Mesh& mesh, std::size_t maxCount);

} // namespace occlusa
