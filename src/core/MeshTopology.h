#pragma once

#include "core/Mesh.h"

#include <cstdint>
#include <span>
#include <vector>

namespace occlusa {

// Vertex-to-vertex adjacency (compressed rows) of a triangle mesh.
class MeshAdjacency {
public:
    MeshAdjacency() = default;
    explicit MeshAdjacency(const Mesh& mesh);

    std::span<const std::uint32_t> neighbors(std::uint32_t v) const
    {
        return {neighbors_.data() + offsets_[v], neighbors_.data() + offsets_[v + 1]};
    }
    std::size_t vertexCount() const { return offsets_.empty() ? 0 : offsets_.size() - 1; }

private:
    std::vector<std::uint32_t> offsets_;
    std::vector<std::uint32_t> neighbors_;
};

// Signed curvature estimate per vertex (1/mm): the mean normal curvature towards the 1-ring
// neighbours. Positive on convex features (ridges, cusp tips, preparation margins), negative
// in concave ones (fissures, the inner angle of a shoulder). Requires outward vertex normals.
std::vector<float> vertexConvexity(const Mesh& mesh, const MeshAdjacency& adjacency, int smoothingIterations = 1);

// Shortest path along mesh edges from `from` to `to` where each edge costs its length times
// the mean of its endpoint costs (`vertexCost` empty = pure edge length). Returns the vertex
// sequence including both ends, or empty if unreachable within `maxCost`.
std::vector<std::uint32_t> shortestPath(const Mesh& mesh, const MeshAdjacency& adjacency, std::span<const float> vertexCost,
                                        std::uint32_t from, std::uint32_t to, float maxCost = 1e30f);

// Graph distance (sum of edge lengths) from `source` to every vertex up to `maxDistance`
// (others are +infinity).
std::vector<float> geodesicDistances(const Mesh& mesh, const MeshAdjacency& adjacency, std::uint32_t source, float maxDistance);

// Every undirected edge is shared by exactly two triangles with opposite orientation.
bool isClosedManifold(const Mesh& mesh);

// Number of edges used by only one triangle.
std::size_t boundaryEdgeCount(const Mesh& mesh);

// Signed enclosed volume (positive for a closed mesh with outward normals).
double signedVolume(const Mesh& mesh);

double surfaceArea(const Mesh& mesh);

} // namespace occlusa
