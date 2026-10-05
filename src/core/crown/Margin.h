#pragma once

#include "core/KdTree.h"
#include "core/Mesh.h"
#include "core/MeshBvh.h"
#include "core/MeshTopology.h"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace occlusa::crown {

// Analysis of the scan that contains a preparation: topology, curvature, spatial indices.
// Built once per scan (it takes a moment on full-arch scans) and shared by all restorations.
class PrepScan {
public:
    explicit PrepScan(std::shared_ptr<const Mesh> mesh);

    const Mesh& mesh() const { return *mesh_; }
    std::shared_ptr<const Mesh> meshPtr() const { return mesh_; }
    const MeshAdjacency& adjacency() const { return adjacency_; }
    const std::vector<float>& convexity() const { return convexity_; }
    // Edge cost factor for margin tracing: low on strongly convex vertices, so paths follow the margin ridge.
    const std::vector<float>& marginCost() const { return marginCost_; }
    float convexityScale() const { return convexityScale_; }
    const MeshBvh& bvh() const { return bvh_; }

    std::uint32_t nearestVertex(const glm::vec3& p) const;

private:
    std::shared_ptr<const Mesh> mesh_;
    MeshAdjacency adjacency_;
    std::vector<float> convexity_;
    std::vector<float> marginCost_;
    float convexityScale_ = 1.0f;
    KdTree vertexTree_;
    MeshBvh bvh_;
};

// Margin line: a loop of scan vertices where consecutive vertices share a mesh edge.
struct MarginLine {
    std::vector<std::uint32_t> vertices;
    bool closed = false;

    bool empty() const { return vertices.empty(); }
    std::vector<glm::vec3> points(const Mesh& mesh) const;
};

// Connect control vertices with curvature-weighted shortest paths (closed: also last -> first).
// Back-tracking detours that would make the loop touch itself are removed.
MarginLine traceMargin(const PrepScan& scan, const std::vector<std::uint32_t>& controls, bool closed);

struct MarginDetectOptions {
    float searchRadius = 7.5f; // mm from the preparation axis
    float maxDepth = 9.0f;     // mm below the clicked point
    int bins = 72;             // angular samples around the preparation
    int controlCount = 24;     // editable control points of the result
};

struct MarginDetection {
    std::vector<std::uint32_t> controls;
    MarginLine line;
    glm::dvec3 occlusalDirection{0.0}; // unit, from the margin towards the preparation top
    int binsFound = 0;
    int bins = 0;
};

// Detect the preparation margin from a point on the preparation (ideally its occlusal surface).
// Walking down the surface from that point in every direction, the margin is the first strong
// convex edge after the inner (concave) angle of the chamfer/shoulder. Outliers are rejected
// against their angular neighbours and the result is traced along the convex ridge.
std::optional<MarginDetection> detectMargin(const PrepScan& scan, const glm::vec3& pointOnPrep, const MarginDetectOptions& options = {},
                                            std::string* error = nullptr);

// The part of the scan inside a closed margin: the die surface the crown sits on.
struct DieRegion {
    Mesh mesh;
    std::vector<std::uint32_t> scanVertex; // die vertex -> scan vertex
    std::vector<std::uint32_t> boundary;   // die vertex indices along the margin, in margin order
};

// Cut the scan along the margin and keep the side containing `insidePoint` (a point on the preparation).
std::optional<DieRegion> extractDie(const PrepScan& scan, const MarginLine& margin, const glm::vec3& insidePoint, std::string* error = nullptr);

// Geometry helpers.
glm::dvec3 vectorArea(const std::vector<glm::vec3>& loop); // 0.5 * sum p_i x p_i+1
glm::dvec3 centroid(const std::vector<glm::vec3>& points);
double polylineLength(const std::vector<glm::vec3>& points, bool closed);
std::vector<glm::vec3> smoothPolyline(std::vector<glm::vec3> points, bool closed, int iterations);

} // namespace occlusa::crown
