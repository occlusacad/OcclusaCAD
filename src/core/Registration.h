#pragma once

#include "core/KdTree.h"
#include "core/Mesh.h"
#include "core/Progress.h"

#include <optional>
#include <vector>

namespace occlusa {

// ---------------------------------------------------------------------------
// Point-pair (landmark) registration
// ---------------------------------------------------------------------------

struct PointPairResult {
    glm::dmat4 transform{1.0}; // maps source points onto target points
    double rms = 0.0;          // residual of the landmark pairs (mm)
    std::vector<double> residuals;
};

// Least-squares rigid transform (rotation + translation, no scale) mapping `source[i]`
// to `target[i]` using Horn's closed-form quaternion method. Requires >= 3 non-collinear pairs.
std::optional<PointPairResult> rigidFromPointPairs(const std::vector<glm::dvec3>& source, const std::vector<glm::dvec3>& target);

// ---------------------------------------------------------------------------
// Surface refinement (ICP)
// ---------------------------------------------------------------------------

// Target surface for ICP: points + normals in world space (e.g. a hard-tissue iso-surface).
struct IcpTarget {
    std::vector<glm::vec3> normals; // indexed like tree.points()
    KdTree tree;

    static IcpTarget fromMesh(const Mesh& worldMesh);
    bool empty() const { return tree.empty(); }
};

struct IcpOptions {
    int maxIterations = 60;
    double maxCorrespondenceDistance = 2.0; // mm; pairs further apart are ignored (first stage)
    // Coarse-to-fine: after converging, the search distance is halved until it reaches this value.
    // Rejects systematic near-misses (e.g. gingiva on the scan lying close to bone in the CBCT).
    double finalCorrespondenceDistance = 0.5;
    double trimFraction = 0.85;             // keep this fraction of the closest pairs (robust to gingiva etc.)
    double convergenceTranslation = 1e-4;   // mm per iteration
    double convergenceRotationDeg = 1e-3;   // degrees per iteration
    bool pointToPlane = true;
    std::size_t maxSourcePoints = 30000;
    // Restrict source samples to this world-space box (e.g. teeth region). Ignored when invalid.
    Aabb sourceRegion;
};

struct IcpResult {
    glm::dmat4 transform{1.0};
    double rms = 0.0;           // RMS distance of inlier pairs at the final search distance (mm)
    double meanDistance = 0.0;  // mean distance of inlier pairs (mm)
    double inlierFraction = 0.0;
    int iterations = 0;
    bool converged = false;
};

// Refine `initial` (source local -> world) so the source surface fits the target surface.
IcpResult refineIcp(const Mesh& source, const glm::dmat4& initial, const IcpTarget& target, const IcpOptions& options,
                    const ProgressFn& progress = {});

struct DeviationStats {
    double mean = 0.0;
    double rms = 0.0;
    double median = 0.0;
    double p90 = 0.0;
    double fractionWithin = 0.0; // fraction of sampled points within `tolerance`
    std::size_t samples = 0;
};

// Distances from transformed source samples to the nearest target point.
DeviationStats measureDeviation(const Mesh& source, const glm::dmat4& transform, const IcpTarget& target, double tolerance = 0.5,
                                double maxDistance = 5.0, std::size_t maxSamples = 20000);

} // namespace occlusa
