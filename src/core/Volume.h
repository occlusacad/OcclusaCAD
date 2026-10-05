#pragma once

#include "core/Math.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace occlusa {

// Voxel grid geometry in patient (world) space.
//   world = origin + direction * (spacing * voxelIndex)
// direction columns are the unit vectors of the i, j and k voxel axes.
struct VolumeGeometry {
    glm::ivec3 dims{0};
    glm::dvec3 spacing{1.0};
    glm::dvec3 origin{0.0}; // world position of voxel (0,0,0) centre
    glm::dmat3 direction{1.0};

    glm::dmat4 voxelToWorld() const;
    glm::dmat4 worldToVoxel() const;
    // Maps world to normalized texture coordinates where voxel centres sit at (i+0.5)/dim.
    glm::dmat4 worldToTexture() const;

    // World-space bounding box of the voxel centres' hull expanded by half a voxel.
    Aabb worldBounds() const;
    glm::dvec3 worldCenter() const;
    std::size_t voxelCount() const { return static_cast<std::size_t>(dims.x) * dims.y * dims.z; }
};

struct VolumeInfo {
    std::string patientName;
    std::string patientId;
    std::string studyDate;
    std::string studyDescription;
    std::string seriesDescription;
    std::string seriesInstanceUid;
    std::string modality;
    std::string manufacturer;
    std::optional<double> windowCenter;
    std::optional<double> windowWidth;
};

// Scalar volume (CT / CBCT). Stored as int16 "stored values"; real-world value
// (HU for calibrated CT, arbitrary for many CBCT units) = stored * slope + intercept.
class Volume {
public:
    VolumeGeometry geometry;
    VolumeInfo info;
    double rescaleSlope = 1.0;
    double rescaleIntercept = 0.0;
    std::vector<std::int16_t> voxels; // x fastest, then y, then z

    bool empty() const { return voxels.empty(); }

    std::size_t index(int i, int j, int k) const
    {
        return (static_cast<std::size_t>(k) * geometry.dims.y + static_cast<std::size_t>(j)) * geometry.dims.x + static_cast<std::size_t>(i);
    }
    std::int16_t stored(int i, int j, int k) const { return voxels[index(i, j, k)]; }
    double value(int i, int j, int k) const { return stored(i, j, k) * rescaleSlope + rescaleIntercept; }

    // Trilinear interpolation in voxel coordinates (clamped to the grid). Returns real-world value.
    double sampleVoxel(const glm::dvec3& v) const;
    double sampleWorld(const glm::dvec3& w) const;
    bool containsWorld(const glm::dvec3& w) const;

    // Real-world min/max (computed lazily and cached).
    std::pair<double, double> valueRange() const;
    // Histogram of real-world values; bins span valueRange().
    std::vector<std::uint64_t> histogram(int bins) const;

    // Suggest an iso threshold for hard tissue (bone/teeth) using Otsu's method above air.
    double suggestBoneThreshold() const;
    // Suggest a display window (center, width) from header or histogram percentiles.
    std::pair<double, double> suggestWindow() const;

    // Cast a ray in world space and return the first point where the value crosses `iso`.
    std::optional<glm::dvec3> raycastIso(const Ray& worldRay, double iso, double maxDistance = 1e9) const;

private:
    mutable std::optional<std::pair<double, double>> cachedRange_;
};

} // namespace occlusa
