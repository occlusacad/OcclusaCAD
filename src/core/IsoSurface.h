#pragma once

#include "core/Mesh.h"
#include "core/Progress.h"
#include "core/Volume.h"

namespace occlusa {

struct IsoSurfaceOptions {
    double isoValue = 500.0; // real-world units (HU for calibrated CT)
    int step = 1;            // sample every Nth voxel (2 = 8x fewer cells)
    // Optional region of interest in voxel indices (inclusive min, exclusive max). Empty = whole volume.
    glm::ivec3 roiMin{0};
    glm::ivec3 roiMax{0};
};

// Extract the iso-surface of a volume as a triangle mesh in world (patient) coordinates
// using naive surface nets (dual contouring without QEF). Produces watertight, well-shaped
// quads (split into triangles) without lookup tables. Normals point out of the hard tissue.
Mesh extractIsoSurface(const Volume& volume, const IsoSurfaceOptions& options, const ProgressFn& progress = {});

} // namespace occlusa
