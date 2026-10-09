#pragma once

#include "core/Mesh.h"
#include "core/Progress.h"

#include <optional>

namespace occlusa::implant {

// Finds the implant position from a scanned scan body.
//
// The user clicks on the top of the scan body in the scan. The library scan body (implant frame)
// is placed there along the surface normal in several rotations about its axis, each refined by
// ICP against the scan around the click; the best fit wins. All coordinates are those of the scan.
struct ScanBodyFitResult {
    glm::dmat4 implantToScan{1.0};
    double rms = 0.0;            // of the inlier pairs (mm)
    double meanDistance = 0.0;
    double inlierFraction = 0.0; // of the visible scan body samples
    double fractionWithin = 0.0; // samples within 0.05 mm of the scan
    bool ok = false;
};

std::optional<ScanBodyFitResult> fitScanBody(const Mesh& scan, const Mesh& scanBody, const glm::dvec3& clickPoint, const ProgressFn& progress = {});

// Refine an existing placement (after a manual correction).
ScanBodyFitResult refineScanBody(const Mesh& scan, const Mesh& scanBody, const glm::dmat4& implantToScan);

// Points (with normals) spread evenly over a mesh's surface, about `spacing` apart.
Mesh sampleSurface(const Mesh& mesh, double spacing);

} // namespace occlusa::implant
