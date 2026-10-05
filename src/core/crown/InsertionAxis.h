#pragma once

#include "core/Mesh.h"

#include <vector>

namespace occlusa::crown {

// Undercuts of a die relative to an insertion direction. A surface point is in an undercut
// when material further along the axis (towards occlusal) sticks out beyond it, i.e. the
// crown could not slide over it. Computed in cylindrical coordinates around the axis, which
// suits single preparations (star-shaped cross sections).
struct Blockout {
    std::vector<glm::vec3> positions; // die vertices moved radially out of the undercuts
    std::vector<float> depth;         // per vertex, mm
};

Blockout computeBlockout(const Mesh& die, const glm::dvec3& axis, const glm::dvec3& center);

struct UndercutReport {
    double undercutArea = 0.0; // mm^2 of die surface that needs blocking out (depth > 0.01 mm)
    double dieArea = 0.0;
    double maxDepth = 0.0;     // mm
};

UndercutReport undercutReport(const Mesh& die, const Blockout& blockout);

// Direction minimising undercuts within `maxTiltDeg` of `initial`, preferring small tilts.
glm::dvec3 optimizeInsertionAxis(const Mesh& die, const glm::dvec3& initial, double maxTiltDeg = 25.0);

// Rotate `axis` by two tilt angles about perpendicular directions `u` and `v` (degrees).
glm::dvec3 tiltAxis(const glm::dvec3& axis, const glm::dvec3& u, const glm::dvec3& v, double tiltUDeg, double tiltVDeg);

// Orthonormal frame around an axis.
void perpendicularBasis(const glm::dvec3& axis, glm::dvec3& u, glm::dvec3& v);

} // namespace occlusa::crown
