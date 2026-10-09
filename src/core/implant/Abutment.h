#pragma once

#include "core/Mesh.h"
#include "core/implant/ImplantLibrary.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace occlusa::implant {

// ---------------------------------------------------------------------------
// Custom abutment shape
// ---------------------------------------------------------------------------
//
// Everything is in the implant frame (z along the implant axis). Heights are measured from the
// top circle of the library interface, which the designed part sits on.
//
//   emergence profile: from the interface's top circle up to the margin, a smooth surface
//     controlled by 9 margin points (radius, height) and 9 mid points halfway up. A mid point
//     moves along the normal of the straight line from the interface to its margin point:
//     0 = straight (conical), > 0 convex, < 0 concave.
//   shoulder: from the margin inwards by `shoulderWidth`.
//   core (the top cap the crown sits on): rises from the shoulder to its top outline. While
//     `coreLocked`, the top outline follows the margin points (height above the margin,
//     taper); unlocked, each of its 9 points is placed freely.
//   screw channel: along the implant axis.
//
// Control point k sits at azimuth `phase + k * 40 degrees`; between them the values are
// interpolated with a closed Catmull-Rom spline, so the surface is smooth all round.

constexpr int kControlPoints = 9;

struct AbutmentShape {
    std::array<double, kControlPoints> marginRadius{};
    std::array<double, kControlPoints> marginHeight{};
    std::array<double, kControlPoints> midOffset{};

    double shoulderWidth = 0.5;
    double coreHeight = 5.0;    // above the margin (locked core)
    double taperDeg = 6.0;      // per side (locked core)
    bool coreLocked = true;
    std::array<double, kControlPoints> coreRadius{}; // top outline when unlocked
    std::array<double, kControlPoints> coreTop{};    // its height above the interface top

    double screwChannelDiameter = 2.4;
    double phase = 0.0;         // azimuth of control point 0 (radians, implant frame)

    double azimuth(int k) const;
    // Core top outline point k (radius, height), locked or not.
    glm::dvec2 corePoint(int k, double minCoreRadius) const;
    // Copy the locked core outline into coreRadius/coreTop (before unlocking).
    void unlockCore(double minCoreRadius);
};

// Default shape on a connection: margin circle 0.5 mm wider (in diameter) than the interface's
// top circle, `marginHeight` above it, straight emergence.
AbutmentShape defaultAbutmentShape(const Connection& connection, double marginHeight = 1.5);

// Smallest core radius that keeps the minimum wall around the screw channel.
double minCoreRadius(const AbutmentShape& shape, const Connection& connection);

// Closed periodic Catmull-Rom interpolation of 9 control values at an azimuth.
double interpolateControls(const std::array<double, kControlPoints>& values, double azimuth, double phase);

struct AbutmentGeometry {
    Mesh designed;               // the designed part (implant frame), closed, with the screw channel
    std::size_t outerRowEnd = 0; // vertices [0, outerRowEnd) are on the outside (emergence to top)
    std::vector<float> wall;     // per vertex: radial wall thickness to the screw channel (outer vertices)
    double minWall = 0.0;        // smallest wall above the interface
    std::array<glm::dvec3, kControlPoints> marginPoints{}, midPoints{}, corePoints{}; // implant frame
    std::vector<std::string> warnings;
};

AbutmentGeometry buildAbutment(const AbutmentShape& shape, const Connection& connection);

// The finished abutment: the designed part united with the unchanged interface (implant frame).
// When the interface is not a closed solid, the two are combined without a union and `merged`
// is false. Also checks the optional library geometry (minimum thickness, blank).
struct AbutmentSolid {
    Mesh mesh;
    bool merged = false;
    std::string error;
    std::vector<std::string> warnings;
};
AbutmentSolid finishAbutment(const AbutmentGeometry& geometry, const AbutmentShape& shape, const Connection& connection);

// Point-in-solid test by ray parity (closed meshes).
bool insideMesh(const Mesh& closedMesh, const glm::vec3& p);

} // namespace occlusa::implant
