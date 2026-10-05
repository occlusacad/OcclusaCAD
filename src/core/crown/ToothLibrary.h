#pragma once

#include <string_view>
#include <vector>

namespace occlusa::crown {

// Parametric tooth anatomy (an in-house library: no third-party tooth shapes are bundled).
// Each template describes the crown in a local frame: u runs mesial (+) / distal (-), v runs
// buccal or labial (+) / lingual or palatal (-), both normalised to [-1, 1] over the occlusal table.

enum class ToothKind { CentralIncisor, LateralIncisor, Canine, FirstPremolar, SecondPremolar, FirstMolar, SecondMolar };

struct Cusp {
    double u, v;      // position on the occlusal table
    double height;    // mm above the occlusal base
    double radius;    // normalised footprint radius (Gaussian sigma)
};

struct ToothTemplate {
    ToothKind kind;
    const char* name;
    double mesioDistal;   // mm at the height of contour
    double buccoLingual;  // mm
    double crownHeight;   // mm, margin to the highest cusp tip
    double contourHeight; // height of contour as a fraction of the crown height
    double squareness;    // superellipse exponent of the outline (2 = ellipse)
    double tableMD;       // occlusal table / incisal edge width relative to the contour (mesio-distal)
    double tableBL;       // ... (bucco-lingual); small for incisors (blade shape)
    double fossaDepth;    // mm below the cusp base at the centre
    double grooveDepth;   // central fissure depth, mm
    std::vector<Cusp> cusps;
};

ToothKind toothKindFromFdi(int fdi);
const ToothTemplate& toothTemplate(ToothKind kind, bool upper);
std::string_view toString(ToothKind kind);

// Occlusal surface height (mm, relative to the occlusal base) at normalised table coordinates.
// `cuspScale` exaggerates or flattens the anatomy (1 = template).
double occlusalHeight(const ToothTemplate& t, double u, double v, double cuspScale);

// Highest point of the occlusal surface relative to the base (the cusp tip height).
double occlusalMaxHeight(const ToothTemplate& t, double cuspScale);

// Superellipse outline radius factor (0..1] in direction (cos a, sin a) for half-axes 1/1.
double outlineFactor(double cosA, double sinA, double exponent);

} // namespace occlusa::crown
