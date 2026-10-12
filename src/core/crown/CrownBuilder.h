#pragma once

#include "core/Mesh.h"
#include "core/MeshBvh.h"
#include "core/crown/Margin.h"
#include "core/crown/ToothLibrary.h"

#include <memory>
#include <optional>
#include <string>
#include <span>
#include <vector>

namespace occlusa::crown {

// Design parameters of one crown. Distances in mm.
struct CrownParameters {
    std::string library;           // tooth library id (empty = the default library)
    ToothKind kind = ToothKind::FirstMolar;
    bool upper = false;
    bool coping = false;           // uniform-thickness coping instead of full anatomy
    double copingThickness = 0.5;

    // Crown bottom (the inside of the crown), after exocad's "Crown Bottoms" dialog.
    // Gap: cement space on the intaglio surface.
    double cementGap = 0.08;       // gap zone ("Gap")
    double distanceToMargin = 1.0; // zone along the margin without cement gap ("From margin")
    double axialSpacing = 0.02;    // additional spacing along the insertion axis (gap zone)
    double radialSpacing = 0.02;   // additional spacing perpendicular to it (gap zone)
    // Border: the crown edge at the margin.
    double borderHorizontal = 0.2; // horizontal border width outwards from the margin
    double borderAngled = 0.3;     // length of the angled part after it
    double borderAngleDeg = 60.0;  // angle of the angled part (from the horizontal)
    double borderVertical = 0.0;   // additional vertical border
    double belowMargin = 0.0;      // offset of the margin along the insertion direction (+ = apical)
    // Advanced: undercuts and manufacturing.
    bool blockOutUndercuts = true;
    double blockoutAngleDeg = 0.0;   // draft of the blocked-out surface (0 = straight down)
    double blockoutUntouchable = 0.0; // zone along the margin that is never blocked out (mm)
    bool anticipateMilling = true;    // tool diameter compensation of the intaglio
    double toolDiameter = 1.2;        // smallest milling tool (choose slightly above the real one)
    bool bullnoseTool = false;
    double toolFlatPercent = 0.0;     // bullnose tool: 0 = round bur, 100 = flat end

    // Material thickness (the crown bottom is at least this thick everywhere).
    double minThickness = 0.5;

    // Anatomy and placement (0 = take from the tooth template).
    double crownHeight = 0.0;      // margin plane to cusp tips
    double halfMesial = 0.0, halfDistal = 0.0, halfBuccal = 0.0, halfLingual = 0.0;
    double rotationDeg = 0.0;      // about the insertion axis
    double tiltMesialDeg = 0.0;    // tilt of the tooth axis towards mesial
    double tiltBuccalDeg = 0.0;    // tilt of the tooth axis towards buccal
    double shiftMesial = 0.0, shiftBuccal = 0.0, shiftOcclusal = 0.0;
    double cuspScale = 1.0;
    bool flipMesioDistal = false;
    bool flipBuccoLingual = false;

    // Pontics: distance of the basal surface from the ridge (negative presses into the tissue).
    double ridgeOffset = 0.0;

    // Contact targets: signed distance to the neighbour / antagonist (negative = overlap).
    double proximalTarget = -0.03;
    double occlusalTarget = 0.05;
};

// Local frame of a restoration, in the coordinates of the scan containing the preparation.
struct CrownFrame {
    glm::dvec3 origin{0.0};  // margin centroid
    glm::dvec3 axis{0, 0, 1}; // insertion direction, pointing occlusally
    glm::dvec3 mesial{1, 0, 0};
    glm::dvec3 buccal{0, 1, 0};
};

// Resolved local axes of a crown (insertion axis A, mesial M, buccal B, footprint centre F),
// including the parameters' rotation, flips and shifts.
struct CrownAxes {
    glm::dvec3 A, M, B, F;
};
CrownAxes crownAxes(const CrownFrame& frame, const CrownParameters& params);

// Guess the mesial and buccal directions from the neighbouring teeth on the scan: they sit
// mesial and distal of the preparation, and the arch curves lingually.
void estimateToothOrientation(const Mesh& scan, const std::vector<glm::vec3>& margin, CrownFrame& frame);

// Everything that depends on the margin, the die, the axis and the cement settings.
struct CrownBase {
    DieRegion die;
    CrownFrame frame;
    std::vector<glm::vec3> margin;       // margin points (die boundary order)
    std::shared_ptr<const Mesh> intaglio; // die offset by the cement gap, undercuts blocked out (die topology)
    MeshBvh intaglioBvh;
    std::vector<float> blockoutDepth;    // per die vertex
    std::vector<float> marginDistance;   // per die vertex, mm to the margin
    std::vector<float> gap;              // per die vertex: cement gap + spacing applied (mm)
    std::vector<float> toolCompensation; // per die vertex: material removed for the milling tool (mm)
};

CrownBase makeCrownBase(DieRegion die, const CrownFrame& frame, const CrownParameters& params);

// The crown bottom as a closed solid: the intaglio, the border at the margin (horizontal, angled,
// vertical) and the minimum thickness over the intaglio. The finished crown is the union of the
// designed shell and this solid, minus the preparation space (see mergeCrown).
Mesh makeCrownBottom(const CrownBase& base, const CrownParameters& params);

// The library tooth placed by the parameters (size, rotation, tilt, shifts, cusp height) but not
// attached to the margin, in prep-scan coordinates. Tooth placement shows and moves this.
Mesh placedLibraryTooth(const CrownBase& base, const CrownParameters& params);

// Neighbouring geometry for contacts, in prep-scan coordinates.
struct ContactScene {
    std::shared_ptr<const MeshBvh> neighbors;  // the prep scan without the preparation
    std::shared_ptr<const MeshBvh> antagonist; // the opposing jaw (optional)
};

// `exclude`: cylinders (centre, radius) along the insertion axis whose scan surface is ignored,
// e.g. the other units of a bridge, which are joined rather than in contact.
ContactScene makeContactScene(const Mesh& prepScan, const CrownBase& base, const Mesh* antagonist, const glm::dmat4& antagonistToPrep,
                              const std::vector<std::pair<glm::dvec3, double>>& exclude = {});

// The restoration: intaglio (die vertices, first) plus an outer anatomic shell whose bottom ring
// is the margin itself, so the result is closed and watertight.
struct CrownMesh {
    Mesh mesh;                          // final geometry (with displacement and thickness enforcement)
    std::vector<glm::vec3> basePositions; // before displacement
    std::vector<glm::vec3> baseNormals;
    std::vector<float> editWeight;      // 0 = fixed (intaglio, margin), 1 = freely editable
    std::vector<float> requiredThickness;
    std::uint32_t outerBegin = 0;       // first outer-shell vertex
    int columns = 0, rings = 0;
    bool watertight = false;
    double volume = 0.0;                // mm^3
    double minThickness = 0.0;          // achieved, where the full minimum thickness is required
    int thickenedVertices = 0;          // pushed out to reach the minimum thickness
};

// `shell`: the zero-thickness anatomic shell adapted to the margin, closed over the die (no
// intaglio, no thickness enforcement): what the technician sculpts before merging. Otherwise the
// classic solid crown on the intaglio with the minimum thickness enforced.
CrownMesh buildCrown(const CrownBase& base, const CrownParameters& params, std::span<const float> displacement = {}, bool shell = false);

// The finished crown: (shell united with the crown bottom) minus the preparation space. The
// shell's die cap and anything below the margin are removed with the preparation space.
struct CrownMerge {
    Mesh mesh;
    bool ok = false;
    std::string error;
    bool watertight = false;
    double volume = 0.0;       // mm^3
    double minThickness = 0.0; // over the intaglio, away from the margin
};
CrownMerge mergeCrown(const CrownBase& base, const CrownMesh& shell, const Mesh& bottom);

// Automatic fitting.
void fitProximal(const CrownBase& base, const ContactScene& contacts, CrownParameters& params);
// `shell`: evaluate the zero-thickness shell (tooth placement, free-forming) instead of the classic solid.
bool fitOcclusalHeight(const CrownBase& base, const ContactScene& contacts, CrownParameters& params, std::span<const float> displacement = {},
                       bool shell = false);

// Adapt the surface to the contact targets (pull back where too close / overlapping, extend
// facing areas within `reach` towards the target). Modifies `displacement` (one value per vertex).
void adaptContacts(const CrownBase& base, const CrownParameters& params, const ContactScene& contacts, std::vector<float>& displacement,
                   bool proximal, bool occlusal, double reach = 0.6, bool shell = false);

// Signed distance of each vertex to the nearest contact geometry (NaN where nothing is within `maxDistance`).
std::vector<float> contactDistances(const CrownMesh& crown, const ContactScene& contacts, float maxDistance = 2.0f);

enum class BrushMode { Add, Remove, Smooth, Flatten };
void applyBrush(const CrownMesh& crown, std::vector<float>& displacement, const glm::vec3& center, float radius, float strength, BrushMode mode);

// Thickness of the outer surface over the intaglio, per vertex (NaN for intaglio vertices).
std::vector<float> thicknessMap(const CrownBase& base, const CrownMesh& crown);

} // namespace occlusa::crown
