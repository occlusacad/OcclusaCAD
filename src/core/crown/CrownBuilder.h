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

    // Cement space on the intaglio (inner) surface.
    double cementGap = 0.03;       // from `distanceToMargin` upwards
    double extraGap = 0.04;        // added a further 1 mm above that
    double distanceToMargin = 0.8; // band along the margin that fits closely
    bool blockOutUndercuts = true;

    // Material thickness.
    double minThickness = 0.5;
    double marginThickness = 0.08;

    // Anatomy and placement (0 = take from the tooth template).
    double crownHeight = 0.0;      // margin plane to cusp tips
    double halfMesial = 0.0, halfDistal = 0.0, halfBuccal = 0.0, halfLingual = 0.0;
    double rotationDeg = 0.0;      // about the insertion axis
    double shiftMesial = 0.0, shiftBuccal = 0.0;
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
};

CrownBase makeCrownBase(DieRegion die, const CrownFrame& frame, const CrownParameters& params);

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

CrownMesh buildCrown(const CrownBase& base, const CrownParameters& params, std::span<const float> displacement = {});

// Automatic fitting.
void fitProximal(const CrownBase& base, const ContactScene& contacts, CrownParameters& params);
bool fitOcclusalHeight(const CrownBase& base, const ContactScene& contacts, CrownParameters& params, std::span<const float> displacement = {});

// Adapt the surface to the contact targets (pull back where too close / overlapping, extend
// facing areas within `reach` towards the target). Modifies `displacement` (one value per vertex).
void adaptContacts(const CrownBase& base, const CrownParameters& params, const ContactScene& contacts, std::vector<float>& displacement,
                   bool proximal, bool occlusal, double reach = 0.6);

// Signed distance of each vertex to the nearest contact geometry (NaN where nothing is within `maxDistance`).
std::vector<float> contactDistances(const CrownMesh& crown, const ContactScene& contacts, float maxDistance = 2.0f);

enum class BrushMode { Add, Remove, Smooth };
void applyBrush(const CrownMesh& crown, std::vector<float>& displacement, const glm::vec3& center, float radius, float strength, BrushMode mode);

// Thickness of the outer surface over the intaglio, per vertex (NaN for intaglio vertices).
std::vector<float> thicknessMap(const CrownBase& base, const CrownMesh& crown);

} // namespace occlusa::crown
