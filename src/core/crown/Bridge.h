#pragma once

#include "core/Math.h"
#include "core/Mesh.h"
#include "core/MeshBvh.h"
#include "core/crown/CrownBuilder.h"

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace occlusa::crown {

// ---------------------------------------------------------------------------
// Bridge layout
// ---------------------------------------------------------------------------

// Position along the arch (0..15 per jaw, distal right to distal left) and adjacency.
int archIndex(int fdi);
bool adjacentTeeth(int a, int b);

bool isAbutmentType(const std::string& type); // crowns and copings on preparations
bool isPonticType(const std::string& type);

// Bridges are runs of adjacent teeth (same jaw) with restorations that contain at least one
// pontic and one abutment. Each result lists the teeth in arch order.
std::vector<std::vector<int>> findBridges(const std::vector<std::pair<int, std::string>>& restorations);

struct BridgeLayoutUnit {
    int tooth = 0;
    bool pontic = false;
    glm::dvec3 center{0.0}; // abutments: input (margin centroid); pontics: output
    glm::dvec3 mesial{0.0}; // abutments: estimated mesial direction (input)
};

// Place the pontics of a bridge (units in arch order): between abutments proportionally to
// template tooth widths, cantilevers next to their abutment along the arch. Returns false if
// the bridge has no abutment.
bool layoutPontics(std::vector<BridgeLayoutUnit>& units, const glm::dvec3& axis);

// Mesial direction for a pontic, from the neighbouring units (the one with the mesial tooth).
glm::dvec3 ponticMesial(const std::vector<BridgeLayoutUnit>& units, std::size_t index, const glm::dvec3& axis);

// One insertion direction for all abutments of a bridge (minimum combined undercut).
glm::dvec3 optimizeCommonAxis(const std::vector<const Mesh*>& dies, const glm::dvec3& initial, double maxTiltDeg = 25.0);

// Angle (degrees) between an abutment's own best axis and the common one: the divergence the
// preparations have to tolerate.
double axisDivergence(const glm::dvec3& a, const glm::dvec3& b);

// ---------------------------------------------------------------------------
// Pontic
// ---------------------------------------------------------------------------

// Base of a pontic: a patch of the gingiva (ridge) under the pontic, projected along the axis
// from an elliptical footprint and lifted by `ridgeOffset` (negative = pressing into the
// tissue). It plays the role of the die, so the crown builder produces a closed pontic whose
// basal surface follows the ridge (modified ridge lap / ovate).
std::optional<DieRegion> makePonticBase(const MeshBvh& scan, const glm::dvec3& center, const glm::dvec3& axis, const glm::dvec3& mesial,
                                        double footprintMD, double footprintBL, double ridgeOffset, std::string* error = nullptr);

// Crown parameters suitable for a pontic built on a ridge base.
CrownParameters ponticParameters(CrownParameters p);

// Make neighbouring units meet: the half widths facing a bridge neighbour are set from the
// distance between the unit centres plus `overlap`, so connectors join solid material.
struct BridgeFitUnit {
    CrownFrame frame;
    CrownParameters* params = nullptr;
};
void fitBridgeWidths(std::vector<BridgeFitUnit>& units, double overlap = 0.15);

// ---------------------------------------------------------------------------
// Connectors and union
// ---------------------------------------------------------------------------

struct ConnectorSpec {
    glm::dvec3 center{0.0};
    glm::dvec3 direction{1, 0, 0}; // from unit a towards unit b
    glm::dvec3 up{0, 0, 1};
    double length = 6.0;  // along `direction`
    double area = 9.0;    // mm^2 of the elliptical cross-section
    double heightRatio = 1.3; // occluso-gingival height / bucco-lingual width
    double height() const;
    double width() const;
};

// Place the connector between two adjacent units: midway between their footprint centres,
// above the gingival embrasure and below the marginal ridges. `bottom` is the lowest allowed
// connector edge (e.g. margin or ridge height + clearance) and `top` the highest, both as
// heights along `up` from `reference`. Returns a warning if the connector does not fit.
ConnectorSpec placeConnector(const glm::dvec3& centerA, const glm::dvec3& centerB, const glm::dvec3& up, const glm::dvec3& reference,
                             double bottom, double top, double area, double heightRatio, std::string* warning = nullptr);

// Manual adjustment of one connector relative to its automatic placement, so it survives changes
// to the units. Offsets are in the connector's own axes (see connectorFrame).
struct ConnectorEdit {
    glm::dvec3 offset{0.0};   // mm: x along the connector (mesio-distal), y bucco-lingual, z occlusal
    double area = 0.0;        // mm^2 (0 = the bridge's default)
    double heightRatio = 0.0; // occluso-gingival height / bucco-lingual width (0 = the bridge's default)
    double length = 0.0;      // mm (0 = automatic)
    bool isDefault() const { return offset == glm::dvec3(0.0) && area <= 0.0 && heightRatio <= 0.0 && length <= 0.0; }
};

// Axes of a connector: x along it, z towards occlusal (perpendicular to x), y = z x x (bucco-lingual).
void connectorFrame(const ConnectorSpec& c, glm::dvec3& x, glm::dvec3& y, glm::dvec3& z);
ConnectorSpec applyConnectorEdit(ConnectorSpec spec, const ConnectorEdit& edit);

// Does the connector stay between the gingival embrasure (`bottom`) and the marginal ridges (`top`),
// heights along `up` from `reference`? Returns a description of the problem otherwise.
std::optional<std::string> checkConnectorFit(const ConnectorSpec& c, const glm::dvec3& reference, double bottom, double top);

Mesh makeConnectorMesh(const ConnectorSpec& c, int segments = 24);

// Closed solid of the preparation space under an abutment (intaglio grown by `grow`, closed by a
// skirt below the margin). Connectors are clipped by it so they never fill the cavity.
Mesh makeCavity(const CrownBase& base, double grow = 0.01);

struct BridgeUnion {
    Mesh mesh;
    bool ok = false;
    std::string error;
    double volume = 0.0;
    bool watertight = false;
    std::vector<double> connectorAreas; // measured cross-section of the united bridge at each connector
};

// One unit of a bridge. Units and connectors are clipped by the preparation spaces (cavities) of
// all abutments except their own, so nothing ever reaches into a preparation.
struct BridgeUnionPart {
    const Mesh* mesh = nullptr;
    int ownCavity = -1; // index into `cavities` (abutments), -1 for pontics
};

BridgeUnion uniteBridge(const std::vector<BridgeUnionPart>& units, const std::vector<ConnectorSpec>& connectors, const std::vector<Mesh>& cavities);

// Area of the cross-section of `mesh` with `plane`, counting only the loops whose centroid lies
// within `radius` of `near` (e.g. one connector).
double crossSectionArea(const Mesh& mesh, const Plane& plane, const glm::dvec3& near, double radius);

} // namespace occlusa::crown
