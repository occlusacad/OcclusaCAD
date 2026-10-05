#pragma once

#include "core/crown/Bridge.h"
#include "core/crown/CrownBuilder.h"
#include "core/crown/Margin.h"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace occlusa::designer {

// Design state of one tooth-borne restoration (crown or coping). Geometry is in the
// coordinates of the scan containing the preparation; the scan's transform places it.
struct RestorationDesign {
    int tooth = 0;           // FDI
    std::string type;        // dental::RestorationType key
    int prepScanId = 0;      // document scan ids (0 = none)
    int antagonistScanId = 0;
    bool scansAssigned = false; // default scans chosen once the case data finished loading

    // Margin
    std::vector<std::uint32_t> controls; // control vertices on the prep scan
    crown::MarginLine margin;
    std::optional<glm::dvec3> prepPoint;  // a point on the preparation (defines the inside of the margin)

    // Insertion axis
    std::optional<glm::dvec3> insertionAxis;
    glm::dvec3 marginAxis{0.0};          // normal of the margin loop (reference for tilts)

    // Crown
    crown::CrownParameters params;
    std::optional<crown::CrownFrame> orientation; // estimated mesial / buccal directions
    std::vector<float> displacement;     // sculpting / contact adaptation, one value per crown vertex
    bool crownGenerated = false;

    // Derived data (rebuilt on demand, not saved).
    std::shared_ptr<const crown::DieRegion> die;
    std::string dieError;
    std::shared_ptr<const crown::CrownBase> base;
    std::shared_ptr<const crown::ContactScene> contacts;
    std::shared_ptr<const crown::CrownMesh> crown;

    bool supported() const;            // crowns, copings and pontics; other types are not designed yet
    bool isPontic() const { return type == "pontic"; }
    // Parameters as used for the geometry (pontics: no cement gap, rounded basal edge).
    crown::CrownParameters effectiveParams() const { return isPontic() ? crown::ponticParameters(params) : params; }
    bool marginClosed() const { return margin.closed && margin.vertices.size() >= 6; }
    void invalidateDie()
    {
        die.reset();
        dieError.clear();
        invalidateBase();
    }
    void invalidateBase()
    {
        base.reset();
        contacts.reset();
        crown.reset();
    }
};

// A bridge: adjacent abutments and pontics joined by connectors into one restoration.
struct BridgeDesign {
    std::vector<int> teeth;          // arch order
    double connectorArea = 9.0;      // mm^2 minimum cross-section (posterior zirconia)
    double connectorHeightRatio = 1.3;
    double embrasure = 1.0;          // gingival clearance below the connectors, mm
    std::optional<glm::dvec3> axis;  // common insertion axis of the abutments (prep scan coordinates)
    std::vector<crown::ConnectorEdit> edits; // per connector (between teeth[i] and teeth[i+1]); may be shorter

    // Derived / UI state (not saved).
    int selectedConnector = -1;
    std::vector<crown::ConnectorSpec> connectors;
    std::vector<std::string> warnings;
    std::shared_ptr<const crown::BridgeUnion> result; // the merged bridge (null = not merged / out of date)

    crown::ConnectorEdit& editFor(std::size_t i)
    {
        if (edits.size() <= i)
            edits.resize(i + 1);
        return edits[i];
    }
    crown::ConnectorEdit editAt(std::size_t i) const { return i < edits.size() ? edits[i] : crown::ConnectorEdit{}; }

    bool contains(int tooth) const
    {
        for (int t : teeth)
            if (t == tooth)
                return true;
        return false;
    }
    std::string label() const
    {
        std::string s;
        for (int t : teeth)
            s += (s.empty() ? "" : "-") + std::to_string(t);
        return s;
    }
};

struct SavedBridge {
    std::vector<int> teeth;
    double connectorArea = 9.0;
    double connectorHeightRatio = 1.3;
    double embrasure = 1.0;
    std::optional<glm::dvec3> axis;
    std::vector<crown::ConnectorEdit> edits;
};

// Persisted form (inside the design state JSON).
struct SavedRestoration {
    int tooth = 0;
    std::string type;
    std::string prepScanSource;
    std::string antagonistSource;
    std::vector<std::uint32_t> controls;
    std::vector<std::uint32_t> margin;
    bool marginClosed = false;
    std::size_t prepVertexCount = 0;      // validates vertex indices against the scan
    std::optional<glm::dvec3> prepPoint;
    std::optional<glm::dvec3> insertionAxis;
    crown::CrownParameters params;
    std::optional<crown::CrownFrame> orientation;
    std::vector<float> displacement;
    bool crownGenerated = false;
    std::string crownFile;                // case-relative exported STL (if saved)
};

} // namespace occlusa::designer
