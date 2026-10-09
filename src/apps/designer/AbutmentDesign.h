#pragma once

#include "core/implant/Abutment.h"
#include "core/implant/ImplantLibrary.h"
#include "core/implant/ScanBodyFit.h"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace occlusa::designer {

// Design state of one implant restoration (custom abutment workflow). The implant position and
// all geometry are in the coordinates of the scan holding the scan body; the scan's transform
// places them in the scene.
struct ImplantRestoration {
    int tooth = 0;          // FDI
    std::string type;       // dental::RestorationType key
    int scanId = 0;         // scan with the scan body
    bool scanAssigned = false;

    // Implant library connection (chosen in the designer).
    std::string libraryId;
    std::string connectionId;

    // Implant position from the scan body: implant frame -> scan coordinates.
    std::optional<glm::dmat4> implantToScan;
    implant::ScanBodyFitResult fit;

    // Abutment (created on the connection once the implant position is known).
    std::optional<implant::AbutmentShape> shape;

    // Derived (not saved).
    std::shared_ptr<const implant::Connection> connection;
    std::string connectionError;
    std::string loadedKey; // "library/connection" that `connection` (or the error) belongs to
    std::shared_ptr<const implant::AbutmentGeometry> geometry;
    std::shared_ptr<const implant::AbutmentSolid> solid; // finished abutment (for export)

    bool designsAbutment() const { return type == "custom_abutment"; }
    bool hasConnection() const { return !libraryId.empty() && !connectionId.empty(); }
    void invalidate()
    {
        geometry.reset();
        solid.reset();
    }
};

// Persisted form (inside the design state JSON).
struct SavedImplant {
    int tooth = 0;
    std::string type;
    std::string scanSource;
    std::string libraryId;
    std::string connectionId;
    std::optional<glm::dmat4> implantToScan;
    double fitRms = 0.0;
    double fitWithin = 0.0;
    std::optional<implant::AbutmentShape> shape;
    std::string abutmentFile; // case-relative exported STL (if saved)
};

} // namespace occlusa::designer
