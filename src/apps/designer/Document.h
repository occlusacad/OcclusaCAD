#pragma once

#include "apps/designer/AbutmentDesign.h"
#include "apps/designer/CrownDesign.h"
#include "core/Dental.h"
#include "core/Mesh.h"
#include "core/Registration.h"
#include "core/Volume.h"
#include "db/CaseModel.h"
#include "gfx/Renderers.h"

#include <filesystem>
#include <map>
#include <set>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace occlusa::designer {

struct VolumeDisplay {
    double windowCenter = 1000.0;
    double windowWidth = 3000.0;
    double isoValue = 600.0;
    gfx::VolumeMode mode = gfx::VolumeMode::Isosurface;
    glm::vec3 color{0.93f, 0.89f, 0.80f};
    float opacity = 1.0f;
    glm::vec3 cropMin{0.0f}; // texture space [0,1]
    glm::vec3 cropMax{1.0f};
    bool visible = true;
    bool showThresholdOnSlices = false;
};

// The CBCT. Its voxel grid defines world space and is never transformed.
struct VolumeObject {
    std::shared_ptr<const Volume> volume;
    std::unique_ptr<gfx::GpuVolume> gpu;
    std::string source; // case-relative path (case mode) or absolute path
    std::string label;
    VolumeDisplay display;

    // Cached hard-tissue surface used as the ICP target.
    std::shared_ptr<const IcpTarget> icpTarget;
    double icpTargetIso = 0.0;
    Aabb icpTargetRegion;
};

struct RegistrationRecord {
    bool registered = false;
    std::string method;     // "point_pairs", "point_pairs+icp", "icp", "manual"
    double landmarkRms = 0.0;
    double surfaceRms = 0.0;
    double fractionWithinTolerance = 0.0;
};

// A surface scan (STL). transform maps scan (file) coordinates into CBCT world space.
struct ScanObject {
    int id = 0;
    std::shared_ptr<const Mesh> mesh;
    std::unique_ptr<gfx::GpuMesh> gpu;
    std::string source;
    std::string label;
    db::FileRole role = db::FileRole::ScanOther;
    glm::dmat4 transform{1.0};
    glm::dmat4 initialTransform{1.0};
    glm::vec3 color{0.9f, 0.78f, 0.66f};
    float opacity = 1.0f;
    bool visible = true;
    bool stepHidden = false;    // hidden by the current step (shown differently there); not saved
    RegistrationRecord registration;
    std::uint64_t revision = 0; // bumps when the transform changes (cache invalidation)

    Aabb worldBounds() const { return mesh ? mesh->bounds().transformed(transform) : Aabb{}; }
    void setTransform(const glm::dmat4& t)
    {
        transform = t;
        ++revision;
    }
};

// Extra geometry drawn in the 3D views (margin lines, die maps, crowns). Display only.
struct DisplayMesh {
    std::unique_ptr<gfx::GpuMesh> gpu;
    int scanId = 0; // placed with this scan's transform (0 = world coordinates)
    glm::vec3 color{1.0f};
    float opacity = 1.0f;
    bool vertexColors = false;
    bool depthBias = false;
    bool visible = true;
};

class Document {
public:
    std::optional<VolumeObject> volume;
    std::vector<std::unique_ptr<ScanObject>> scans;
    glm::dvec3 cursor{0.0}; // MPR crosshair (world)
    bool modified = false;

    // Tooth-borne restorations (crown & bridge workflow).
    std::vector<RestorationDesign> restorations;
    int activeRestoration = 0;
    std::map<int, std::shared_ptr<const crown::PrepScan>> prepScans; // scan id -> analysis (cache)
    std::set<int> prepScansRequested;
    std::vector<SavedRestoration> pendingRestorations;               // saved designs waiting for their scans
    std::vector<BridgeDesign> bridges;
    std::vector<SavedBridge> pendingBridges;
    BridgeDesign* bridgeOf(int tooth)
    {
        for (auto& b : bridges)
            if (b.contains(tooth))
                return &b;
        return nullptr;
    }
    const BridgeDesign* bridgeOf(int tooth) const { return const_cast<Document*>(this)->bridgeOf(tooth); }
    RestorationDesign* active()
    {
        return activeRestoration >= 0 && activeRestoration < static_cast<int>(restorations.size()) ? &restorations[static_cast<std::size_t>(activeRestoration)]
                                                                                                    : nullptr;
    }

    // Implant restorations (custom abutment workflow).
    std::vector<ImplantRestoration> implants;
    int activeImplant = 0;
    std::vector<SavedImplant> pendingImplants; // saved designs waiting for their scans
    ImplantRestoration* activeImplantRestoration()
    {
        return activeImplant >= 0 && activeImplant < static_cast<int>(implants.size()) ? &implants[static_cast<std::size_t>(activeImplant)] : nullptr;
    }

    std::map<std::string, DisplayMesh> overlays;
    void setOverlay(const std::string& key, DisplayMesh mesh)
    {
        overlays[key] = std::move(mesh);
        ++revision_;
    }
    void removeOverlay(const std::string& key)
    {
        if (overlays.erase(key))
            ++revision_;
    }
    void removeOverlaysWithPrefix(const std::string& prefix)
    {
        for (auto it = overlays.begin(); it != overlays.end();)
            it = it->first.rfind(prefix, 0) == 0 ? overlays.erase(it) : std::next(it);
        ++revision_;
    }
    void redraw() { ++revision_; } // display change that is not a document edit

    ScanObject* findScan(int id);
    ScanObject& addScan(std::shared_ptr<const Mesh> mesh, std::string source, std::string label, db::FileRole role);
    void removeScan(int id);
    Aabb sceneBounds() const;
    bool hasData() const { return volume.has_value() || !scans.empty(); }
    std::uint64_t revision() const { return revision_; }
    void touch()
    {
        ++revision_;
        modified = true;
    }

private:
    int nextScanId_ = 1;
    std::uint64_t revision_ = 0;
};

// Persisted design state (stored in the case database as JSON).
struct SavedScan {
    std::string source;
    std::string label;
    db::FileRole role = db::FileRole::ScanOther;
    glm::dmat4 transform{1.0};
    glm::vec3 color{0.9f, 0.78f, 0.66f};
    float opacity = 1.0f;
    bool visible = true;
    RegistrationRecord registration;
};

struct DesignState {
    std::string workflow;
    std::string currentStep;
    std::vector<std::string> completedSteps;
    std::optional<std::string> volumeSource;
    std::optional<VolumeDisplay> volumeDisplay;
    std::vector<SavedScan> scans;
    glm::dvec3 cursor{0.0};
    std::vector<SavedRestoration> restorations;
    std::vector<SavedBridge> bridges;
    std::vector<SavedImplant> implants;
    // Tooth numbering of the saved file (the saving user's setting). In memory teeth are FDI;
    // toJson writes them in this system and fromJson converts back.
    dental::Numbering numbering = dental::Numbering::FDI;

    std::string toJson() const;
    static DesignState fromJson(const std::string& json);
};

} // namespace occlusa::designer
