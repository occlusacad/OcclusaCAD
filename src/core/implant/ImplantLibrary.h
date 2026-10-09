#pragma once

#include "core/Mesh.h"

#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace occlusa::implant {

// ---------------------------------------------------------------------------
// Implant libraries
// ---------------------------------------------------------------------------
//
// An implant library describes implant connections (one per implant system and platform) for
// custom abutments. All geometry of a connection is given in the implant frame: origin at the
// implant platform on the implant axis, +z along the axis out of the implant (coronally).
//
// Per connection:
//   interface     (required) the connection part of the abutment that fits the implant. It is
//                 milled exactly as given and can never be changed by the designer.
//   scan body     (required) the scan body as it sits on the implant; matched to the scan to
//                 find the implant position.
//   screw channel (optional) solid removed from the abutment for the screw; otherwise a
//                 cylinder of `screwChannelDiameter` along the axis.
//   min thickness (optional) solid the abutment must contain (material strength).
//   blank         (optional) pre-milled blank the abutment must fit inside.
//
// On disk (OcclusaCAD format) a library is a folder with `library.json` and STL files:
//   { "format": "occlusacad.implantlibrary", "version": 1, "id": "...", "name": "...",
//     "manufacturer": "...", "author": "...", "license": "...", "description": "...",
//     "connections": [ { "id": "rp41", "name": "RP 4.1", "system": "Conical", "platformDiameter": 4.1,
//         "interface": "rp41/interface.stl", "scanBody": "rp41/scanbody.stl",
//         "screwChannel": "...", "minThickness": "...", "blank": "...",
//         "screwChannelDiameter": 2.4, "minWall": 0.4,
//         "interfaceTop": { "height": 0.6, "diameter": 4.1 } } ] }
// "interfaceTop" is optional: the top circle of the interface is measured from the mesh.

struct ConnectionInfo {
    std::string id;
    std::string name;            // e.g. "RP 4.1"
    std::string system;          // implant system, e.g. "Conical"
    double platformDiameter = 0.0;
    double screwChannelDiameter = 2.4;
    double minWall = 0.4;        // minimum wall thickness of the abutment (mm)
    std::optional<double> interfaceTopHeight;   // given in the manifest (else measured)
    std::optional<double> interfaceTopDiameter;
    // Files relative to the library folder (empty = not provided).
    std::string interfaceFile, scanBodyFile, screwChannelFile, minThicknessFile, blankFile;
};

struct LibraryInfo {
    std::string id;
    std::string name;
    std::string manufacturer;
    std::string author;
    std::string license;
    std::string description;
    std::string format = "occlusacad"; // "occlusacad", "exocad", "3shape"
    std::filesystem::path folder;      // empty for built-in libraries
    bool builtIn = false;
    std::vector<ConnectionInfo> connections;

    const ConnectionInfo* connection(const std::string& connectionId) const;
};

LibraryInfo readImplantLibraryManifest(const std::filesystem::path& folder);
void writeImplantLibraryManifest(const std::filesystem::path& folder, const LibraryInfo& library);

// Loaded geometry of one connection, in the implant frame.
struct Connection {
    ConnectionInfo info;
    std::shared_ptr<const Mesh> interfaceMesh;
    std::shared_ptr<const Mesh> scanBody;
    std::shared_ptr<const Mesh> screwChannel;  // optional
    std::shared_ptr<const Mesh> minThickness;  // optional
    std::shared_ptr<const Mesh> blank;         // optional
    double topHeight = 0.0;  // the interface's top circle: the abutment is built on it
    double topRadius = 0.0;
    bool interfaceClosed = false; // usable for a solid union with the designed part
};

// Top circle of an interface mesh: its highest level and the outer radius there.
void measureInterfaceTop(const Mesh& interfaceMesh, double& height, double& radius);

// The built-in generic library (generated; no third-party content).
constexpr const char* kGenericLibrary = "occlusacad-generic";
LibraryInfo genericLibraryInfo();
Mesh genericInterface(double platformDiameter);
Mesh genericScanBody(double platformDiameter);
// Signed distance to the generic scan body (implant frame); for synthetic test scans.
double genericScanBodySdf(const glm::dvec3& p, double platformDiameter);

// Write a library folder in the OcclusaCAD format (meshes in the implant frame).
void writeImplantLibrary(const std::filesystem::path& folder, LibraryInfo library, const std::vector<std::shared_ptr<const Connection>>& connections);

// All implant libraries known to the application. Geometry is loaded on first use and cached.
// Thread safe.
class ImplantLibraryRegistry {
public:
    static ImplantLibraryRegistry& instance();

    std::vector<LibraryInfo> libraries() const;
    std::optional<LibraryInfo> find(const std::string& id) const;

    // Register every sub-folder of `directory` holding a library. Returns the number added.
    int scanDirectory(const std::filesystem::path& directory);
    // Register one library folder; returns its id (throws on an invalid library).
    std::string addFolder(const std::filesystem::path& folder);

    // Loaded connection (throws when the library or its files are missing or invalid).
    std::shared_ptr<const Connection> connection(const std::string& libraryId, const std::string& connectionId);

private:
    ImplantLibraryRegistry();
    struct Entry {
        LibraryInfo info;
        std::map<std::string, std::shared_ptr<const Connection>> loaded;
    };
    mutable std::mutex mutex_;
    std::vector<std::shared_ptr<Entry>> entries_;
};

// Reads a library folder of any supported format into LibraryInfo (OcclusaCAD library.json for
// now; exocad and 3Shape readers plug in here).
LibraryInfo readImplantLibrary(const std::filesystem::path& folder);
bool looksLikeImplantLibrary(const std::filesystem::path& folder);

} // namespace occlusa::implant
