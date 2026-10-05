#pragma once

#include "core/Mesh.h"

#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace occlusa::crown {

// ---------------------------------------------------------------------------
// Tooth kinds and reference dimensions
// ---------------------------------------------------------------------------

enum class ToothKind { CentralIncisor, LateralIncisor, Canine, FirstPremolar, SecondPremolar, FirstMolar, SecondMolar };

ToothKind toothKindFromFdi(int fdi);
std::string_view toString(ToothKind kind);
int fdiPosition(ToothKind kind); // 1..7

// Average dimensions of a permanent tooth (from the built-in Natural anatomy), mm.
struct ToothTemplate {
    ToothKind kind = ToothKind::FirstMolar;
    const char* name = "";
    double mesioDistal = 0.0;   // at the height of contour
    double buccoLingual = 0.0;
    double crownHeight = 0.0;   // cervical line to the highest cusp tip
    double contourHeight = 0.0; // proximal height of contour, fraction of the crown height
};
const ToothTemplate& toothTemplate(ToothKind kind, bool upper);

// ---------------------------------------------------------------------------
// Library teeth resampled for the crown builder
// ---------------------------------------------------------------------------

// A library tooth seen from a point on its axis: for every azimuth (0 = mesial, 90 degrees =
// buccal) a profile in the (radius, height) half plane from the height of contour over the
// occlusal surface to the axis. The crown builder scales these profiles to the restoration and
// joins them to the margin, so any library tooth works with margin adaptation, thickness
// control and sculpting. Heights are above the tooth's cervical line.
struct ToothShape {
    static constexpr int kAzimuths = 144;
    static constexpr int kProfile = 40; // points per profile; the last one is the apex on the axis

    double halfMesial = 0.0, halfDistal = 0.0, halfBuccal = 0.0, halfLingual = 0.0; // contour extents, mm
    double height = 0.0;         // highest point above the cervical line
    double contourHeight = 0.0;  // mean proximal height of contour, fraction of `height`
    double occlusalLevel = 0.0;  // mean height of the inner occlusal surface (reference for cusp scaling)
    std::vector<glm::dvec2> profiles; // [azimuth * kProfile + k] = (r, z)

    // Profile point k for any azimuth (linear between the samples).
    glm::dvec2 at(double azimuth, int k) const;
};

// Resample a closed tooth mesh given in the tooth frame (x mesial, y buccal, z occlusal).
// `cervicalZ`: height of the cervical line in the mesh (default: its lowest point).
std::shared_ptr<const ToothShape> sampleToothShape(const Mesh& toothFrameMesh, std::optional<double> cervicalZ = std::nullopt);

// ---------------------------------------------------------------------------
// Libraries
// ---------------------------------------------------------------------------

// On disk a library is a folder with `library.json` and one STL per tooth:
//   { "format": "occlusacad.toothlibrary", "version": 1, "id": "...", "name": "...",
//     "author": "...", "license": "...", "description": "...",
//     "teeth": [ { "fdi": 16, "file": "16.stl",
//                  "origin": [x,y,z], "mesial": [x,y,z], "buccal": [x,y,z], "occlusal": [x,y,z],
//                  "cervicalZ": 0.0 }, ... ] }
// The frame (optional, default: the file is already in the tooth frame) maps the STL into the
// tooth frame: origin on the tooth axis at the cervical line, axes mesial / buccal / occlusal.
// One tooth per position and jaw suffices; contralateral teeth use the same shape (the tooth
// frame is defined anatomically, so left and right teeth share it).
struct LibraryToothEntry {
    int fdi = 0;
    std::string file;
    glm::dvec3 origin{0.0};
    glm::dvec3 mesial{1, 0, 0};
    glm::dvec3 buccal{0, 1, 0};
    glm::dvec3 occlusal{0, 0, 1};
    std::optional<double> cervicalZ; // in the tooth frame
};

struct ToothLibraryManifest {
    std::string id;
    std::string name;
    std::string author;
    std::string license;
    std::string description;
    std::vector<LibraryToothEntry> teeth;
};

ToothLibraryManifest readToothLibraryManifest(const std::filesystem::path& folder);
void writeToothLibraryManifest(const std::filesystem::path& folder, const ToothLibraryManifest& manifest);

// Load one tooth into the tooth frame (applies the entry's frame, fixes the winding).
Mesh loadLibraryTooth(const std::filesystem::path& folder, const LibraryToothEntry& entry);

// Write a library folder (STL + manifest) from meshes already in the tooth frame.
void writeToothLibrary(const std::filesystem::path& folder, ToothLibraryManifest manifest, const std::vector<std::pair<int, Mesh>>& teeth);

// Create a manifest for a folder of STL files named by FDI number (e.g. 16.stl, 36.stl), all
// already in the tooth frame. Throws if no tooth files are found.
ToothLibraryManifest manifestFromStlFolder(const std::filesystem::path& folder, const std::string& name);

struct ToothLibraryInfo {
    std::string id;
    std::string name;
    std::string author;
    std::string license;
    std::string description;
    std::filesystem::path folder; // empty for built-in libraries
    bool builtIn = false;
    std::vector<int> teeth;       // FDI numbers provided
};

// All libraries known to the application: the built-in generated ones and library folders
// found in the scanned directories. Shapes are created on first use and cached. Thread safe.
class ToothLibraryRegistry {
public:
    static ToothLibraryRegistry& instance();
    static constexpr const char* kDefaultLibrary = "occlusacad-natural";

    std::vector<ToothLibraryInfo> libraries() const;
    std::optional<ToothLibraryInfo> find(const std::string& id) const;

    // Register every sub-folder of `directory` that holds a library.json. Returns the number added.
    int scanDirectory(const std::filesystem::path& directory);
    // Register one library folder; returns its id (throws on an invalid library).
    std::string addFolder(const std::filesystem::path& folder);

    // Library tooth for a position (falls back to the default library when the library lacks it).
    std::shared_ptr<const Mesh> toothMesh(const std::string& libraryId, ToothKind kind, bool upper);
    std::shared_ptr<const ToothShape> shape(const std::string& libraryId, ToothKind kind, bool upper);

private:
    ToothLibraryRegistry();
    struct Entry;
    std::shared_ptr<const Mesh> loadMesh(const Entry& e, ToothKind kind, bool upper);

    mutable std::mutex mutex_;
    std::vector<std::shared_ptr<Entry>> entries_;
};

// Shape used by a crown: the requested library, falling back to the default.
std::shared_ptr<const ToothShape> toothShape(const std::string& libraryId, ToothKind kind, bool upper);

} // namespace occlusa::crown
