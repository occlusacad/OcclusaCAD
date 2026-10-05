#include "core/crown/ToothLibrary.h"

#include "core/Dental.h"
#include "core/Log.h"
#include "core/MeshBvh.h"
#include "core/MeshTopology.h"
#include "core/Platform.h"
#include "core/StlIO.h"
#include "core/crown/AnatomyGenerator.h"

#include <json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <numbers>
#include <stdexcept>

namespace fs = std::filesystem;
using nlohmann::json;

namespace occlusa::crown {

// ---------------------------------------------------------------------------
// Kinds
// ---------------------------------------------------------------------------

ToothKind toothKindFromFdi(int fdi)
{
    switch (fdi % 10) {
    case 1: return ToothKind::CentralIncisor;
    case 2: return ToothKind::LateralIncisor;
    case 3: return ToothKind::Canine;
    case 4: return ToothKind::FirstPremolar;
    case 5: return ToothKind::SecondPremolar;
    case 6: return ToothKind::FirstMolar;
    default: return ToothKind::SecondMolar; // 7 and 8
    }
}

int fdiPosition(ToothKind kind)
{
    return static_cast<int>(kind) + 1;
}

std::string_view toString(ToothKind kind)
{
    switch (kind) {
    case ToothKind::CentralIncisor: return "Central incisor";
    case ToothKind::LateralIncisor: return "Lateral incisor";
    case ToothKind::Canine: return "Canine";
    case ToothKind::FirstPremolar: return "First premolar";
    case ToothKind::SecondPremolar: return "Second premolar";
    case ToothKind::FirstMolar: return "First molar";
    case ToothKind::SecondMolar: return "Second molar";
    }
    return "";
}

// ---------------------------------------------------------------------------
// Shape sampling
// ---------------------------------------------------------------------------

glm::dvec2 ToothShape::at(double azimuth, int k) const
{
    const double twoPi = 2.0 * std::numbers::pi;
    double a = std::fmod(azimuth, twoPi);
    if (a < 0.0)
        a += twoPi;
    const double t = a / twoPi * kAzimuths;
    const int i0 = static_cast<int>(std::floor(t)) % kAzimuths;
    const int i1 = (i0 + 1) % kAzimuths;
    const double f = t - std::floor(t);
    const auto kk = static_cast<std::size_t>(std::clamp(k, 0, kProfile - 1));
    return glm::mix(profiles[static_cast<std::size_t>(i0) * kProfile + kk], profiles[static_cast<std::size_t>(i1) * kProfile + kk], f);
}

std::shared_ptr<const ToothShape> sampleToothShape(const Mesh& mesh, std::optional<double> cervicalZ)
{
    if (mesh.empty())
        throw std::runtime_error("The tooth mesh is empty.");
    auto shared = std::make_shared<Mesh>(mesh);
    if (shared->normals.size() != shared->positions.size())
        shared->computeVertexNormals();
    const MeshBvh bvh(shared);
    double zmin = 1e9, zmax = -1e9;
    for (const auto& p : mesh.positions) {
        zmin = std::min(zmin, static_cast<double>(p.z));
        zmax = std::max(zmax, static_cast<double>(p.z));
    }
    const double z0 = cervicalZ.value_or(zmin);
    // Centre: middle of the crown's footprint, about half way up (the crown is star-shaped from there).
    glm::dvec2 lo(1e9), hi(-1e9);
    for (const auto& p : mesh.positions)
        if (p.z > z0 + 0.3 * (zmax - z0)) {
            lo = glm::min(lo, glm::dvec2(p.x, p.y));
            hi = glm::max(hi, glm::dvec2(p.x, p.y));
        }
    const glm::dvec3 centre(0.5 * (lo + hi), z0 + 0.45 * (zmax - z0));

    auto shape = std::make_shared<ToothShape>();
    constexpr int kPolar = 200;
    const int A = ToothShape::kAzimuths, K = ToothShape::kProfile;
    shape->profiles.resize(static_cast<std::size_t>(A * K));
    std::vector<double> contourR(static_cast<std::size_t>(A)), contourZ(static_cast<std::size_t>(A));
    double inner = 0.0;
    int innerCount = 0;
    for (int a = 0; a < A; ++a) {
        const double th = 2.0 * std::numbers::pi * a / A;
        std::vector<glm::dvec2> raw; // (r, z) from the apex (polar 0) downwards
        for (int j = 0; j <= kPolar; ++j) {
            const double phi = std::numbers::pi * 0.95 * j / kPolar;
            const glm::dvec3 d(std::sin(phi) * std::cos(th), std::sin(phi) * std::sin(th), std::cos(phi));
            if (auto hit = bvh.raycast(glm::vec3(centre), glm::vec3(d), 100.0f)) {
                const glm::dvec3 q(hit->point);
                raw.emplace_back(glm::length(glm::dvec2(q) - glm::dvec2(centre)), q.z - z0);
            }
        }
        if (raw.size() < 8)
            throw std::runtime_error("The tooth is not closed or not in the tooth frame (rays from its centre escape).");
        // Height of contour: the widest point below the occlusal part.
        std::size_t jc = 0;
        for (std::size_t j = 0; j < raw.size(); ++j)
            if (raw[j].x >= raw[jc].x && raw[j].y > 0.05 * (zmax - z0))
                jc = j;
        contourR[static_cast<std::size_t>(a)] = raw[jc].x;
        contourZ[static_cast<std::size_t>(a)] = raw[jc].y;
        // Profile from the contour up to the apex, resampled by arc length.
        std::vector<glm::dvec2> path(raw.rbegin() + static_cast<std::ptrdiff_t>(raw.size() - 1 - jc), raw.rend());
        path.back().x = 0.0;
        std::vector<double> s(path.size(), 0.0);
        for (std::size_t j = 1; j < path.size(); ++j)
            s[j] = s[j - 1] + glm::length(path[j] - path[j - 1]);
        const double total = std::max(s.back(), 1e-9);
        std::size_t seg = 0;
        for (int k = 0; k < K; ++k) {
            const double target = total * k / (K - 1);
            while (seg + 2 < path.size() && s[seg + 1] < target)
                ++seg;
            const double f = std::clamp((target - s[seg]) / std::max(s[seg + 1] - s[seg], 1e-12), 0.0, 1.0);
            glm::dvec2 q = glm::mix(path[seg], path[seg + 1], f);
            if (k == K - 1)
                q = path.back();
            shape->profiles[static_cast<std::size_t>(a * K + k)] = q;
            if (q.x < 0.5 * raw[jc].x) {
                inner += q.y;
                ++innerCount;
            }
        }
    }
    for (int a = 0; a < A; ++a) {
        const double th = 2.0 * std::numbers::pi * a / A;
        const double r = contourR[static_cast<std::size_t>(a)];
        shape->halfMesial = std::max(shape->halfMesial, r * std::cos(th));
        shape->halfDistal = std::max(shape->halfDistal, -r * std::cos(th));
        shape->halfBuccal = std::max(shape->halfBuccal, r * std::sin(th));
        shape->halfLingual = std::max(shape->halfLingual, -r * std::sin(th));
    }
    for (const auto& q : shape->profiles)
        shape->height = std::max(shape->height, q.y);
    shape->contourHeight = 0.5 * (contourZ[0] + contourZ[static_cast<std::size_t>(A / 2)]) / std::max(shape->height, 1e-9);
    shape->occlusalLevel = innerCount ? inner / innerCount : 0.8 * shape->height;
    return shape;
}

// ---------------------------------------------------------------------------
// Library files
// ---------------------------------------------------------------------------

namespace {

json vec(const glm::dvec3& v)
{
    return json::array({v.x, v.y, v.z});
}

glm::dvec3 vec(const json& j, const char* key, glm::dvec3 fallback)
{
    if (!j.contains(key) || !j[key].is_array() || j[key].size() != 3)
        return fallback;
    return {j[key][0].get<double>(), j[key][1].get<double>(), j[key][2].get<double>()};
}

std::string slug(const std::string& s)
{
    std::string out;
    for (char c : s) {
        const auto u = static_cast<unsigned char>(c);
        if (std::isalnum(u))
            out += static_cast<char>(std::tolower(u));
        else if (!out.empty() && out.back() != '-')
            out += '-';
    }
    while (!out.empty() && out.back() == '-')
        out.pop_back();
    return out.empty() ? "library" : out;
}

} // namespace

ToothLibraryManifest readToothLibraryManifest(const fs::path& folder)
{
    std::ifstream in(folder / "library.json");
    if (!in)
        throw std::runtime_error("No library.json in " + platform::pathToUtf8(folder));
    const json j = json::parse(in);
    if (j.value("format", "") != "occlusacad.toothlibrary")
        throw std::runtime_error("library.json is not an OcclusaCAD tooth library.");
    ToothLibraryManifest m;
    m.name = j.value("name", platform::pathToUtf8(folder.filename()));
    m.id = j.value("id", slug(m.name));
    m.author = j.value("author", "");
    m.license = j.value("license", "");
    m.description = j.value("description", "");
    for (const auto& t : j.value("teeth", json::array())) {
        LibraryToothEntry e;
        e.fdi = t.value("fdi", 0);
        e.file = t.value("file", "");
        e.origin = vec(t, "origin", e.origin);
        e.mesial = vec(t, "mesial", e.mesial);
        e.buccal = vec(t, "buccal", e.buccal);
        e.occlusal = vec(t, "occlusal", e.occlusal);
        if (t.contains("cervicalZ"))
            e.cervicalZ = t["cervicalZ"].get<double>();
        if (!dental::isValidFdi(e.fdi) || e.file.empty())
            throw std::runtime_error("library.json has an invalid tooth entry.");
        m.teeth.push_back(e);
    }
    if (m.teeth.empty())
        throw std::runtime_error("The library has no teeth.");
    return m;
}

void writeToothLibraryManifest(const fs::path& folder, const ToothLibraryManifest& m)
{
    json j;
    j["format"] = "occlusacad.toothlibrary";
    j["version"] = 1;
    j["id"] = m.id.empty() ? slug(m.name) : m.id;
    j["name"] = m.name;
    j["author"] = m.author;
    j["license"] = m.license;
    j["description"] = m.description;
    json teeth = json::array();
    for (const auto& e : m.teeth) {
        json t = {{"fdi", e.fdi}, {"file", e.file}, {"origin", vec(e.origin)}, {"mesial", vec(e.mesial)}, {"buccal", vec(e.buccal)},
                  {"occlusal", vec(e.occlusal)}};
        if (e.cervicalZ)
            t["cervicalZ"] = *e.cervicalZ;
        teeth.push_back(t);
    }
    j["teeth"] = teeth;
    fs::create_directories(folder);
    std::ofstream out(folder / "library.json");
    out << j.dump(2);
    if (!out)
        throw std::runtime_error("Could not write " + platform::pathToUtf8(folder / "library.json"));
}

Mesh loadLibraryTooth(const fs::path& folder, const LibraryToothEntry& e)
{
    Mesh m = readStl(folder / platform::pathFromUtf8(e.file));
    const glm::dvec3 O = glm::normalize(e.occlusal);
    glm::dvec3 M = e.mesial - glm::dot(e.mesial, O) * O;
    if (glm::length(M) < 1e-9)
        throw std::runtime_error("Tooth " + std::to_string(e.fdi) + ": the mesial axis is parallel to the occlusal axis.");
    M = glm::normalize(M);
    glm::dvec3 B = e.buccal - glm::dot(e.buccal, O) * O - glm::dot(e.buccal, M) * M;
    if (glm::length(B) < 1e-9)
        B = glm::cross(O, M);
    B = glm::normalize(B);
    for (auto& p : m.positions) {
        const glm::dvec3 d = glm::dvec3(p) - e.origin;
        p = glm::vec3(glm::dot(d, M), glm::dot(d, B), glm::dot(d, O));
    }
    if (signedVolume(m) < 0.0) // reflected frame or inverted file
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
            std::swap(m.indices[t + 1], m.indices[t + 2]);
    m.computeVertexNormals();
    return m;
}

void writeToothLibrary(const fs::path& folder, ToothLibraryManifest manifest, const std::vector<std::pair<int, Mesh>>& teeth)
{
    fs::create_directories(folder);
    manifest.teeth.clear();
    for (const auto& [fdi, mesh] : teeth) {
        LibraryToothEntry e;
        e.fdi = fdi;
        e.file = std::to_string(fdi) + ".stl";
        e.cervicalZ = 0.0;
        writeStlBinary(folder / e.file, mesh, glm::dmat4(1.0), "OcclusaCAD tooth library: " + manifest.name + " " + std::to_string(fdi));
        manifest.teeth.push_back(e);
    }
    writeToothLibraryManifest(folder, manifest);
}

ToothLibraryManifest manifestFromStlFolder(const fs::path& folder, const std::string& name)
{
    ToothLibraryManifest m;
    m.name = name;
    m.id = slug(name);
    for (const auto& entry : fs::directory_iterator(folder)) {
        if (!entry.is_regular_file())
            continue;
        std::string ext = platform::pathToUtf8(entry.path().extension());
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (ext != ".stl")
            continue;
        int fdi = 0;
        try {
            fdi = std::stoi(platform::pathToUtf8(entry.path().stem()));
        } catch (...) {
            continue;
        }
        if (!dental::isValidFdi(fdi))
            continue;
        LibraryToothEntry e;
        e.fdi = fdi;
        e.file = platform::pathToUtf8(entry.path().filename());
        m.teeth.push_back(e);
    }
    std::sort(m.teeth.begin(), m.teeth.end(), [](const auto& a, const auto& b) { return a.fdi < b.fdi; });
    if (m.teeth.empty())
        throw std::runtime_error("No tooth STL files named by FDI number (e.g. 16.stl) in " + platform::pathToUtf8(folder));
    return m;
}

// ---------------------------------------------------------------------------
// Registry
// ---------------------------------------------------------------------------

struct ToothLibraryRegistry::Entry {
    ToothLibraryInfo info;
    std::optional<LibraryStyle> style; // built-in
    ToothLibraryManifest manifest;
    std::map<int, std::shared_ptr<const Mesh>> meshes;
    std::map<int, std::shared_ptr<const ToothShape>> shapes;
};

namespace {

int key(ToothKind kind, bool upper)
{
    return (upper ? 100 : 0) + static_cast<int>(kind);
}

} // namespace

ToothLibraryRegistry::ToothLibraryRegistry()
{
    struct Builtin {
        const char* id;
        const char* name;
        LibraryStyle style;
        const char* description;
    };
    const Builtin builtins[] = {
        {"occlusacad-natural", "OcclusaCAD Natural", LibraryStyle::Natural, "Average adult anatomy with clear cusps, ridges and grooves."},
        {"occlusacad-young", "OcclusaCAD Young", LibraryStyle::Young, "Pronounced cusps and deep fissures, rounder outlines."},
        {"occlusacad-mature", "OcclusaCAD Mature", LibraryStyle::Mature, "Worn anatomy: low cusps, wide occlusal tables, flat incisal edges."},
    };
    for (const auto& b : builtins) {
        auto e = std::make_shared<Entry>();
        e->info.id = b.id;
        e->info.name = b.name;
        e->info.author = "OcclusaCAD";
        e->info.license = "Generated by OcclusaCAD (no third-party content)";
        e->info.description = b.description;
        e->info.builtIn = true;
        for (int q : {1, 3})
            for (int p = 1; p <= 7; ++p)
                e->info.teeth.push_back(q * 10 + p);
        e->style = b.style;
        entries_.push_back(e);
    }
}

ToothLibraryRegistry& ToothLibraryRegistry::instance()
{
    static ToothLibraryRegistry registry;
    return registry;
}

std::vector<ToothLibraryInfo> ToothLibraryRegistry::libraries() const
{
    std::lock_guard lock(mutex_);
    std::vector<ToothLibraryInfo> out;
    for (const auto& e : entries_)
        out.push_back(e->info);
    return out;
}

std::optional<ToothLibraryInfo> ToothLibraryRegistry::find(const std::string& id) const
{
    std::lock_guard lock(mutex_);
    for (const auto& e : entries_)
        if (e->info.id == id)
            return e->info;
    return std::nullopt;
}

std::string ToothLibraryRegistry::addFolder(const fs::path& folder)
{
    auto e = std::make_shared<Entry>();
    e->manifest = readToothLibraryManifest(folder);
    e->info.id = e->manifest.id;
    e->info.name = e->manifest.name;
    e->info.author = e->manifest.author;
    e->info.license = e->manifest.license;
    e->info.description = e->manifest.description;
    e->info.folder = folder;
    for (const auto& t : e->manifest.teeth)
        e->info.teeth.push_back(t.fdi);
    std::lock_guard lock(mutex_);
    for (auto& existing : entries_)
        if (existing->info.id == e->info.id) {
            if (existing->info.builtIn)
                throw std::runtime_error("The library id '" + e->info.id + "' is reserved for a built-in library.");
            existing = e; // a newer copy of the same library
            return e->info.id;
        }
    entries_.push_back(e);
    return e->info.id;
}

int ToothLibraryRegistry::scanDirectory(const fs::path& directory)
{
    std::error_code ec;
    if (!fs::is_directory(directory, ec))
        return 0;
    int added = 0;
    for (const auto& entry : fs::directory_iterator(directory, ec)) {
        if (!entry.is_directory() || !fs::exists(entry.path() / "library.json"))
            continue;
        try {
            addFolder(entry.path());
            ++added;
        } catch (const std::exception& ex) {
            log::warn("Tooth library {} skipped: {}", platform::pathToUtf8(entry.path()), ex.what());
        }
    }
    return added;
}

std::shared_ptr<const Mesh> ToothLibraryRegistry::loadMesh(const Entry& e, ToothKind kind, bool upper)
{
    if (e.style)
        return std::make_shared<const Mesh>(generateToothCrown(kind, upper, *e.style));
    // Same position in the same jaw, either side.
    const int pos = fdiPosition(kind);
    for (const auto& t : e.manifest.teeth)
        if (dental::isUpper(t.fdi) == upper && t.fdi % 10 == pos)
            return std::make_shared<const Mesh>(loadLibraryTooth(e.info.folder, t));
    return nullptr;
}

std::shared_ptr<const Mesh> ToothLibraryRegistry::toothMesh(const std::string& id, ToothKind kind, bool upper)
{
    std::lock_guard lock(mutex_);
    std::shared_ptr<Entry> e, fallback;
    for (const auto& x : entries_) {
        if (x->info.id == id)
            e = x;
        if (x->info.id == kDefaultLibrary)
            fallback = x;
    }
    for (const auto& lib : {e, fallback}) {
        if (!lib)
            continue;
        const int k = key(kind, upper);
        if (auto it = lib->meshes.find(k); it != lib->meshes.end() && it->second)
            return it->second;
        try {
            if (auto m = loadMesh(*lib, kind, upper)) {
                lib->meshes[k] = m;
                return m;
            }
        } catch (const std::exception& ex) {
            log::warn("Tooth library {}: {}", lib->info.name, ex.what());
        }
    }
    return nullptr;
}

std::shared_ptr<const ToothShape> ToothLibraryRegistry::shape(const std::string& id, ToothKind kind, bool upper)
{
    {
        std::lock_guard lock(mutex_);
        for (const auto& x : entries_)
            if (x->info.id == id)
                if (auto it = x->shapes.find(key(kind, upper)); it != x->shapes.end())
                    return it->second;
    }
    auto mesh = toothMesh(id, kind, upper);
    if (!mesh)
        throw std::runtime_error("No tooth library provides a " + std::string(toString(kind)) + ".");
    std::optional<double> cervical;
    std::string resolved = kDefaultLibrary;
    {
        std::lock_guard lock(mutex_);
        for (const auto& x : entries_)
            if (x->info.id == id) {
                resolved = id;
                if (x->style)
                    cervical = 0.0;
                for (const auto& t : x->manifest.teeth)
                    if (dental::isUpper(t.fdi) == upper && t.fdi % 10 == fdiPosition(kind))
                        cervical = t.cervicalZ;
            }
    }
    auto s = sampleToothShape(*mesh, cervical);
    std::lock_guard lock(mutex_);
    for (const auto& x : entries_)
        if (x->info.id == resolved)
            x->shapes[key(kind, upper)] = s;
    return s;
}

std::shared_ptr<const ToothShape> toothShape(const std::string& libraryId, ToothKind kind, bool upper)
{
    return ToothLibraryRegistry::instance().shape(libraryId.empty() ? ToothLibraryRegistry::kDefaultLibrary : libraryId, kind, upper);
}

} // namespace occlusa::crown
