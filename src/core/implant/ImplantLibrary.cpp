#include "core/implant/ImplantLibrary.h"

#include "core/Log.h"
#include "core/MeshBoolean.h"
#include "core/Platform.h"
#include "core/StlIO.h"

#include <json.hpp>

#include <cctype>
#include <cmath>
#include <fstream>
#include <numbers>
#include <stdexcept>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace occlusa::implant {

const ConnectionInfo* LibraryInfo::connection(const std::string& connectionId) const
{
    for (const auto& c : connections)
        if (c.id == connectionId)
            return &c;
    return nullptr;
}

namespace {

std::string slug(const std::string& s)
{
    std::string out;
    for (char ch : s) {
        if (std::isalnum(static_cast<unsigned char>(ch)))
            out += static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        else if (!out.empty() && out.back() != '-')
            out += '-';
    }
    while (!out.empty() && out.back() == '-')
        out.pop_back();
    return out.empty() ? "library" : out;
}

void orientOutward(Mesh& m)
{
    if (signedVolume(m) < 0.0)
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
            std::swap(m.indices[t + 1], m.indices[t + 2]);
    m.computeVertexNormals();
}

// Solid of revolution about +z from a profile in the (r, z) half plane. A closed loop must stay
// off the axis; an open profile must start and end on the axis (r = 0), where it closes in poles.
Mesh revolve(const std::vector<glm::dvec2>& profile, bool closedLoop, int segments)
{
    Mesh m;
    const std::size_t n = profile.size();
    std::vector<std::vector<std::uint32_t>> ids(n);
    for (std::size_t i = 0; i < n; ++i) {
        const bool pole = !closedLoop && profile[i].x <= 1e-9;
        for (int s = 0; s < (pole ? 1 : segments); ++s) {
            const double a = 2.0 * std::numbers::pi * s / segments;
            ids[i].push_back(static_cast<std::uint32_t>(m.positions.size()));
            m.positions.emplace_back(profile[i].x * std::cos(a), profile[i].x * std::sin(a), profile[i].y);
        }
    }
    auto at = [&](std::size_t i, int s) { return ids[i].size() == 1 ? ids[i][0] : ids[i][static_cast<std::size_t>(s % segments)]; };
    const std::size_t rows = closedLoop ? n : n - 1;
    for (std::size_t i = 0; i < rows; ++i) {
        const std::size_t j = (i + 1) % n;
        for (int s = 0; s < segments; ++s) {
            const std::uint32_t a = at(i, s), b = at(i, s + 1), c = at(j, s + 1), d = at(j, s);
            if (a != b)
                m.indices.insert(m.indices.end(), {a, b, c});
            if (c != d)
                m.indices.insert(m.indices.end(), {a, c, d});
        }
    }
    orientOutward(m);
    return m;
}

// Straight prism over a convex or simple polygon (counter-clockwise, xy) from z0 to z1.
Mesh prism(const std::vector<glm::dvec2>& poly, double z0, double z1)
{
    Mesh m;
    const auto n = static_cast<std::uint32_t>(poly.size());
    for (double z : {z0, z1})
        for (const auto& p : poly)
            m.positions.emplace_back(p.x, p.y, z);
    for (std::uint32_t i = 0; i < n; ++i) {
        const std::uint32_t j = (i + 1) % n;
        m.indices.insert(m.indices.end(), {i, j, n + j, i, n + j, n + i});
    }
    for (std::uint32_t i = 1; i + 1 < n; ++i) {
        m.indices.insert(m.indices.end(), {0, i + 1, i});             // bottom
        m.indices.insert(m.indices.end(), {n, n + i, n + i + 1});     // top
    }
    orientOutward(m);
    return m;
}

Mesh cylinder(double r, double z0, double z1, int segments = 64)
{
    return revolve({{0.0, z0}, {r, z0}, {r, z1}, {0.0, z1}}, false, segments);
}

std::vector<glm::dvec2> hexagon(double acrossFlats)
{
    std::vector<glm::dvec2> p;
    const double rc = acrossFlats * 0.5 / std::cos(std::numbers::pi / 6.0);
    for (int i = 0; i < 6; ++i) {
        const double a = std::numbers::pi / 6.0 + i * std::numbers::pi / 3.0;
        p.emplace_back(rc * std::cos(a), rc * std::sin(a));
    }
    return p;
}

// Generic connection dimensions.
struct GenericSize {
    const char* id;
    const char* name;
    double platform;
    double hexAcrossFlats;
};
constexpr GenericSize kGenericSizes[] = {
    {"np35", "NP 3.5", 3.5, 2.5},
    {"rp41", "RP 4.1", 4.1, 2.7},
    {"wp50", "WP 5.0", 5.0, 2.7},
};
constexpr double kCollarHeight = 0.6;
constexpr double kScanBodyRadius = 2.0;
constexpr double kScanBodyHeight = 10.0;
constexpr double kScanBodyFlatX = 1.4;
constexpr double kScanBodyFlatZ = 4.0;

std::vector<glm::dvec2> scanBodyProfile(double platformDiameter)
{
    const double R = platformDiameter * 0.5;
    return {{0.0, 0.0}, {R, 0.0}, {R, 1.0}, {kScanBodyRadius, 1.0}, {kScanBodyRadius, kScanBodyHeight - 0.4}, {kScanBodyRadius - 0.4, kScanBodyHeight},
            {0.0, kScanBodyHeight}};
}

double sdPolygon(const glm::dvec2& p, const std::vector<glm::dvec2>& v)
{
    double d = glm::dot(p - v[0], p - v[0]);
    double s = 1.0;
    for (std::size_t i = 0, j = v.size() - 1; i < v.size(); j = i, ++i) {
        const glm::dvec2 e = v[j] - v[i], w = p - v[i];
        const glm::dvec2 b = w - e * std::clamp(glm::dot(w, e) / glm::dot(e, e), 0.0, 1.0);
        d = std::min(d, glm::dot(b, b));
        const bool c1 = p.y >= v[i].y, c2 = p.y < v[j].y, c3 = e.x * w.y > e.y * w.x;
        if ((c1 && c2 && c3) || (!c1 && !c2 && !c3))
            s = -s;
    }
    return s * std::sqrt(d);
}

std::shared_ptr<const Mesh> loadOptional(const fs::path& folder, const std::string& file, bool closed)
{
    if (file.empty())
        return nullptr;
    Mesh m = readStl(folder / platform::pathFromUtf8(file));
    if (m.empty())
        throw std::runtime_error(file + " is empty.");
    if (closed)
        orientOutward(m);
    return std::make_shared<const Mesh>(std::move(m));
}

} // namespace

// ---------------------------------------------------------------------------
// Manifest
// ---------------------------------------------------------------------------

LibraryInfo readImplantLibraryManifest(const fs::path& folder)
{
    std::ifstream in(folder / "library.json");
    if (!in)
        throw std::runtime_error("No library.json in " + platform::pathToUtf8(folder));
    const json j = json::parse(in);
    if (j.value("format", "") != "occlusacad.implantlibrary")
        throw std::runtime_error("library.json is not an OcclusaCAD implant library.");
    LibraryInfo lib;
    lib.name = j.value("name", platform::pathToUtf8(folder.filename()));
    lib.id = j.value("id", slug(lib.name));
    lib.manufacturer = j.value("manufacturer", "");
    lib.author = j.value("author", "");
    lib.license = j.value("license", "");
    lib.description = j.value("description", "");
    lib.folder = folder;
    for (const auto& c : j.value("connections", json::array())) {
        ConnectionInfo ci;
        ci.name = c.value("name", "");
        ci.id = c.value("id", slug(ci.name));
        ci.system = c.value("system", "");
        ci.platformDiameter = c.value("platformDiameter", 0.0);
        ci.screwChannelDiameter = c.value("screwChannelDiameter", ci.screwChannelDiameter);
        ci.minWall = c.value("minWall", ci.minWall);
        ci.interfaceFile = c.value("interface", "");
        ci.scanBodyFile = c.value("scanBody", "");
        ci.screwChannelFile = c.value("screwChannel", "");
        ci.minThicknessFile = c.value("minThickness", "");
        ci.blankFile = c.value("blank", "");
        if (c.contains("interfaceTop")) {
            const auto& t = c["interfaceTop"];
            if (t.contains("height"))
                ci.interfaceTopHeight = t["height"].get<double>();
            if (t.contains("diameter"))
                ci.interfaceTopDiameter = t["diameter"].get<double>();
        }
        if (ci.name.empty() || ci.interfaceFile.empty() || ci.scanBodyFile.empty())
            throw std::runtime_error("Connection '" + ci.id + "' needs a name, an interface and a scan body.");
        lib.connections.push_back(std::move(ci));
    }
    if (lib.connections.empty())
        throw std::runtime_error("The implant library has no connections.");
    return lib;
}

void writeImplantLibraryManifest(const fs::path& folder, const LibraryInfo& lib)
{
    json j;
    j["format"] = "occlusacad.implantlibrary";
    j["version"] = 1;
    j["id"] = lib.id.empty() ? slug(lib.name) : lib.id;
    j["name"] = lib.name;
    j["manufacturer"] = lib.manufacturer;
    j["author"] = lib.author;
    j["license"] = lib.license;
    j["description"] = lib.description;
    json conns = json::array();
    for (const auto& c : lib.connections) {
        json o;
        o["id"] = c.id;
        o["name"] = c.name;
        o["system"] = c.system;
        o["platformDiameter"] = c.platformDiameter;
        o["screwChannelDiameter"] = c.screwChannelDiameter;
        o["minWall"] = c.minWall;
        o["interface"] = c.interfaceFile;
        o["scanBody"] = c.scanBodyFile;
        for (const auto& [k, v] : {std::pair{"screwChannel", &c.screwChannelFile}, {"minThickness", &c.minThicknessFile}, {"blank", &c.blankFile}})
            if (!v->empty())
                o[k] = *v;
        if (c.interfaceTopHeight || c.interfaceTopDiameter) {
            json t;
            if (c.interfaceTopHeight)
                t["height"] = *c.interfaceTopHeight;
            if (c.interfaceTopDiameter)
                t["diameter"] = *c.interfaceTopDiameter;
            o["interfaceTop"] = t;
        }
        conns.push_back(o);
    }
    j["connections"] = conns;
    fs::create_directories(folder);
    std::ofstream(folder / "library.json") << j.dump(2);
}

bool looksLikeImplantLibrary(const fs::path& folder)
{
    std::error_code ec;
    return fs::exists(folder / "library.json", ec);
}

LibraryInfo readImplantLibrary(const fs::path& folder)
{
    if (fs::exists(folder / "library.json"))
        return readImplantLibraryManifest(folder);
    throw std::runtime_error("No supported implant library in " + platform::pathToUtf8(folder) + ".");
}

void measureInterfaceTop(const Mesh& m, double& height, double& radius)
{
    height = -1e30;
    for (const auto& p : m.positions)
        height = std::max(height, static_cast<double>(p.z));
    radius = 0.0;
    for (const auto& p : m.positions)
        if (p.z > height - 0.05)
            radius = std::max(radius, std::hypot(static_cast<double>(p.x), static_cast<double>(p.y)));
}

// ---------------------------------------------------------------------------
// Generic library
// ---------------------------------------------------------------------------

LibraryInfo genericLibraryInfo()
{
    LibraryInfo lib;
    lib.id = kGenericLibrary;
    lib.name = "OcclusaCAD Generic";
    lib.manufacturer = "OcclusaCAD";
    lib.author = "OcclusaCAD";
    lib.license = "Generated by OcclusaCAD (no third-party content). For testing; not a real implant system.";
    lib.description = "Generic internal-hex connection in three platforms with a matching scan body.";
    lib.builtIn = true;
    for (const auto& s : kGenericSizes) {
        ConnectionInfo c;
        c.id = s.id;
        c.name = s.name;
        c.system = "Generic internal hex";
        c.platformDiameter = s.platform;
        c.screwChannelDiameter = 2.4;
        c.minWall = 0.4;
        c.interfaceFile = std::string(s.id) + "/interface.stl";
        c.scanBodyFile = std::string(s.id) + "/scanbody.stl";
        lib.connections.push_back(c);
    }
    return lib;
}

Mesh genericInterface(double platformDiameter)
{
    double af = 2.7;
    for (const auto& s : kGenericSizes)
        if (std::abs(s.platform - platformDiameter) < 1e-6)
            af = s.hexAcrossFlats;
    const Mesh collar = cylinder(platformDiameter * 0.5, 0.0, kCollarHeight, 96);
    const Mesh hex = prism(hexagon(af), -1.4, 0.01);
    const Mesh shaft = cylinder(0.85, -2.0, 2.0, 48);
    const Mesh seat = cylinder(1.1, -0.2, 2.0, 48);
    BooleanResult r = meshBoolean({&collar, &hex}, {&shaft, &seat});
    if (!r.ok())
        throw std::runtime_error("generic interface: " + r.error);
    return std::move(r.mesh);
}

Mesh genericScanBody(double platformDiameter)
{
    const Mesh body = revolve(scanBodyProfile(platformDiameter), false, 96);
    const Mesh flat = prism({{kScanBodyFlatX, -3.0}, {3.0, -3.0}, {3.0, 3.0}, {kScanBodyFlatX, 3.0}}, kScanBodyFlatZ, kScanBodyHeight + 1.0);
    BooleanResult r = meshBoolean({&body}, {&flat});
    if (!r.ok())
        throw std::runtime_error("generic scan body: " + r.error);
    return std::move(r.mesh);
}

double genericScanBodySdf(const glm::dvec3& p, double platformDiameter)
{
    const double rev = sdPolygon(glm::dvec2(std::hypot(p.x, p.y), p.z), scanBodyProfile(platformDiameter));
    const double inFlat = std::max(kScanBodyFlatX - p.x, kScanBodyFlatZ - p.z); // < 0 inside the removed block
    return std::max(rev, -inFlat);
}

void writeImplantLibrary(const fs::path& folder, LibraryInfo lib, const std::vector<std::shared_ptr<const Connection>>& connections)
{
    fs::create_directories(folder);
    lib.connections.clear();
    for (const auto& c : connections) {
        ConnectionInfo ci = c->info;
        auto put = [&](const std::shared_ptr<const Mesh>& mesh, const char* name, std::string& field) {
            field.clear();
            if (!mesh)
                return;
            field = ci.id + "/" + name;
            fs::create_directories(folder / ci.id);
            writeStlBinary(folder / platform::pathFromUtf8(field), *mesh, glm::dmat4(1.0), lib.name + " " + ci.name + " " + name);
        };
        put(c->interfaceMesh, "interface.stl", ci.interfaceFile);
        put(c->scanBody, "scanbody.stl", ci.scanBodyFile);
        put(c->screwChannel, "screwchannel.stl", ci.screwChannelFile);
        put(c->minThickness, "minthickness.stl", ci.minThicknessFile);
        put(c->blank, "blank.stl", ci.blankFile);
        lib.connections.push_back(ci);
    }
    lib.builtIn = false;
    writeImplantLibraryManifest(folder, lib);
}

// ---------------------------------------------------------------------------
// Registry
// ---------------------------------------------------------------------------

ImplantLibraryRegistry::ImplantLibraryRegistry()
{
    auto e = std::make_shared<Entry>();
    e->info = genericLibraryInfo();
    entries_.push_back(e);
}

ImplantLibraryRegistry& ImplantLibraryRegistry::instance()
{
    static ImplantLibraryRegistry registry;
    return registry;
}

std::vector<LibraryInfo> ImplantLibraryRegistry::libraries() const
{
    std::lock_guard lock(mutex_);
    std::vector<LibraryInfo> out;
    for (const auto& e : entries_)
        out.push_back(e->info);
    return out;
}

std::optional<LibraryInfo> ImplantLibraryRegistry::find(const std::string& id) const
{
    std::lock_guard lock(mutex_);
    for (const auto& e : entries_)
        if (e->info.id == id)
            return e->info;
    return std::nullopt;
}

std::string ImplantLibraryRegistry::addFolder(const fs::path& folder)
{
    auto e = std::make_shared<Entry>();
    e->info = readImplantLibrary(folder);
    std::lock_guard lock(mutex_);
    for (auto& existing : entries_)
        if (existing->info.id == e->info.id) {
            if (existing->info.builtIn)
                throw std::runtime_error("The library id '" + e->info.id + "' is reserved for a built-in library.");
            existing = e;
            return e->info.id;
        }
    entries_.push_back(e);
    return e->info.id;
}

int ImplantLibraryRegistry::scanDirectory(const fs::path& directory)
{
    std::error_code ec;
    if (!fs::is_directory(directory, ec))
        return 0;
    int added = 0;
    for (const auto& entry : fs::directory_iterator(directory, ec)) {
        if (!entry.is_directory() || !looksLikeImplantLibrary(entry.path()))
            continue;
        try {
            addFolder(entry.path());
            ++added;
        } catch (const std::exception& ex) {
            log::warn("Implant library {} skipped: {}", platform::pathToUtf8(entry.path()), ex.what());
        }
    }
    return added;
}

std::shared_ptr<const Connection> ImplantLibraryRegistry::connection(const std::string& libraryId, const std::string& connectionId)
{
    std::shared_ptr<Entry> e;
    {
        std::lock_guard lock(mutex_);
        for (const auto& x : entries_)
            if (x->info.id == libraryId)
                e = x;
        if (!e)
            throw std::runtime_error("Implant library '" + libraryId + "' is not installed.");
        if (auto it = e->loaded.find(connectionId); it != e->loaded.end())
            return it->second;
    }
    const ConnectionInfo* ci = e->info.connection(connectionId);
    if (!ci)
        throw std::runtime_error("Implant library '" + e->info.name + "' has no connection '" + connectionId + "'.");
    auto c = std::make_shared<Connection>();
    c->info = *ci;
    if (e->info.builtIn) {
        c->interfaceMesh = std::make_shared<const Mesh>(genericInterface(ci->platformDiameter));
        c->scanBody = std::make_shared<const Mesh>(genericScanBody(ci->platformDiameter));
    } else {
        c->interfaceMesh = loadOptional(e->info.folder, ci->interfaceFile, true);
        c->scanBody = loadOptional(e->info.folder, ci->scanBodyFile, false);
        c->screwChannel = loadOptional(e->info.folder, ci->screwChannelFile, true);
        c->minThickness = loadOptional(e->info.folder, ci->minThicknessFile, true);
        c->blank = loadOptional(e->info.folder, ci->blankFile, true);
    }
    measureInterfaceTop(*c->interfaceMesh, c->topHeight, c->topRadius);
    if (ci->interfaceTopHeight)
        c->topHeight = *ci->interfaceTopHeight;
    if (ci->interfaceTopDiameter)
        c->topRadius = *ci->interfaceTopDiameter * 0.5;
    c->interfaceClosed = isClosedManifold(*c->interfaceMesh);
    if (c->topRadius <= 0.1)
        throw std::runtime_error("The interface of '" + ci->name + "' has no usable top circle.");
    std::lock_guard lock(mutex_);
    e->loaded[connectionId] = c;
    return c;
}

} // namespace occlusa::implant
