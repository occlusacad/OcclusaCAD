#include "core/crown/Bridge.h"

#include "core/Dental.h"
#include "core/Geometry.h"
#include "core/MeshTopology.h"
#include "core/crown/InsertionAxis.h"
#include "core/crown/ToothLibrary.h"

#include <manifold/manifold.h>

#include <algorithm>
#include <cmath>
#include <format>
#include <map>
#include <numbers>
#include <optional>
#include <unordered_map>

namespace occlusa::crown {

// ---------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------

int archIndex(int fdi)
{
    const auto& arch = dental::isUpper(fdi) ? dental::upperArch() : dental::lowerArch();
    const auto it = std::find(arch.begin(), arch.end(), fdi);
    return it == arch.end() ? -100 : static_cast<int>(it - arch.begin());
}

bool adjacentTeeth(int a, int b)
{
    return dental::isValidFdi(a) && dental::isValidFdi(b) && dental::isUpper(a) == dental::isUpper(b) && std::abs(archIndex(a) - archIndex(b)) == 1;
}

bool isAbutmentType(const std::string& type)
{
    return type == "anatomic_crown" || type == "coping";
}

bool isPonticType(const std::string& type)
{
    return type == "pontic";
}

std::vector<std::vector<int>> findBridges(const std::vector<std::pair<int, std::string>>& restorations)
{
    std::map<int, std::string> units;
    for (const auto& [tooth, type] : restorations)
        if (dental::isValidFdi(tooth) && (isAbutmentType(type) || isPonticType(type)))
            units[tooth] = type;
    std::vector<std::vector<int>> out;
    for (const bool upper : {true, false}) {
        std::vector<int> run;
        auto flush = [&] {
            bool pontic = false, abutment = false;
            for (int t : run) {
                pontic |= isPonticType(units[t]);
                abutment |= isAbutmentType(units[t]);
            }
            if (run.size() >= 2 && pontic && abutment)
                out.push_back(run);
            run.clear();
        };
        for (int fdi : upper ? dental::upperArch() : dental::lowerArch()) {
            if (units.count(fdi)) {
                run.push_back(fdi);
            } else {
                flush();
            }
        }
        flush();
    }
    return out;
}

namespace {

double templateWidth(int fdi)
{
    return toothTemplate(toothKindFromFdi(fdi), dental::isUpper(fdi)).mesioDistal;
}

glm::dvec3 flatten(const glm::dvec3& v, const glm::dvec3& axis)
{
    return v - glm::dot(v, axis) * axis;
}

// Is tooth a mesial of tooth b (closer to the midline along the arch)?
bool isMesialOf(int a, int b)
{
    if (dental::quadrantOf(a) != dental::quadrantOf(b))
        return dental::positionInQuadrant(a) <= dental::positionInQuadrant(b);
    return dental::positionInQuadrant(a) < dental::positionInQuadrant(b);
}

} // namespace

bool layoutPontics(std::vector<BridgeLayoutUnit>& units, const glm::dvec3& axisIn)
{
    const glm::dvec3 axis = glm::normalize(axisIn);
    std::vector<std::size_t> abutments;
    for (std::size_t i = 0; i < units.size(); ++i)
        if (!units[i].pontic)
            abutments.push_back(i);
    if (abutments.empty())
        return false;
    for (std::size_t i = 0; i < units.size(); ++i) {
        if (!units[i].pontic)
            continue;
        // Nearest abutments on either side (in arch order).
        std::optional<std::size_t> left, right;
        for (std::size_t a : abutments) {
            if (a < i)
                left = a;
            if (a > i && !right)
                right = a;
        }
        if (left && right) {
            // Distribute the span between the abutments by template widths.
            double total = templateWidth(units[*left].tooth) * 0.5 + templateWidth(units[*right].tooth) * 0.5;
            double before = templateWidth(units[*left].tooth) * 0.5;
            for (std::size_t k = *left + 1; k < *right; ++k) {
                total += templateWidth(units[k].tooth);
                if (k < i)
                    before += templateWidth(units[k].tooth);
            }
            before += templateWidth(units[i].tooth) * 0.5;
            const double t = before / total;
            units[i].center = glm::mix(units[*left].center, units[*right].center, t);
        } else {
            // Cantilever: continue along the arch from the abutment.
            const std::size_t a = left ? *left : *right;
            glm::dvec3 dir;
            if (abutments.size() >= 2) {
                const std::size_t other = left ? abutments[abutments.size() - 2] : abutments[1];
                dir = flatten(units[a].center - units[other].center, axis);
            } else {
                dir = flatten(units[a].mesial, axis);
                if (!isMesialOf(units[i].tooth, units[a].tooth))
                    dir = -dir;
            }
            if (glm::length(dir) < 1e-9)
                return false;
            dir = glm::normalize(dir);
            double dist = templateWidth(units[a].tooth) * 0.5;
            const std::size_t from = std::min(a, i), to = std::max(a, i);
            for (std::size_t k = from + 1; k < to; ++k)
                dist += templateWidth(units[k].tooth);
            dist += templateWidth(units[i].tooth) * 0.5;
            units[i].center = units[a].center + dir * dist;
        }
    }
    return true;
}

glm::dvec3 ponticMesial(const std::vector<BridgeLayoutUnit>& units, std::size_t i, const glm::dvec3& axisIn)
{
    const glm::dvec3 axis = glm::normalize(axisIn);
    glm::dvec3 dir(0.0);
    // Along the line through the neighbouring units, pointing to the more mesial tooth.
    const std::size_t a = i > 0 ? i - 1 : i, b = i + 1 < units.size() ? i + 1 : i;
    if (a != b) {
        dir = flatten(units[b].center - units[a].center, axis);
        if (isMesialOf(units[a].tooth, units[b].tooth))
            dir = -dir;
    }
    if (glm::length(dir) < 1e-9)
        return glm::dvec3(0.0);
    return glm::normalize(dir);
}

glm::dvec3 optimizeCommonAxis(const std::vector<const Mesh*>& dies, const glm::dvec3& initial, double maxTiltDeg)
{
    Mesh merged;
    for (const Mesh* d : dies) {
        const auto base = static_cast<std::uint32_t>(merged.positions.size());
        merged.positions.insert(merged.positions.end(), d->positions.begin(), d->positions.end());
        for (std::uint32_t i : d->indices)
            merged.indices.push_back(base + i);
    }
    return optimizeInsertionAxis(merged, initial, maxTiltDeg);
}

double axisDivergence(const glm::dvec3& a, const glm::dvec3& b)
{
    return glm::degrees(std::acos(std::clamp(glm::dot(glm::normalize(a), glm::normalize(b)), -1.0, 1.0)));
}

// ---------------------------------------------------------------------------
// Pontic
// ---------------------------------------------------------------------------

std::optional<DieRegion> makePonticBase(const MeshBvh& scan, const glm::dvec3& center, const glm::dvec3& axisIn, const glm::dvec3& mesialIn,
                                        double footprintMD, double footprintBL, double ridgeOffset, std::string* error)
{
    const glm::dvec3 A = glm::normalize(axisIn);
    glm::dvec3 M = flatten(mesialIn, A);
    if (glm::length(M) < 1e-9) {
        if (error)
            *error = "The pontic has no mesial direction.";
        return std::nullopt;
    }
    M = glm::normalize(M);
    const glm::dvec3 B = glm::cross(A, M);
    constexpr int kRings = 8, kCols = 96;
    DieRegion die;
    Mesh& m = die.mesh;
    auto project = [&](const glm::dvec3& p) -> std::optional<glm::vec3> {
        if (auto hit = scan.raycast(glm::vec3(p + A * 25.0), glm::vec3(-A), 60.0f))
            return glm::vec3(glm::dvec3(hit->point) + A * ridgeOffset);
        return std::nullopt;
    };
    const auto c = project(center);
    if (!c) {
        if (error)
            *error = "There is no gingiva under the pontic.";
        return std::nullopt;
    }
    m.positions.push_back(*c);
    for (int k = 1; k <= kRings; ++k) {
        const double s = static_cast<double>(k) / kRings;
        for (int j = 0; j < kCols; ++j) {
            const double t = 2.0 * std::numbers::pi * j / kCols;
            const auto p = project(center + M * (std::cos(t) * footprintMD * s) + B * (std::sin(t) * footprintBL * s));
            if (!p) {
                if (error)
                    *error = "The ridge under the pontic is incomplete in the scan.";
                return std::nullopt;
            }
            m.positions.push_back(*p);
        }
    }
    auto idx = [&](int ring, int col) { return static_cast<std::uint32_t>(1 + (ring - 1) * kCols + ((col % kCols) + kCols) % kCols); };
    for (int j = 0; j < kCols; ++j)
        m.indices.insert(m.indices.end(), {0u, idx(1, j), idx(1, j + 1)});
    for (int k = 1; k < kRings; ++k)
        for (int j = 0; j < kCols; ++j) {
            const auto a = idx(k, j), b = idx(k, j + 1), cc = idx(k + 1, j + 1), d = idx(k + 1, j);
            m.indices.insert(m.indices.end(), {a, d, cc, a, cc, b});
        }
    // Face the pontic (the ridge's outward side).
    const glm::vec3 n0 = glm::cross(m.positions[m.indices[1]] - m.positions[m.indices[0]], m.positions[m.indices[2]] - m.positions[m.indices[0]]);
    if (glm::dot(glm::dvec3(n0), A) < 0.0)
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
            std::swap(m.indices[t + 1], m.indices[t + 2]);
    m.computeVertexNormals();
    for (int j = 0; j < kCols; ++j)
        die.boundary.push_back(idx(kRings, j));
    return die;
}

CrownParameters ponticParameters(CrownParameters p)
{
    p.coping = false;
    p.cementGap = 0.0;
    p.extraGap = 0.0;
    p.distanceToMargin = 0.0;
    p.blockOutUndercuts = false;
    p.marginThickness = std::max(p.marginThickness, 0.6); // rounded basal edge
    return p;
}

void fitBridgeWidths(std::vector<BridgeFitUnit>& units, double overlap)
{
    for (std::size_t i = 0; i < units.size(); ++i) {
        const CrownAxes ax = crownAxes(units[i].frame, *units[i].params);
        for (const std::size_t j : {i - 1, i + 1}) {
            if (j >= units.size())
                continue;
            const glm::dvec3 d = flatten(units[j].frame.origin - units[i].frame.origin, ax.A);
            const double half = glm::length(d) * 0.5 + overlap;
            if (glm::dot(d, ax.M) > 0.0)
                units[i].params->halfMesial = half;
            else
                units[i].params->halfDistal = half;
        }
    }
}

// ---------------------------------------------------------------------------
// Connectors
// ---------------------------------------------------------------------------

double ConnectorSpec::width() const
{
    return std::sqrt(4.0 * area / (std::numbers::pi * heightRatio));
}

double ConnectorSpec::height() const
{
    return width() * heightRatio;
}

ConnectorSpec placeConnector(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& upIn, const glm::dvec3& reference, double bottom, double top,
                             double area, double heightRatio, std::string* warning)
{
    ConnectorSpec c;
    c.up = glm::normalize(upIn);
    c.area = area;
    c.heightRatio = heightRatio;
    const glm::dvec3 span = flatten(b - a, c.up);
    c.direction = glm::length(span) > 1e-9 ? glm::normalize(span) : glm::dvec3(1, 0, 0);
    c.length = std::max(glm::length(span) * 0.62, 2.0);
    const double h = c.height();
    double hc = 0.5 * (bottom + top);
    if (top - bottom < h) {
        if (warning)
            *warning = std::format("The connector ({:.1f} mm high) does not fit the {:.1f} mm between the gingival embrasure and the "
                                   "marginal ridges.",
                                   h, std::max(top - bottom, 0.0));
    } else {
        hc = std::clamp(hc, bottom + 0.5 * h, top - 0.5 * h);
    }
    const glm::dvec3 mid = 0.5 * (a + b);
    c.center = mid + c.up * (hc - glm::dot(mid - reference, c.up));
    return c;
}

void connectorFrame(const ConnectorSpec& c, glm::dvec3& x, glm::dvec3& y, glm::dvec3& z)
{
    x = glm::normalize(c.direction);
    z = c.up - glm::dot(c.up, x) * x;
    z = glm::length(z) > 1e-9 ? glm::normalize(z) : glm::dvec3(0, 0, 1);
    y = glm::cross(z, x);
}

ConnectorSpec applyConnectorEdit(ConnectorSpec spec, const ConnectorEdit& edit)
{
    glm::dvec3 x, y, z;
    connectorFrame(spec, x, y, z);
    spec.center += x * edit.offset.x + y * edit.offset.y + z * edit.offset.z;
    if (edit.area > 0.0)
        spec.area = edit.area;
    if (edit.heightRatio > 0.0)
        spec.heightRatio = edit.heightRatio;
    if (edit.length > 0.0)
        spec.length = edit.length;
    return spec;
}

std::optional<std::string> checkConnectorFit(const ConnectorSpec& c, const glm::dvec3& reference, double bottom, double top)
{
    const double h = c.height();
    const double centre = glm::dot(c.center - reference, glm::normalize(c.up));
    const double below = bottom - (centre - 0.5 * h);
    const double above = (centre + 0.5 * h) - top;
    constexpr double tol = 1e-3;
    if (h > top - bottom + tol)
        return std::format("{:.1f} mm high, but only {:.1f} mm between the gingival embrasure and the marginal ridges", h, std::max(top - bottom, 0.0));
    if (below > tol)
        return std::format("reaches {:.1f} mm into the gingival embrasure", below);
    if (above > tol)
        return std::format("rises {:.1f} mm above the marginal ridges", above);
    return std::nullopt;
}

Mesh makeConnectorMesh(const ConnectorSpec& c, int segments)
{
    // Ellipsoid: semi-axes length/2 along the direction, width/2 bucco-lingually, height/2 along up.
    glm::dvec3 x, y, z;
    connectorFrame(c, x, y, z);
    const int rings = std::max(segments / 2, 6), sectors = std::max(segments, 8);
    // A polygon inscribed in the ellipse has less area; scale up so the requested area is a minimum.
    const double inscribed = sectors / (2.0 * std::numbers::pi) * std::sin(2.0 * std::numbers::pi / sectors);
    const double grow = 1.0 / std::sqrt(inscribed);
    const double ax = c.length * 0.5, ay = c.width() * 0.5 * grow, az = c.height() * 0.5 * grow;
    Mesh m;
    m.positions.emplace_back(c.center + x * ax);
    for (int r = 1; r < rings; ++r) {
        const double phi = std::numbers::pi * r / rings;
        for (int s = 0; s < sectors; ++s) {
            const double th = 2.0 * std::numbers::pi * s / sectors;
            m.positions.emplace_back(c.center + x * (ax * std::cos(phi)) + y * (ay * std::sin(phi) * std::cos(th)) + z * (az * std::sin(phi) * std::sin(th)));
        }
    }
    m.positions.emplace_back(c.center - x * ax);
    const auto south = static_cast<std::uint32_t>(m.positions.size() - 1);
    auto idx = [&](int r, int s) { return static_cast<std::uint32_t>(1 + (r - 1) * sectors + (s % sectors)); };
    for (int s = 0; s < sectors; ++s)
        m.indices.insert(m.indices.end(), {0u, idx(1, s), idx(1, s + 1)});
    for (int r = 1; r < rings - 1; ++r)
        for (int s = 0; s < sectors; ++s)
            m.indices.insert(m.indices.end(), {idx(r, s), idx(r + 1, s), idx(r + 1, s + 1), idx(r, s), idx(r + 1, s + 1), idx(r, s + 1)});
    for (int s = 0; s < sectors; ++s)
        m.indices.insert(m.indices.end(), {idx(rings - 1, s), south, idx(rings - 1, s + 1)});
    if (signedVolume(m) < 0.0)
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
            std::swap(m.indices[t + 1], m.indices[t + 2]);
    m.computeVertexNormals();
    return m;
}

Mesh makeCavity(const CrownBase& base, double grow)
{
    const Mesh& in = *base.intaglio;
    const glm::dvec3 A = glm::normalize(base.frame.axis);
    Mesh m;
    m.positions.resize(in.positions.size());
    for (std::size_t i = 0; i < in.positions.size(); ++i)
        m.positions[i] = in.positions[i] + in.normals[i] * static_cast<float>(grow);
    m.indices = in.indices;
    const auto& bnd = base.die.boundary;
    const std::size_t L = bnd.size();
    bool forward = false; // does the intaglio use the margin edge b0 -> b1?
    for (std::size_t t = 0; t + 2 < in.indices.size() && !forward; t += 3)
        for (int e = 0; e < 3; ++e)
            if (in.indices[t + e] == bnd[0] && in.indices[t + (e + 1) % 3] == bnd[1])
                forward = true;
    const auto skirt0 = static_cast<std::uint32_t>(m.positions.size());
    glm::dvec3 c(0.0);
    for (std::size_t i = 0; i < L; ++i) {
        m.positions.push_back(glm::vec3(glm::dvec3(m.positions[bnd[i]]) - A * 3.0));
        c += glm::dvec3(m.positions[bnd[i]]);
    }
    c /= static_cast<double>(L);
    const auto bottom = static_cast<std::uint32_t>(m.positions.size());
    m.positions.push_back(glm::vec3(c - A * 3.0));
    auto tri = [&](std::uint32_t a, std::uint32_t b, std::uint32_t d) {
        if (forward)
            m.indices.insert(m.indices.end(), {a, b, d});
        else
            m.indices.insert(m.indices.end(), {a, d, b});
    };
    for (std::size_t i = 0; i < L; ++i) {
        const std::size_t j = (i + 1) % L;
        const auto bi = bnd[i], bj = bnd[j];
        const auto si = static_cast<std::uint32_t>(skirt0 + i), sj = static_cast<std::uint32_t>(skirt0 + j);
        tri(bj, bi, si);
        tri(bj, si, sj);
        tri(sj, si, bottom);
    }
    if (signedVolume(m) < 0.0)
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
            std::swap(m.indices[t + 1], m.indices[t + 2]);
    m.computeVertexNormals();
    return m;
}

// ---------------------------------------------------------------------------
// Union
// ---------------------------------------------------------------------------

namespace {

manifold::Manifold toManifold(const Mesh& mesh)
{
    manifold::MeshGL gl;
    gl.numProp = 3;
    gl.vertProperties.reserve(mesh.positions.size() * 3);
    for (const auto& p : mesh.positions)
        gl.vertProperties.insert(gl.vertProperties.end(), {p.x, p.y, p.z});
    gl.triVerts = mesh.indices;
    return manifold::Manifold(gl);
}

Mesh fromManifold(const manifold::Manifold& man)
{
    const manifold::MeshGL gl = man.GetMeshGL();
    Mesh m;
    const std::size_t n = gl.vertProperties.size() / gl.numProp;
    m.positions.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
        m.positions.emplace_back(gl.vertProperties[i * gl.numProp], gl.vertProperties[i * gl.numProp + 1], gl.vertProperties[i * gl.numProp + 2]);
    m.indices.assign(gl.triVerts.begin(), gl.triVerts.end());
    m.computeVertexNormals();
    return m;
}

std::string statusText(manifold::Manifold::Error e)
{
    using E = manifold::Manifold::Error;
    switch (e) {
    case E::NoError: return "no error";
    case E::NotManifold: return "a part is not a closed manifold";
    case E::NonFiniteVertex: return "a part has invalid coordinates";
    default: return std::format("geometry error {}", static_cast<int>(e));
    }
}

} // namespace

BridgeUnion uniteBridge(const std::vector<BridgeUnionPart>& units, const std::vector<ConnectorSpec>& connectors, const std::vector<Mesh>& cavities)
{
    BridgeUnion out;
    try {
        std::vector<manifold::Manifold> cav;
        for (const auto& c : cavities) {
            cav.push_back(toManifold(c));
            if (cav.back().Status() != manifold::Manifold::Error::NoError) {
                out.error = "Preparation space: " + statusText(cav.back().Status());
                return out;
            }
        }
        // Union of all cavities except one (or all, for -1).
        auto cavitiesExcept = [&](int skip) -> std::optional<manifold::Manifold> {
            std::vector<manifold::Manifold> list;
            for (std::size_t i = 0; i < cav.size(); ++i)
                if (static_cast<int>(i) != skip)
                    list.push_back(cav[i]);
            if (list.empty())
                return std::nullopt;
            return manifold::Manifold::BatchBoolean(list, manifold::OpType::Add);
        };
        std::vector<manifold::Manifold> parts;
        for (std::size_t i = 0; i < units.size(); ++i) {
            manifold::Manifold m = toManifold(*units[i].mesh);
            if (m.Status() != manifold::Manifold::Error::NoError) {
                out.error = std::format("Unit {}: {}", i + 1, statusText(m.Status()));
                return out;
            }
            if (auto others = cavitiesExcept(units[i].ownCavity))
                m = m - *others;
            parts.push_back(m);
        }
        const auto allCavities = cavitiesExcept(-1);
        for (const auto& c : connectors) {
            manifold::Manifold con = toManifold(makeConnectorMesh(c));
            if (allCavities)
                con = con - *allCavities; // never fill the space of a preparation
            parts.push_back(con);
        }
        const manifold::Manifold all = manifold::Manifold::BatchBoolean(parts, manifold::OpType::Add);
        if (all.Status() != manifold::Manifold::Error::NoError) {
            out.error = statusText(all.Status());
            return out;
        }
        out.mesh = fromManifold(all);
        out.volume = all.Volume();
        out.watertight = isClosedManifold(out.mesh);
        out.ok = !out.mesh.empty();
        for (const auto& c : connectors)
            out.connectorAreas.push_back(crossSectionArea(out.mesh, Plane{c.center, c.direction}, c.center, std::max(c.width(), c.height()) * 1.5));
    } catch (const std::exception& e) {
        out.ok = false;
        out.error = e.what();
    }
    return out;
}

double crossSectionArea(const Mesh& mesh, const Plane& plane, const glm::dvec3& near, double radius)
{
    const auto segs = slicePlane(mesh, glm::dmat4(1.0), plane);
    if (segs.empty())
        return 0.0;
    // Chain segments into loops through shared (quantised) end points.
    auto key = [](const glm::vec3& p) {
        const auto q = [](float v) { return static_cast<std::int64_t>(std::llround(static_cast<double>(v) * 1e4)); };
        return std::to_string(q(p.x)) + "," + std::to_string(q(p.y)) + "," + std::to_string(q(p.z));
    };
    std::unordered_map<std::string, std::vector<std::size_t>> at;
    for (std::size_t i = 0; i < segs.size(); ++i) {
        at[key(segs[i].a)].push_back(i);
        at[key(segs[i].b)].push_back(i);
    }
    glm::dvec3 u = std::abs(plane.normal.x) < 0.9 ? glm::dvec3(1, 0, 0) : glm::dvec3(0, 1, 0);
    u = glm::normalize(glm::cross(plane.normal, u));
    const glm::dvec3 v = glm::cross(plane.normal, u);
    auto to2d = [&](const glm::vec3& p) {
        const glm::dvec3 d = glm::dvec3(p) - plane.point;
        return glm::dvec2(glm::dot(d, u), glm::dot(d, v));
    };
    std::vector<std::vector<glm::dvec2>> loops;
    std::vector<std::uint8_t> used(segs.size(), 0);
    for (std::size_t s = 0; s < segs.size(); ++s) {
        if (used[s])
            continue;
        std::vector<glm::dvec2> loop;
        used[s] = 1;
        loop.push_back(to2d(segs[s].a));
        glm::vec3 cur = segs[s].b;
        const std::string start = key(segs[s].a);
        for (std::size_t guard = 0; guard < segs.size(); ++guard) {
            const std::string k = key(cur);
            if (k == start)
                break;
            loop.push_back(to2d(cur));
            bool advanced = false;
            for (std::size_t n : at[k]) {
                if (used[n])
                    continue;
                used[n] = 1;
                cur = key(segs[n].a) == k ? segs[n].b : segs[n].a;
                advanced = true;
                break;
            }
            if (!advanced)
                break;
        }
        if (loop.size() >= 3)
            loops.push_back(std::move(loop));
    }
    auto area = [](const std::vector<glm::dvec2>& l) {
        double a = 0.0;
        for (std::size_t i = 0; i < l.size(); ++i) {
            const auto& p = l[i];
            const auto& q = l[(i + 1) % l.size()];
            a += p.x * q.y - q.x * p.y;
        }
        return 0.5 * a;
    };
    auto inside = [](const glm::dvec2& p, const std::vector<glm::dvec2>& l) {
        bool in = false;
        for (std::size_t i = 0, j = l.size() - 1; i < l.size(); j = i++)
            if ((l[i].y > p.y) != (l[j].y > p.y) && p.x < (l[j].x - l[i].x) * (p.y - l[i].y) / (l[j].y - l[i].y) + l[i].x)
                in = !in;
        return in;
    };
    const glm::dvec2 c2 = to2d(glm::vec3(near));
    double total = 0.0;
    for (std::size_t i = 0; i < loops.size(); ++i) {
        glm::dvec2 cen(0.0);
        for (const auto& p : loops[i])
            cen += p;
        cen /= static_cast<double>(loops[i].size());
        if (glm::length(cen - c2) > radius)
            continue;
        int depth = 0;
        for (std::size_t j = 0; j < loops.size(); ++j)
            if (j != i && inside(loops[i].front(), loops[j]))
                ++depth;
        total += (depth % 2 == 0 ? 1.0 : -1.0) * std::abs(area(loops[i]));
    }
    return total;
}

} // namespace occlusa::crown
