#include "core/implant/Abutment.h"

#include "core/MeshBoolean.h"
#include "core/MeshBvh.h"

#include <cmath>
#include <format>
#include <numbers>

namespace occlusa::implant {

namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;
constexpr int kAzimuths = 108;      // columns around (12 per control point)
constexpr int kEmergenceRows = 18;
constexpr int kCoreRows = 12;
constexpr int kTopRows = 6;
constexpr double kOverlap = 0.08;   // the designed part reaches this far into the interface
constexpr double kMinMarginHeight = 0.2;

double wrap(double a)
{
    a = std::fmod(a, kTwoPi);
    return a < 0.0 ? a + kTwoPi : a;
}

// Ray-parity point-in-solid test against a BVH.
bool inside(const MeshBvh& bvh, const glm::vec3& p)
{
    const glm::vec3 dir = glm::normalize(glm::vec3(0.5773f, 0.6123f, 0.5404f)); // avoids axis-aligned degeneracies
    glm::vec3 o = p;
    int hits = 0;
    for (int i = 0; i < 64; ++i) {
        auto h = bvh.raycast(o, dir);
        if (!h)
            break;
        ++hits;
        o = h->point + dir * 1e-4f;
    }
    return hits % 2 == 1;
}

} // namespace

double AbutmentShape::azimuth(int k) const
{
    return phase + kTwoPi * k / kControlPoints;
}

double interpolateControls(const std::array<double, kControlPoints>& v, double azimuth, double phase)
{
    const double u = wrap(azimuth - phase) / (kTwoPi / kControlPoints);
    const int i = static_cast<int>(std::floor(u)) % kControlPoints;
    const double t = u - std::floor(u);
    auto at = [&](int k) { return v[static_cast<std::size_t>(((k % kControlPoints) + kControlPoints) % kControlPoints)]; };
    const double p0 = at(i - 1), p1 = at(i), p2 = at(i + 1), p3 = at(i + 2);
    const double t2 = t * t, t3 = t2 * t;
    return 0.5 * ((2.0 * p1) + (-p0 + p2) * t + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
}

glm::dvec2 AbutmentShape::corePoint(int k, double minR) const
{
    const auto i = static_cast<std::size_t>(k);
    if (!coreLocked)
        return {std::max(coreRadius[i], minR), std::max(coreTop[i], marginHeight[i] + 0.5)};
    const double base = marginRadius[i] - shoulderWidth;
    const double r = base - coreHeight * std::tan(glm::radians(taperDeg));
    return {std::max(r, minR), marginHeight[i] + coreHeight};
}

void AbutmentShape::unlockCore(double minR)
{
    for (int k = 0; k < kControlPoints; ++k) {
        const glm::dvec2 c = corePoint(k, minR);
        coreRadius[static_cast<std::size_t>(k)] = c.x;
        coreTop[static_cast<std::size_t>(k)] = c.y;
    }
    coreLocked = false;
}

AbutmentShape defaultAbutmentShape(const Connection& c, double marginHeight)
{
    AbutmentShape s;
    s.marginRadius.fill(c.topRadius + 0.25);
    s.marginHeight.fill(std::max(marginHeight, kMinMarginHeight));
    s.midOffset.fill(0.0);
    s.screwChannelDiameter = c.info.screwChannelDiameter;
    s.unlockCore(minCoreRadius(s, c));
    s.coreLocked = true;
    return s;
}

double minCoreRadius(const AbutmentShape& s, const Connection& c)
{
    return s.screwChannelDiameter * 0.5 + std::max(c.info.minWall, 0.2);
}

AbutmentGeometry buildAbutment(const AbutmentShape& s, const Connection& c)
{
    AbutmentGeometry g;
    const double R0 = c.topRadius, z0 = c.topHeight;
    const double rc = s.screwChannelDiameter * 0.5;
    const double minCoreR = minCoreRadius(s, c);
    const double geomMinR = rc + 0.15; // geometric validity (the wall check reports thin areas)

    // Per-azimuth parameters, interpolated from the control points.
    std::array<double, kControlPoints> coreR{}, coreZ{};
    // A core that follows the margin keeps the minimum wall; a freely placed one is only kept valid
    // (and reported when thin).
    for (int k = 0; k < kControlPoints; ++k) {
        const glm::dvec2 cp = s.corePoint(k, s.coreLocked ? minCoreR : geomMinR);
        coreR[static_cast<std::size_t>(k)] = cp.x;
        coreZ[static_cast<std::size_t>(k)] = cp.y;
    }

    auto profile = [&](double a, glm::dvec2* midOut) {
        std::vector<glm::dvec2> p;
        const double rm = std::max(interpolateControls(s.marginRadius, a, s.phase), geomMinR + 0.1);
        const double hm = std::max(interpolateControls(s.marginHeight, a, s.phase), kMinMarginHeight);
        const double d = interpolateControls(s.midOffset, a, s.phase);
        const glm::dvec2 B(R0, 0.0), M(rm, hm);
        const glm::dvec2 dir = glm::normalize(M - B);
        const glm::dvec2 n(dir.y, -dir.x); // outward
        const glm::dvec2 P = (B + M) * 0.5 + n * d;
        if (midOut)
            *midOut = P;
        const glm::dvec2 C = 2.0 * P - (B + M) * 0.5; // quadratic Bezier through P at t = 0.5
        for (int i = 0; i <= kEmergenceRows; ++i) {
            const double t = static_cast<double>(i) / kEmergenceRows;
            p.push_back((1 - t) * (1 - t) * B + 2 * (1 - t) * t * C + t * t * M);
        }
        // Shoulder with a rounded inner corner.
        const double w = std::clamp(s.shoulderWidth, 0.0, rm - geomMinR);
        const double rho = std::min(0.3, w * 0.5);
        const double cr = std::max(std::max(interpolateControls(coreR, a, s.phase), geomMinR), 0.0);
        const double cz = std::max(interpolateControls(coreZ, a, s.phase), hm + rho + 0.3);
        const glm::dvec2 S(rm - w, hm);
        p.push_back({rm - (w - rho) * 0.5, hm});
        for (int i = 0; i <= 3; ++i) {
            const double phi = (std::numbers::pi * 0.5) * i / 3.0;
            p.push_back({S.x + rho - rho * std::sin(phi), hm + rho - rho * std::cos(phi)});
        }
        // Core wall.
        const glm::dvec2 W0 = p.back(), T(std::min(cr, W0.x + 2.0), cz);
        for (int i = 1; i <= kCoreRows; ++i)
            p.push_back(glm::mix(W0, T, static_cast<double>(i) / kCoreRows));
        // Rounded top down to the screw channel.
        for (int i = 1; i <= kTopRows; ++i) {
            const double u = static_cast<double>(i) / kTopRows;
            p.push_back({T.x + (rc - T.x) * u, T.y + 0.35 * (1.0 - (1.0 - u) * (1.0 - u))});
        }
        // Screw channel down into the interface, then the bottom inside the interface.
        p.push_back({rc, -kOverlap});
        p.push_back({R0 - kOverlap, -kOverlap});
        return p;
    };

    // Build the closed grid.
    std::vector<std::vector<glm::dvec2>> cols(kAzimuths);
    for (int j = 0; j < kAzimuths; ++j)
        cols[static_cast<std::size_t>(j)] = profile(s.phase + kTwoPi * j / kAzimuths, nullptr);
    const std::size_t rows = cols[0].size();
    const std::size_t outerRows = kEmergenceRows + 1 + 5 + kCoreRows + kTopRows; // up to the channel edge
    Mesh& m = g.designed;
    m.positions.resize(rows * kAzimuths);
    for (std::size_t r = 0; r < rows; ++r)
        for (int j = 0; j < kAzimuths; ++j) {
            const double a = s.phase + kTwoPi * j / kAzimuths;
            const glm::dvec2 q = cols[static_cast<std::size_t>(j)][r];
            m.positions[r * kAzimuths + static_cast<std::size_t>(j)] = glm::vec3(q.x * std::cos(a), q.x * std::sin(a), z0 + q.y);
        }
    for (std::size_t r = 0; r < rows; ++r) {
        const std::size_t r2 = (r + 1) % rows;
        for (int j = 0; j < kAzimuths; ++j) {
            const auto j2 = static_cast<std::size_t>((j + 1) % kAzimuths);
            const auto a = static_cast<std::uint32_t>(r * kAzimuths + static_cast<std::size_t>(j)), b = static_cast<std::uint32_t>(r * kAzimuths + j2);
            const auto cIdx = static_cast<std::uint32_t>(r2 * kAzimuths + j2), d = static_cast<std::uint32_t>(r2 * kAzimuths + static_cast<std::size_t>(j));
            m.indices.insert(m.indices.end(), {a, cIdx, b, a, d, cIdx});
        }
    }
    if (signedVolume(m) < 0.0)
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
            std::swap(m.indices[t + 1], m.indices[t + 2]);
    m.computeVertexNormals();
    g.outerRowEnd = outerRows * kAzimuths;

    // Wall thickness around the screw channel.
    g.wall.assign(m.positions.size(), 1e3f);
    g.minWall = 1e9;
    // (emergence, shoulder and core; the top is above the channel, not beside it)
    for (std::size_t v = 0; v < (outerRows - kTopRows) * kAzimuths; ++v) {
        const glm::vec3& p = m.positions[v];
        if (p.z < z0 - 1e-4)
            continue;
        const double w = std::hypot(p.x, p.y) - rc;
        g.wall[v] = static_cast<float>(w);
        g.minWall = std::min(g.minWall, w);
    }

    // Handles.
    for (int k = 0; k < kControlPoints; ++k) {
        const double a = s.azimuth(k);
        const glm::dvec3 ux(std::cos(a), std::sin(a), 0.0);
        glm::dvec2 mid;
        const auto prof = profile(a, &mid);
        const glm::dvec2 M = prof[kEmergenceRows];
        const auto i = static_cast<std::size_t>(k);
        g.marginPoints[i] = ux * M.x + glm::dvec3(0, 0, z0 + M.y);
        g.midPoints[i] = ux * mid.x + glm::dvec3(0, 0, z0 + mid.y);
        const glm::dvec2 T = prof[outerRows - kTopRows - 1];
        g.corePoints[i] = ux * T.x + glm::dvec3(0, 0, z0 + T.y);
    }

    // Design checks.
    const double minWall = c.info.minWall;
    if (g.minWall < minWall - 1e-3)
        g.warnings.push_back(std::format("Wall around the screw channel is {:.2f} mm (minimum {:.2f} mm).", g.minWall, minWall));
    for (int k = 0; k < kControlPoints; ++k) {
        const auto i = static_cast<std::size_t>(k);
        const glm::dvec2 cp = s.corePoint(k, s.coreLocked ? minCoreR : geomMinR);
        if (cp.x < minCoreR - 1e-6 && minWall > 0.0) {
            g.warnings.push_back("The core is thinner than the minimum wall allows; lower the taper or widen the margin.");
            break;
        }
        if (cp.x > s.marginRadius[i] - s.shoulderWidth + 1e-6) {
            g.warnings.push_back("The core has an undercut: its top is wider than its base.");
            break;
        }
    }
    return g;
}

bool insideMesh(const Mesh& mesh, const glm::vec3& p)
{
    const MeshBvh bvh(std::make_shared<const Mesh>(mesh));
    return inside(bvh, p);
}

AbutmentSolid finishAbutment(const AbutmentGeometry& g, const AbutmentShape& s, const Connection& c)
{
    AbutmentSolid out;
    out.warnings = g.warnings;
    // The designed part, minus the library's screw channel if it has one.
    Mesh designed = g.designed;
    if (c.screwChannel) {
        BooleanResult r = meshBoolean({&designed}, {c.screwChannel.get()});
        if (r.ok())
            designed = std::move(r.mesh);
        else
            out.warnings.push_back("The library screw channel could not be applied (" + r.error + ").");
    }
    if (c.interfaceClosed) {
        BooleanResult r = meshBoolean({&designed, c.interfaceMesh.get()});
        if (r.ok()) {
            out.mesh = std::move(r.mesh);
            out.merged = true;
        } else {
            out.error = "Could not unite the abutment with the interface: " + r.error;
        }
    }
    if (!out.merged) {
        if (!c.interfaceClosed)
            out.warnings.push_back("The library interface is not a closed solid; it is exported unchanged beside the designed part.");
        out.mesh = designed;
        const auto base = static_cast<std::uint32_t>(out.mesh.positions.size());
        out.mesh.positions.insert(out.mesh.positions.end(), c.interfaceMesh->positions.begin(), c.interfaceMesh->positions.end());
        for (std::uint32_t i : c.interfaceMesh->indices)
            out.mesh.indices.push_back(base + i);
        out.mesh.computeVertexNormals();
    }

    // Optional library geometry.
    if (c.minThickness || c.blank) {
        const MeshBvh designedBvh(std::make_shared<const Mesh>(g.designed));
        if (c.minThickness) {
            std::size_t outside = 0;
            const auto& pts = c.minThickness->positions;
            for (std::size_t i = 0; i < pts.size(); i += std::max<std::size_t>(1, pts.size() / 2000))
                if (pts[i].z > c.topHeight + 0.05 && !inside(designedBvh, pts[i]))
                    ++outside;
            if (outside)
                out.warnings.push_back("The abutment does not cover the library's minimum thickness everywhere.");
        }
        if (c.blank) {
            const MeshBvh blankBvh(c.blank);
            std::size_t outside = 0;
            for (std::size_t v = 0; v < g.outerRowEnd; v += 7)
                if (!inside(blankBvh, g.designed.positions[v]))
                    ++outside;
            if (outside)
                out.warnings.push_back("The abutment does not fit inside the pre-milled blank.");
        }
    }
    (void)s;
    return out;
}

} // namespace occlusa::implant
