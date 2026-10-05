#include "core/crown/CrownBuilder.h"

#include "core/MeshTopology.h"
#include "core/Parallel.h"
#include "core/crown/InsertionAxis.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <numbers>

namespace occlusa::crown {

namespace {

constexpr int kSideRings1 = 12; // margin -> height of contour; the library profile follows

double smoothstep(double e0, double e1, double x)
{
    const double t = std::clamp((x - e0) / std::max(e1 - e0, 1e-9), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

glm::dvec2 bezier(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, double t)
{
    const double u = 1.0 - t;
    return u * u * u * p0 + 3.0 * u * u * t * p1 + 3.0 * u * t * t * p2 + t * t * t * p3;
}

double wrapAngle(double a)
{
    while (a > std::numbers::pi)
        a -= 2.0 * std::numbers::pi;
    while (a <= -std::numbers::pi)
        a += 2.0 * std::numbers::pi;
    return a;
}

double pointSegmentDistance(const glm::dvec3& p, const glm::dvec3& a, const glm::dvec3& b)
{
    const glm::dvec3 ab = b - a;
    const double len2 = glm::dot(ab, ab);
    const double t = len2 > 0.0 ? std::clamp(glm::dot(p - a, ab) / len2, 0.0, 1.0) : 0.0;
    return glm::length(p - (a + ab * t));
}

using Axes = CrownAxes;

Axes resolveAxes(const CrownFrame& fr, const CrownParameters& p)
{
    Axes ax;
    ax.A = glm::normalize(fr.axis);
    const glm::dmat4 rot = glm::rotate(glm::dmat4(1.0), glm::radians(p.rotationDeg), ax.A);
    glm::dvec3 m = glm::dvec3(rot * glm::dvec4(fr.mesial, 0.0));
    glm::dvec3 b = glm::dvec3(rot * glm::dvec4(fr.buccal, 0.0));
    if (p.flipMesioDistal)
        m = -m;
    if (p.flipBuccoLingual)
        b = -b;
    ax.M = glm::normalize(m - glm::dot(m, ax.A) * ax.A);
    b = b - glm::dot(b, ax.A) * ax.A - glm::dot(b, ax.M) * ax.M;
    ax.B = glm::length(b) > 1e-9 ? glm::normalize(b) : glm::cross(ax.A, ax.M);
    ax.F = fr.origin + ax.M * p.shiftMesial + ax.B * p.shiftBuccal;
    return ax;
}

struct HalfWidths {
    double mesial, distal, buccal, lingual;
};

HalfWidths halfWidths(const ToothShape& t, const CrownParameters& p)
{
    return {p.halfMesial > 0 ? p.halfMesial : t.halfMesial, p.halfDistal > 0 ? p.halfDistal : t.halfDistal,
            p.halfBuccal > 0 ? p.halfBuccal : t.halfBuccal, p.halfLingual > 0 ? p.halfLingual : t.halfLingual};
}

double crownHeightOf(const ToothShape& t, const CrownParameters& p)
{
    return p.crownHeight > 0 ? p.crownHeight : t.height;
}

} // namespace

CrownAxes crownAxes(const CrownFrame& frame, const CrownParameters& params)
{
    return resolveAxes(frame, params);
}

// ---------------------------------------------------------------------------
// Orientation
// ---------------------------------------------------------------------------

void estimateToothOrientation(const Mesh& scan, const std::vector<glm::vec3>& margin, CrownFrame& frame)
{
    const glm::dvec3 A = glm::normalize(frame.axis);
    glm::dvec3 u, v;
    perpendicularBasis(A, u, v);
    double rMax = 0.0, hMax = -1e9;
    for (const auto& m : margin) {
        const glm::dvec3 d = glm::dvec3(m) - frame.origin;
        const double h = glm::dot(d, A);
        hMax = std::max(hMax, h);
        rMax = std::max(rMax, glm::length(d - h * A));
    }
    // Neighbouring crowns: an axial distribution (two opposite lobes) around the preparation.
    glm::dvec2 sum2(0.0);
    int count = 0;
    glm::dvec3 scanCentroid(0.0);
    for (const auto& p : scan.positions) {
        const glm::dvec3 d = glm::dvec3(p) - frame.origin;
        scanCentroid += glm::dvec3(p);
        const double h = glm::dot(d, A);
        if (h < hMax + 1.0 || h > hMax + 7.0)
            continue;
        const glm::dvec3 rv = d - h * A;
        const double r = glm::length(rv);
        if (r < rMax + 0.5 || r > rMax + 7.0)
            continue;
        const double ang = std::atan2(glm::dot(rv, v), glm::dot(rv, u));
        sum2 += glm::dvec2(std::cos(2.0 * ang), std::sin(2.0 * ang));
        ++count;
    }
    if (!scan.positions.empty())
        scanCentroid /= static_cast<double>(scan.positions.size());

    glm::dvec3 M;
    if (count >= 30 && glm::length(sum2) > 0.2 * count) {
        const double th = 0.5 * std::atan2(sum2.y, sum2.x);
        M = u * std::cos(th) + v * std::sin(th);
    } else {
        // Fallback: the longer axis of the margin outline.
        double sxx = 0, sxy = 0, syy = 0;
        for (const auto& m : margin) {
            const glm::dvec3 d = glm::dvec3(m) - frame.origin;
            const double x = glm::dot(d, u), y = glm::dot(d, v);
            sxx += x * x;
            sxy += x * y;
            syy += y * y;
        }
        const double th = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
        M = u * std::cos(th) + v * std::sin(th);
    }
    // Mesial points towards the bulk of the arch; lingual towards the inside of the arch.
    const glm::dvec3 toCentroid = scanCentroid - frame.origin;
    if (glm::dot(toCentroid, M) < 0.0)
        M = -M;
    glm::dvec3 B = glm::normalize(glm::cross(A, M));
    if (glm::dot(toCentroid, B) > 0.0)
        B = -B;
    frame.mesial = M;
    frame.buccal = B;
}

// ---------------------------------------------------------------------------
// Base: intaglio surface
// ---------------------------------------------------------------------------

CrownBase makeCrownBase(DieRegion die, const CrownFrame& frame, const CrownParameters& p)
{
    CrownBase base;
    base.frame = frame;
    base.frame.axis = glm::normalize(frame.axis);
    // The margin follows scan vertices and so carries the scan's staircase; the restoration uses a
    // smoothed margin curve (the die boundary moves with it, so the crown stays closed).
    {
        std::vector<glm::vec3> raw;
        for (std::uint32_t b : die.boundary)
            raw.push_back(die.mesh.positions[b]);
        const auto smooth = smoothPolyline(raw, true, 4);
        for (std::size_t i = 0; i < die.boundary.size(); ++i)
            die.mesh.positions[die.boundary[i]] = smooth[i];
        base.margin = smooth;
    }

    const std::size_t n = die.mesh.vertexCount();
    base.marginDistance.assign(n, 0.0f);
    const std::size_t L = base.margin.size();
    parallelFor(n, [&](std::size_t i) {
        const glm::dvec3 q(die.mesh.positions[i]);
        double best = 1e9;
        for (std::size_t k = 0; k < L; ++k)
            best = std::min(best, pointSegmentDistance(q, glm::dvec3(base.margin[k]), glm::dvec3(base.margin[(k + 1) % L])));
        base.marginDistance[i] = static_cast<float>(best);
    });

    Mesh intaglio = die.mesh;
    if (intaglio.normals.size() != intaglio.positions.size())
        intaglio.computeVertexNormals();
    for (std::size_t i = 0; i < n; ++i) {
        const double d = base.marginDistance[i];
        const double gap = p.cementGap * smoothstep(p.distanceToMargin * 0.4, p.distanceToMargin, d) +
                           p.extraGap * smoothstep(p.distanceToMargin + 0.5, p.distanceToMargin + 1.5, d);
        intaglio.positions[i] += intaglio.normals[i] * static_cast<float>(gap);
    }
    for (std::uint32_t b : die.boundary)
        intaglio.positions[b] = die.mesh.positions[b]; // the margin itself stays exactly on the scan
    if (p.blockOutUndercuts) {
        Blockout bo = computeBlockout(intaglio, base.frame.axis, base.frame.origin);
        intaglio.positions = std::move(bo.positions);
        base.blockoutDepth = std::move(bo.depth);
        for (std::uint32_t b : die.boundary)
            intaglio.positions[b] = die.mesh.positions[b];
    } else {
        base.blockoutDepth.assign(n, 0.0f);
    }
    intaglio.computeVertexNormals();
    auto shared = std::make_shared<const Mesh>(std::move(intaglio));
    base.intaglio = shared;
    base.intaglioBvh = MeshBvh(shared);
    base.die = std::move(die);
    return base;
}

// ---------------------------------------------------------------------------
// Contacts
// ---------------------------------------------------------------------------

ContactScene makeContactScene(const Mesh& prepScan, const CrownBase& base, const Mesh* antagonist, const glm::dmat4& antagonistToPrep,
                              const std::vector<std::pair<glm::dvec3, double>>& exclude)
{
    ContactScene scene;
    const glm::dvec3 A = base.frame.axis, O = base.frame.origin;
    double rMax = 0.0, hMin = 1e9;
    for (const auto& m : base.margin) {
        const glm::dvec3 d = glm::dvec3(m) - O;
        const double h = glm::dot(d, A);
        hMin = std::min(hMin, h);
        rMax = std::max(rMax, glm::length(d - h * A));
    }
    const double cropRadius = rMax + 12.0;

    auto crop = [&](const Mesh& src, const glm::dmat4& toPrep, auto&& keepVertex) {
        auto out = std::make_shared<Mesh>();
        std::vector<std::int64_t> remap(src.vertexCount(), -2);
        const glm::dmat3 nm = glm::transpose(glm::inverse(glm::dmat3(toPrep)));
        auto vertexIndex = [&](std::uint32_t v) -> std::int64_t {
            if (remap[v] != -2)
                return remap[v];
            const glm::dvec3 p = transformPoint(toPrep, glm::dvec3(src.positions[v]));
            if (!keepVertex(p)) {
                remap[v] = -1;
                return -1;
            }
            remap[v] = static_cast<std::int64_t>(out->positions.size());
            out->positions.emplace_back(p);
            out->normals.emplace_back(src.normals.size() == src.positions.size() ? glm::vec3(glm::normalize(nm * glm::dvec3(src.normals[v])))
                                                                                  : glm::vec3(0, 0, 1));
            return remap[v];
        };
        for (std::size_t t = 0; t + 2 < src.indices.size(); t += 3) {
            std::int64_t idx[3];
            bool ok = true;
            for (int k = 0; k < 3 && ok; ++k) {
                idx[k] = vertexIndex(src.indices[t + k]);
                ok = idx[k] >= 0;
            }
            if (ok)
                for (int k = 0; k < 3; ++k)
                    out->indices.push_back(static_cast<std::uint32_t>(idx[k]));
        }
        if (out->normals.empty() || src.normals.size() != src.positions.size())
            out->computeVertexNormals();
        return out;
    };

    auto neighbors = crop(prepScan, glm::dmat4(1.0), [&](const glm::dvec3& p) {
        const glm::dvec3 d = p - O;
        if (glm::length(d) > cropRadius)
            return false;
        const double h = glm::dot(d, A);
        if (h < hMin + 0.8)
            return false; // gingiva and the cervical parts of the neighbours
        if (glm::length(d - h * A) <= rMax + 0.5)
            return false; // the preparation itself
        for (const auto& [c, radius] : exclude) {
            const glm::dvec3 e = p - c;
            if (glm::length(e - glm::dot(e, A) * A) < radius)
                return false;
        }
        return true;
    });
    if (!neighbors->empty())
        scene.neighbors = std::make_shared<const MeshBvh>(std::shared_ptr<const Mesh>(neighbors));
    if (antagonist && !antagonist->empty()) {
        auto ant = crop(*antagonist, antagonistToPrep, [&](const glm::dvec3& p) { return glm::length(p - O) < cropRadius + 6.0; });
        if (!ant->empty())
            scene.antagonist = std::make_shared<const MeshBvh>(std::shared_ptr<const Mesh>(ant));
    }
    return scene;
}

// ---------------------------------------------------------------------------
// Crown generation
// ---------------------------------------------------------------------------

CrownMesh buildCrown(const CrownBase& base, const CrownParameters& p, std::span<const float> displacement)
{
    CrownMesh out;
    const std::shared_ptr<const ToothShape> shape = toothShape(p.library, p.kind, p.upper);
    const Axes ax = resolveAxes(base.frame, p);
    const HalfWidths hw = halfWidths(*shape, p);
    const Mesh& intaglio = *base.intaglio;
    const std::size_t dieCount = intaglio.vertexCount();
    const std::size_t L = base.margin.size();
    if (L < 3)
        return out;

    // Margin in cylindrical coordinates around the footprint centre.
    std::vector<double> mh(L), mr(L), ang(L);
    for (std::size_t i = 0; i < L; ++i) {
        const glm::dvec3 rel = glm::dvec3(base.margin[i]) - ax.F;
        mh[i] = glm::dot(rel, ax.A);
        const glm::dvec3 pl = rel - mh[i] * ax.A;
        mr[i] = glm::length(pl);
        ang[i] = std::atan2(glm::dot(pl, ax.B), glm::dot(pl, ax.M));
    }
    std::vector<double> un(L);
    un[0] = ang[0];
    for (std::size_t i = 1; i < L; ++i)
        un[i] = un[i - 1] + wrapAngle(ang[i] - ang[i - 1]);
    const double total = un[L - 1] + wrapAngle(ang[0] - ang[L - 1]) - un[0];
    const double dirSign = total >= 0.0 ? 1.0 : -1.0;
    // Monotone, smoothed column angles (the margin follows mesh edges and zig-zags slightly).
    std::vector<double> psi(L);
    for (std::size_t i = 0; i < L; ++i)
        psi[i] = dirSign * un[i];
    auto psiExt = [&](std::ptrdiff_t j) {
        const auto n = static_cast<std::ptrdiff_t>(L);
        if (j < 0)
            return psi[static_cast<std::size_t>(j + n)] - 2.0 * std::numbers::pi;
        if (j >= n)
            return psi[static_cast<std::size_t>(j - n)] + 2.0 * std::numbers::pi;
        return psi[static_cast<std::size_t>(j)];
    };
    std::vector<double> colAngle(L);
    for (std::size_t i = 0; i < L; ++i) {
        double s = 0.0;
        for (int k = -3; k <= 3; ++k)
            s += psiExt(static_cast<std::ptrdiff_t>(i) + k);
        colAngle[i] = s / 7.0;
        if (i > 0)
            colAngle[i] = std::max(colAngle[i], colAngle[i - 1] + 1e-5);
    }

    const double H = crownHeightOf(*shape, p);
    const double sz = H / std::max(shape->height, 1e-6);
    const double cuspScale = p.coping ? 0.0 : p.cuspScale;
    constexpr int K = ToothShape::kProfile;
    // Rings: margin, margin -> contour, then the library profile up to (not including) the apex.
    const int rings = 1 + kSideRings1 + (K - 2);
    out.columns = static_cast<int>(L);
    out.rings = rings;
    // Cusp scaling exaggerates or flattens the relief around the occlusal level.
    auto relief = [&](const glm::dvec2& q, double contourR) {
        const double w = 1.0 - smoothstep(0.6 * contourR, 0.9 * contourR, q.x);
        return q.y + w * (cuspScale - 1.0) * (q.y - shape->occlusalLevel);
    };

    // Vertex layout: [intaglio (die order)] [rings 1..rings-1, L columns each] [centre].
    out.outerBegin = static_cast<std::uint32_t>(dieCount);
    const std::size_t outerCount = static_cast<std::size_t>(rings - 1) * L + 1;
    const std::size_t total_ = dieCount + outerCount;
    std::vector<glm::vec3> pos(total_);
    std::vector<float> weight(total_, 0.0f), required(total_, 0.0f);
    for (std::size_t i = 0; i < dieCount; ++i)
        pos[i] = intaglio.positions[i];
    auto index = [&](int ring, std::size_t col) -> std::uint32_t {
        col %= L;
        if (ring == 0)
            return base.die.boundary[col];
        return static_cast<std::uint32_t>(dieCount + static_cast<std::size_t>(ring - 1) * L + col);
    };
    const auto centre = static_cast<std::uint32_t>(dieCount + outerCount - 1);

    double hTop = -1e9; // intaglio top along the axis (coping)
    for (const auto& q : intaglio.positions)
        hTop = std::max(hTop, glm::dot(glm::dvec3(q) - ax.F, ax.A));

    // Radial support at height h and vertical support at radius r, dilated by `reach` (the required
    // thickness): the widest wall within `reach` above, the highest top within `reach` inwards.
    // Without the dilation, points just past an edge of the die would find no support at all.
    auto support = [&](const glm::dvec3& planeDir, double h, double r, double reach, double& rMin, double& hMin) {
        rMin = -1e9;
        hMin = -1e9;
        for (double dh : {0.0, 0.5, 1.0})
            if (auto hit = base.intaglioBvh.raycast(glm::vec3(ax.F + ax.A * (h + dh * reach)), glm::vec3(planeDir), 30.0f))
                rMin = std::max(rMin, static_cast<double>(hit->t));
        for (double dr : {0.0, 0.5, 1.0}) {
            const glm::dvec3 top = ax.F + planeDir * std::max(r - dr * reach, 0.0) + ax.A * (hTop + 20.0);
            if (auto hit = base.intaglioBvh.raycast(glm::vec3(top), glm::vec3(-ax.A), 60.0f))
                hMin = std::max(hMin, hTop + 20.0 - hit->t);
        }
    };

    parallelFor(L, [&](std::size_t i) {
        const double a = dirSign * colAngle[i];
        const double c = std::cos(a), s = std::sin(a);
        const glm::dvec3 d = ax.M * c + ax.B * s;
        const double rm = mr[i], hmI = mh[i];
        // Library azimuth that lands on this column after the anisotropic scaling to the crown size.
        const double sx = c >= 0 ? hw.mesial / shape->halfMesial : hw.distal / shape->halfDistal;
        const double sy = s >= 0 ? hw.buccal / shape->halfBuccal : hw.lingual / shape->halfLingual;
        const double th = std::atan2(s / sy, c / sx);
        const double radial = std::hypot(sx * std::cos(th), sy * std::sin(th));
        std::array<glm::dvec2, K> prof;
        const double contourLib = shape->at(th, 0).x;
        for (int k = 0; k < K; ++k) {
            const glm::dvec2 q = shape->at(th, k);
            prof[static_cast<std::size_t>(k)] = glm::dvec2(q.x * radial, relief(q, contourLib) * sz);
        }
        // Keep the height of contour outside and above the margin (fading out towards the apex).
        const double needR = rm + 0.4 + p.marginThickness;
        const double needH = hmI + 0.8;
        const double dR = std::max(0.0, needR - prof[0].x), dH = std::max(0.0, needH - prof[0].y);
        for (int k = 0; k < K; ++k) {
            const double f = 1.0 - static_cast<double>(k) / (K - 1);
            prof[static_cast<std::size_t>(k)].x += dR * f * f;
            prof[static_cast<std::size_t>(k)].y += dH * f;
        }
        const double Rc = prof[0].x, Hc = prof[0].y;
        double copingContourR = Rc;
        std::vector<glm::dvec2> rh(static_cast<std::size_t>(rings));

        for (int ring = 1; ring < rings; ++ring) {
            double r, h;
            bool cap = false;
            if (ring <= kSideRings1) {
                const double t = std::pow(static_cast<double>(ring) / kSideRings1, 1.3);
                const glm::dvec2 q = bezier({rm, hmI}, {rm + p.marginThickness + 0.35 * (Rc - rm), hmI + 0.35 * (Hc - hmI)},
                                            {Rc, hmI + 0.7 * (Hc - hmI)}, {Rc, Hc}, t);
                r = q.x;
                h = q.y;
            } else if (p.coping) {
                // Coping: a dome over the die from the contour ring inwards.
                const double f = static_cast<double>(ring - kSideRings1) / (K - 1);
                r = copingContourR * (1.0 - f);
                h = Hc;
                cap = true;
            } else {
                const glm::dvec2 q = prof[static_cast<std::size_t>(ring - kSideRings1)];
                r = q.x;
                h = q.y;
                cap = r < 0.85 * Rc; // occlusal surface: supported vertically only
            }
            const double tau = cap ? p.minThickness : glm::mix(p.marginThickness, p.minThickness, smoothstep(0.0, 1.2, h - hmI));
            double rMin, hMin;
            support(d, h, r, p.coping ? p.copingThickness : tau, rMin, hMin);
            if (p.coping) {
                // Uniform shell over the intaglio.
                const double thick = std::max(p.copingThickness * smoothstep(0.0, 1.0, h - hmI), p.marginThickness);
                if (!cap && rMin > -1e8)
                    r = rMin + thick;
                if (hMin > -1e8 && (cap || h < hMin + thick))
                    h = hMin + thick;
                if (ring == kSideRings1)
                    copingContourR = r;
            } else {
                // Side walls are supported radially, the occlusal cap only vertically (pushing cap
                // rings outwards would fold them over each other).
                if (!cap && rMin > -1e8 && r < rMin + tau)
                    r = rMin + tau;
                if (hMin > -1e8 && h < hMin + tau)
                    h = hMin + tau;
            }
            const std::uint32_t vi = index(ring, i);
            rh[static_cast<std::size_t>(ring)] = glm::dvec2(r, h);
            required[vi] = static_cast<float>(p.coping ? std::min(tau, p.copingThickness) : tau);
            weight[vi] = ring <= kSideRings1 ? static_cast<float>(smoothstep(1.0, 4.0, ring)) : 1.0f;
        }
        // From the height of contour to the apex the radius must not grow again, or the thickness
        // pushes above would fold the surface over itself: push outer rings out where needed.
        for (int ring = rings - 1; ring > kSideRings1; --ring)
            rh[static_cast<std::size_t>(ring - 1)].x = std::max(rh[static_cast<std::size_t>(ring - 1)].x, rh[static_cast<std::size_t>(ring)].x);
        for (int ring = 1; ring < rings; ++ring)
            pos[index(ring, i)] = glm::vec3(ax.F + ax.A * rh[static_cast<std::size_t>(ring)].y + d * rh[static_cast<std::size_t>(ring)].x);
    });
    {
        const glm::dvec2 apex = shape->at(0.0, K - 1);
        double h = relief(apex, 1e-9 + shape->at(0.0, 0).x) * sz;
        const double tau = p.coping ? p.copingThickness : p.minThickness;
        double rMin, hMin;
        support(ax.M, h, 0.0, tau, rMin, hMin);
        if (hMin > -1e8 && (p.coping || h < hMin + tau))
            h = hMin + tau;
        pos[centre] = glm::vec3(ax.F + ax.A * h);
        weight[centre] = 1.0f;
        required[centre] = static_cast<float>(tau);
    }

    // Faces: reversed die faces (intaglio) + outer grid + centre fan.
    Mesh& m = out.mesh;
    m.positions = pos;
    for (std::size_t t = 0; t + 2 < intaglio.indices.size(); t += 3) {
        m.indices.push_back(intaglio.indices[t]);
        m.indices.push_back(intaglio.indices[t + 2]);
        m.indices.push_back(intaglio.indices[t + 1]);
    }
    // The die uses the margin edge b0->b1 in one direction; the outer shell must use it the same
    // way (the reversed intaglio uses the opposite one) for a consistently oriented closed surface.
    bool dieForward = false;
    const std::uint32_t b0 = base.die.boundary[0], b1 = base.die.boundary[1];
    for (std::size_t t = 0; t + 2 < intaglio.indices.size() && !dieForward; t += 3)
        for (int e = 0; e < 3; ++e)
            if (intaglio.indices[t + e] == b0 && intaglio.indices[t + (e + 1) % 3] == b1)
                dieForward = true;
    auto tri = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
        if (dieForward) {
            m.indices.insert(m.indices.end(), {a, b, c});
        } else {
            m.indices.insert(m.indices.end(), {a, c, b});
        }
    };
    for (int ring = 0; ring + 1 < rings; ++ring)
        for (std::size_t c = 0; c < L; ++c) {
            const std::uint32_t a = index(ring, c), b = index(ring, c + 1), cc = index(ring + 1, c + 1), d = index(ring + 1, c);
            tri(a, b, cc);
            tri(a, cc, d);
        }
    for (std::size_t c = 0; c < L; ++c)
        tri(index(rings - 1, c), index(rings - 1, c + 1), centre);

    if (signedVolume(m) < 0.0)
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
            std::swap(m.indices[t + 1], m.indices[t + 2]);
    m.computeVertexNormals();
    out.basePositions = m.positions;
    out.baseNormals = m.normals;
    out.editWeight = std::move(weight);
    out.requiredThickness = std::move(required);

    // Displacement (sculpting / contact adaptation) along the base normals.
    if (displacement.size() == m.vertexCount())
        for (std::size_t v = out.outerBegin; v < m.vertexCount(); ++v)
            m.positions[v] += out.baseNormals[v] * (displacement[v] * out.editWeight[v]);

    // Enforce the minimum thickness over the intaglio after editing (a few passes: pushing one
    // vertex out can change which part of the intaglio is closest).
    double minThick = 1e9;
    std::vector<int> thickened(m.vertexCount(), 0);
    for (int pass = 0; pass < 4; ++pass) {
        std::atomic<int> moved{0};
        parallelFor(m.vertexCount() - out.outerBegin, [&](std::size_t k) {
            const std::size_t v = out.outerBegin + k;
            if (out.editWeight[v] <= 0.0f)
                return;
            const float need = out.requiredThickness[v];
            if (auto cp = base.intaglioBvh.closestPoint(m.positions[v], need + 6.0f)) {
                if (cp->signedDistance < 0.0f) {
                    // Carved through the intaglio: back out along the intaglio normal.
                    m.positions[v] = cp->point + cp->normal * need;
                    thickened[v] = 1;
                    ++moved;
                } else if (cp->signedDistance < need * 0.98f) {
                    // Move away from the intaglio along its normal, blended with the shell normal.
                    const glm::vec3 dir = glm::normalize(cp->normal + out.baseNormals[v]);
                    const float along = std::max(glm::dot(dir, cp->normal), 0.3f);
                    m.positions[v] += dir * ((need - cp->signedDistance) / along);
                    thickened[v] = 1;
                    ++moved;
                }
            }
        });
        if (moved == 0)
            break;
    }
    for (int t : thickened)
        out.thickenedVertices += t;
    m.computeVertexNormals();
    // Reported over the full-thickness zone (the required thickness ramps up from the margin).
    float fullThickness = 0.0f;
    for (std::size_t v = out.outerBegin; v < m.vertexCount(); ++v)
        fullThickness = std::max(fullThickness, out.requiredThickness[v]);
    for (std::size_t v = out.outerBegin; v < m.vertexCount(); ++v) {
        if (out.editWeight[v] < 0.999f || out.requiredThickness[v] < 0.99f * fullThickness)
            continue;
        if (auto cp = base.intaglioBvh.closestPoint(m.positions[v], 5.0f))
            minThick = std::min(minThick, static_cast<double>(cp->signedDistance));
    }
    out.minThickness = minThick > 1e8 ? 0.0 : minThick;
    out.volume = signedVolume(m);
    out.watertight = isClosedManifold(m);
    return out;
}

// ---------------------------------------------------------------------------
// Fitting
// ---------------------------------------------------------------------------

void fitProximal(const CrownBase& base, const ContactScene& contacts, CrownParameters& p)
{
    const std::shared_ptr<const ToothShape> shape = toothShape(p.library, p.kind, p.upper);
    p.shiftMesial = 0.0;
    p.shiftBuccal = 0.0;
    const Axes ax = resolveAxes(base.frame, p);
    const double H = crownHeightOf(*shape, p);
    double hMarginMax = -1e9;
    double extent[4] = {0, 0, 0, 0}; // margin extent towards +M, -M, +B, -B
    for (const auto& q : base.margin) {
        const glm::dvec3 d = glm::dvec3(q) - ax.F;
        hMarginMax = std::max(hMarginMax, glm::dot(d, ax.A));
        extent[0] = std::max(extent[0], glm::dot(d, ax.M));
        extent[1] = std::max(extent[1], -glm::dot(d, ax.M));
        extent[2] = std::max(extent[2], glm::dot(d, ax.B));
        extent[3] = std::max(extent[3], -glm::dot(d, ax.B));
    }
    const double Hc = std::max(shape->contourHeight * H, hMarginMax + 1.0);
    auto gap = [&](const glm::dvec3& dir) -> double {
        if (!contacts.neighbors)
            return -1.0;
        // Sample a few heights around the height of contour; the contact is the closest one.
        double best = -1.0;
        for (double dh : {-0.8, 0.0, 0.8, 1.6}) {
            if (auto hit = contacts.neighbors->raycast(glm::vec3(ax.F + ax.A * (Hc + dh)), glm::vec3(dir), 9.0f))
                best = best < 0 ? hit->t : std::min(best, static_cast<double>(hit->t));
        }
        return best;
    };
    const double dm = gap(ax.M), dd = gap(-ax.M);
    p.halfMesial = dm > 0 ? dm - p.proximalTarget : shape->halfMesial;
    p.halfDistal = dd > 0 ? dd - p.proximalTarget : shape->halfDistal;
    p.halfMesial = std::max(p.halfMesial, extent[0] + 0.5);
    p.halfDistal = std::max(p.halfDistal, extent[1] + 0.5);
    // Bucco-lingual width follows the mesio-distal fit proportionally (anatomic ratio).
    const double ratio = std::clamp((p.halfMesial + p.halfDistal) / (shape->halfMesial + shape->halfDistal), 0.75, 1.3);
    p.halfBuccal = std::max(shape->halfBuccal * std::sqrt(ratio), extent[2] + 0.6);
    p.halfLingual = std::max(shape->halfLingual * std::sqrt(ratio), extent[3] + 0.6);
}

bool fitOcclusalHeight(const CrownBase& base, const ContactScene& contacts, CrownParameters& p, std::span<const float> displacement)
{
    if (!contacts.antagonist)
        return false;
    const Axes ax = resolveAxes(base.frame, p);
    double hMarginMax = -1e9;
    for (const auto& q : base.margin)
        hMarginMax = std::max(hMarginMax, glm::dot(glm::dvec3(q) - ax.F, ax.A));
    auto clearance = [&](double H) {
        CrownParameters q = p;
        q.crownHeight = H;
        const CrownMesh c = buildCrown(base, q, displacement);
        double best = 5.0;
        for (std::size_t v = c.outerBegin; v < c.mesh.vertexCount(); ++v) {
            if (c.editWeight[v] < 0.999f)
                continue;
            if (auto cp = contacts.antagonist->closestPoint(c.mesh.positions[v], 5.0f))
                best = std::min(best, static_cast<double>(cp->signedDistance));
        }
        return best;
    };
    double lo = hMarginMax + 3.0, hi = hMarginMax + 15.0;
    const double target = p.occlusalTarget;
    if (clearance(lo) <= target) {
        p.crownHeight = lo;
        return true;
    }
    if (clearance(hi) > target)
        return false; // antagonist out of reach: keep the template height
    for (int it = 0; it < 18; ++it) {
        const double mid = 0.5 * (lo + hi);
        if (clearance(mid) > target)
            lo = mid;
        else
            hi = mid;
    }
    p.crownHeight = lo;
    return true;
}

namespace {

// Laplacian smoothing of the displacement field over the outer shell (removes ripples left by
// per-vertex contact corrections).
void smoothDisplacement(const CrownBase& base, const CrownParameters& p, std::vector<float>& displacement, int iterations)
{
    const CrownMesh c = buildCrown(base, p, displacement);
    if (displacement.size() != c.mesh.vertexCount())
        return;
    const MeshAdjacency adj(c.mesh);
    for (int it = 0; it < iterations; ++it) {
        std::vector<float> s = displacement;
        for (std::size_t v = c.outerBegin; v < displacement.size(); ++v) {
            float sum = 0.0f;
            int cnt = 0;
            for (std::uint32_t w : adj.neighbors(static_cast<std::uint32_t>(v)))
                if (w >= c.outerBegin) {
                    sum += displacement[w];
                    ++cnt;
                }
            if (cnt > 0)
                s[v] = (0.5f * displacement[v] + 0.5f * sum / static_cast<float>(cnt)) * std::min(1.0f, c.editWeight[v] + 0.001f);
        }
        displacement.swap(s);
    }
}

} // namespace

void adaptContacts(const CrownBase& base, const CrownParameters& p, const ContactScene& contacts, std::vector<float>& displacement,
                   bool proximal, bool occlusal, double reach)
{
    constexpr int kRounds = 10;
    for (int round = 0; round < kRounds; ++round) {
        // The last rounds only pull back, so spreading an extension never leaves an overlap.
        const bool allowExtend = round < kRounds - 4;
        if (round == kRounds - 4)
            smoothDisplacement(base, p, displacement, 6);
        const CrownMesh c = buildCrown(base, p, displacement);
        const std::size_t nv = c.mesh.vertexCount();
        if (displacement.size() != nv)
            displacement.assign(nv, 0.0f);
        std::vector<float> delta(nv, 0.0f);
        std::vector<std::uint8_t> touched(nv, 0);
        parallelFor(nv - c.outerBegin, [&](std::size_t k) {
            const std::size_t v = c.outerBegin + k;
            if (c.editWeight[v] <= 0.0f)
                return;
            const glm::vec3 pos = c.mesh.positions[v], nrm = c.mesh.normals[v];
            float pull = 0.0f, push = 0.0f;
            auto handle = [&](const MeshBvh* bvh, double target, bool extend) {
                if (!bvh)
                    return;
                const auto cp = bvh->closestPoint(pos, static_cast<float>(reach + 1.0));
                if (!cp)
                    return;
                const double need = target - cp->signedDistance; // > 0: too close / overlapping
                if (need > 0.0) {
                    pull = std::min(pull, static_cast<float>(-need));
                } else if (extend && cp->signedDistance < reach && cp->distance > 1e-6f) {
                    const float facing = glm::dot(nrm, (cp->point - pos) / cp->distance);
                    if (facing > 0.6f)
                        push = std::max(push, static_cast<float>(-need * (1.0 - cp->signedDistance / reach) * 0.8));
                }
            };
            if (proximal)
                handle(contacts.neighbors.get(), p.proximalTarget, allowExtend);
            if (occlusal)
                handle(contacts.antagonist.get(), p.occlusalTarget, false);
            if (pull < 0.0f) {
                delta[v] = pull;
                touched[v] = 1;
            } else if (push > 0.0f) {
                delta[v] = push;
                touched[v] = 1;
            }
        });
        bool any = false;
        for (auto t : touched)
            any |= t != 0;
        if (!any) {
            if (allowExtend)
                continue;
            break;
        }
        // Spread the correction so it forms a smooth contact facet rather than a dent.
        const MeshAdjacency adj(c.mesh);
        for (int it = 0; it < 2; ++it) {
            std::vector<float> s = delta;
            for (std::size_t v = c.outerBegin; v < nv; ++v) {
                float sum = delta[v];
                float extreme = delta[v];
                int cnt = 1;
                for (std::uint32_t w : adj.neighbors(static_cast<std::uint32_t>(v))) {
                    if (w < c.outerBegin)
                        continue;
                    sum += delta[w];
                    ++cnt;
                    if (std::abs(delta[w]) > std::abs(extreme))
                        extreme = delta[w];
                }
                // Keep the full correction where it is needed; blend around it.
                s[v] = touched[v] ? delta[v] : 0.6f * extreme + 0.4f * sum / static_cast<float>(cnt);
            }
            delta.swap(s);
        }
        for (std::size_t v = c.outerBegin; v < nv; ++v)
            displacement[v] += delta[v] * c.editWeight[v];
    }
}

std::vector<float> contactDistances(const CrownMesh& crown, const ContactScene& contacts, float maxDistance)
{
    const std::size_t nv = crown.mesh.vertexCount();
    std::vector<float> out(nv, std::numeric_limits<float>::quiet_NaN());
    parallelFor(nv - crown.outerBegin, [&](std::size_t k) {
        const std::size_t v = crown.outerBegin + k;
        float best = std::numeric_limits<float>::quiet_NaN();
        for (const MeshBvh* bvh : {contacts.neighbors.get(), contacts.antagonist.get()}) {
            if (!bvh)
                continue;
            if (auto cp = bvh->closestPoint(crown.mesh.positions[v], maxDistance))
                if (std::isnan(best) || cp->signedDistance < best)
                    best = cp->signedDistance;
        }
        out[v] = best;
    });
    return out;
}

void applyBrush(const CrownMesh& crown, std::vector<float>& displacement, const glm::vec3& center, float radius, float strength, BrushMode mode)
{
    const std::size_t nv = crown.mesh.vertexCount();
    if (displacement.size() != nv)
        displacement.assign(nv, 0.0f);
    std::unique_ptr<MeshAdjacency> adj;
    if (mode == BrushMode::Smooth)
        adj = std::make_unique<MeshAdjacency>(crown.mesh);
    std::vector<float> delta(nv, 0.0f);
    for (std::size_t v = crown.outerBegin; v < nv; ++v) {
        const float d = glm::length(crown.mesh.positions[v] - center);
        if (d >= radius || crown.editWeight[v] <= 0.0f)
            continue;
        const float f = 1.0f - (d / radius) * (d / radius);
        const float w = f * f;
        switch (mode) {
        case BrushMode::Add: delta[v] = strength * w; break;
        case BrushMode::Remove: delta[v] = -strength * w; break;
        case BrushMode::Smooth: {
            glm::vec3 avg(0.0f);
            int cnt = 0;
            for (std::uint32_t nb : adj->neighbors(static_cast<std::uint32_t>(v))) {
                avg += crown.mesh.positions[nb];
                ++cnt;
            }
            if (cnt > 0) {
                const glm::vec3 lap = avg / static_cast<float>(cnt) - crown.mesh.positions[v];
                delta[v] = glm::dot(lap, crown.baseNormals[v]) * w * std::clamp(strength * 4.0f, 0.1f, 1.0f);
            }
            break;
        }
        }
    }
    for (std::size_t v = crown.outerBegin; v < nv; ++v)
        displacement[v] += delta[v];
}

std::vector<float> thicknessMap(const CrownBase& base, const CrownMesh& crown)
{
    const std::size_t nv = crown.mesh.vertexCount();
    std::vector<float> out(nv, std::numeric_limits<float>::quiet_NaN());
    parallelFor(nv - crown.outerBegin, [&](std::size_t k) {
        const std::size_t v = crown.outerBegin + k;
        if (auto cp = base.intaglioBvh.closestPoint(crown.mesh.positions[v], 4.0f))
            out[v] = cp->signedDistance;
    });
    return out;
}

} // namespace occlusa::crown
