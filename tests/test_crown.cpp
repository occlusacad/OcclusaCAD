#include "TestUtil.h"

#include "core/IsoSurface.h"
#include "core/MeshBvh.h"
#include "core/MeshTopology.h"
#include "core/crown/CrownBuilder.h"
#include "core/crown/InsertionAxis.h"
#include "core/crown/Margin.h"

#include <doctest.h>

#include <cmath>
#include <memory>
#include <random>

using namespace occlusa;
using namespace occlusa::crown;

namespace {

// Synthetic lower-molar shoulder preparation (local frame: z occlusal, x mesio-distal) with
// two neighbouring teeth along x and a gingiva slab. Margin: ellipse (5.0, 4.5) at z = 0.
constexpr double kMarginRx = 5.0, kMarginRy = 4.5;

double ellipse2d(double x, double y, double rx, double ry)
{
    return (std::sqrt((x / rx) * (x / rx) + (y / ry) * (y / ry)) - 1.0) * std::min(rx, ry);
}

double smin(double a, double b, double k)
{
    const double h = std::clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
    return glm::mix(b, a, h) - k * h * (1.0 - h);
}

double prepSdf(const glm::dvec3& p)
{
    const double top = 4.5, rho = 0.6;
    const double f = std::clamp(p.z, 0.0, top) / top;
    const double e = ellipse2d(p.x, p.y, 4.2 - 0.6 * f, 3.7 - 0.6 * f);
    const double qx = std::max(e + rho, 0.0), qz = std::max(p.z - top + rho, 0.0);
    double prep = std::sqrt(qx * qx + qz * qz) - rho + std::min(std::max(e + rho, p.z - top + rho), 0.0);
    prep = std::max(prep, -6.0 - p.z);
    const double stump = std::max({ellipse2d(p.x, p.y, kMarginRx, kMarginRy), p.z, -6.0 - p.z});
    const double tooth = std::min(prep, stump);
    double neighbors = 1e9;
    for (double sx : {-1.0, 1.0}) {
        const glm::dvec3 q = (p - glm::dvec3(sx * 10.5, 0.0, 3.0)) / glm::dvec3(5.0, 4.5, 4.0);
        neighbors = std::min(neighbors, (glm::length(q) - 1.0) * 4.0);
    }
    const double gum = std::max(p.z + 2.0, -8.0 - p.z);
    return smin(std::min(tooth, neighbors), gum, 0.5);
}

struct Fixture {
    glm::dmat4 toScan{1.0}; // local -> scan coordinates (arbitrary pose)
    std::shared_ptr<Mesh> scan;
    Mesh antagonist;        // flat occluding plane at z = 7.5 (local), facing down

    Fixture()
    {
        Volume v;
        const double res = 0.2;
        const glm::dvec3 lo(-17.0, -9.0, -6.0), hi(17.0, 9.0, 8.0);
        v.geometry.spacing = glm::dvec3(res);
        v.geometry.dims = glm::ivec3(glm::ceil((hi - lo) / res));
        v.geometry.origin = lo;
        v.voxels.resize(v.geometry.voxelCount());
        for (int z = 0; z < v.geometry.dims.z; ++z)
            for (int y = 0; y < v.geometry.dims.y; ++y)
                for (int x = 0; x < v.geometry.dims.x; ++x) {
                    const double sd = prepSdf(lo + glm::dvec3(x, y, z) * res);
                    v.voxels[v.index(x, y, z)] = static_cast<std::int16_t>(std::clamp(-sd * 1000.0, -32000.0, 32000.0));
                }
        IsoSurfaceOptions opt;
        opt.isoValue = 0.0;
        Mesh m = extractIsoSurface(v, opt);
        // Open the bottom like a real scan.
        Mesh open;
        std::vector<std::int64_t> remap(m.positions.size(), -1);
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
            bool keep = true;
            for (int k = 0; k < 3; ++k)
                keep &= m.positions[m.indices[t + k]].z > -5.0f;
            if (!keep)
                continue;
            for (int k = 0; k < 3; ++k) {
                const auto i = m.indices[t + k];
                if (remap[i] < 0) {
                    remap[i] = static_cast<std::int64_t>(open.positions.size());
                    open.positions.push_back(m.positions[i]);
                }
                open.indices.push_back(static_cast<std::uint32_t>(remap[i]));
            }
        }
        toScan = glm::translate(glm::dmat4(1.0), glm::dvec3(12.0, -30.0, 5.0)) * glm::rotate(glm::dmat4(1.0), 0.7, glm::normalize(glm::dvec3(0.3, 1.0, 0.4)));
        for (auto& p : open.positions)
            p = glm::vec3(transformPoint(toScan, glm::dvec3(p)));
        open.computeVertexNormals();
        scan = std::make_shared<Mesh>(std::move(open));

        // Antagonist: a fine grid plane, normals facing the preparation (-z local).
        const int n = 60;
        for (int j = 0; j <= n; ++j)
            for (int i = 0; i <= n; ++i)
                antagonist.positions.emplace_back(transformPoint(toScan, glm::dvec3(-15.0 + 30.0 * i / n, -15.0 + 30.0 * j / n, 7.5)));
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                const std::uint32_t a = j * (n + 1) + i, b = a + 1, c = a + n + 1, d = c + 1;
                antagonist.indices.insert(antagonist.indices.end(), {a, c, b, b, c, d}); // faces -z
            }
        antagonist.computeVertexNormals();
    }

    glm::dvec3 local(const glm::dvec3& scanPoint) const { return transformPoint(glm::inverse(toScan), scanPoint); }
    glm::dvec3 axisInScan() const { return glm::normalize(transformVector(toScan, glm::dvec3(0, 0, 1))); }
};

const Fixture& fixture()
{
    static const Fixture f;
    return f;
}

double marginError(const Fixture& f, const glm::vec3& scanPoint)
{
    const glm::dvec3 q = f.local(glm::dvec3(scanPoint));
    const double radial = std::abs(ellipse2d(q.x, q.y, kMarginRx, kMarginRy));
    return std::sqrt(radial * radial + q.z * q.z);
}

} // namespace

TEST_CASE("BVH ray casts and closest points match brute force")
{
    auto mesh = std::make_shared<Mesh>(test::makeCube(10.0f));
    const MeshBvh bvh(mesh);
    auto hit = bvh.raycast({0, 0, 20}, {0, 0, -1});
    REQUIRE(hit);
    CHECK(hit->t == doctest::Approx(15.0f).epsilon(1e-5));
    CHECK(hit->point.z == doctest::Approx(5.0f));
    CHECK_FALSE(bvh.raycast({20, 20, 20}, {0, 0, -1}));

    std::mt19937 rng(3);
    std::uniform_real_distribution<float> u(-9.0f, 9.0f);
    for (int i = 0; i < 200; ++i) {
        const glm::vec3 p(u(rng), u(rng), u(rng));
        float brute = 1e9f;
        for (std::size_t t = 0; t < mesh->triangleCount(); ++t) {
            const glm::vec3 q = closestPointOnTriangle(p, mesh->positions[mesh->indices[3 * t]], mesh->positions[mesh->indices[3 * t + 1]],
                                                       mesh->positions[mesh->indices[3 * t + 2]]);
            brute = std::min(brute, glm::length(p - q));
        }
        const auto cp = bvh.closestPoint(p, 100.0f);
        REQUIRE(cp);
        CHECK(cp->distance == doctest::Approx(brute).epsilon(1e-4));
        const bool inside = std::abs(p.x) < 5 && std::abs(p.y) < 5 && std::abs(p.z) < 5;
        CHECK((cp->signedDistance < 0) == inside);
    }
}

TEST_CASE("Mesh topology helpers")
{
    const Mesh cube = test::makeCube(4.0f);
    CHECK(isClosedManifold(cube));
    CHECK(signedVolume(cube) == doctest::Approx(64.0));
    CHECK(surfaceArea(cube) == doctest::Approx(96.0));
    const MeshAdjacency adj(cube);
    CHECK(adj.vertexCount() == 8);
    const auto path = shortestPath(cube, adj, {}, 0, 6);
    REQUIRE(path.size() >= 2);
    for (std::size_t i = 1; i < path.size(); ++i) {
        const auto nb = adj.neighbors(path[i - 1]);
        CHECK(std::find(nb.begin(), nb.end(), path[i]) != nb.end());
    }
    const auto conv = vertexConvexity(cube, adj, 0);
    for (float k : conv)
        CHECK(k > 0.0f); // all corners convex
}

TEST_CASE("Margin detection, die extraction and insertion axis on a shoulder preparation")
{
    const Fixture& f = fixture();
    const PrepScan prep(f.scan);
    const glm::vec3 click(transformPoint(f.toScan, glm::dvec3(0.3, -0.2, 4.5)));
    std::string err;
    const auto det = detectMargin(prep, click, {}, &err);
    INFO(err);
    REQUIRE(det);
    CHECK(det->line.closed);
    CHECK(det->binsFound > det->bins * 8 / 10);

    // Consecutive margin vertices share a mesh edge.
    const auto& loop = det->line.vertices;
    for (std::size_t i = 0; i < loop.size(); ++i) {
        const auto nb = prep.adjacency().neighbors(loop[i]);
        CHECK(std::find(nb.begin(), nb.end(), loop[(i + 1) % loop.size()]) != nb.end());
    }
    // Accuracy against the analytic margin (outer rim of the shoulder).
    double sum = 0.0, worst = 0.0;
    const auto pts = det->line.points(*f.scan);
    for (const auto& p : pts) {
        const double e = marginError(f, p);
        sum += e;
        worst = std::max(worst, e);
    }
    MESSAGE("margin error mean " << sum / pts.size() << " max " << worst);
    CHECK(sum / pts.size() < 0.12);
    CHECK(worst < 0.3);
    CHECK(glm::dot(det->occlusalDirection, f.axisInScan()) > std::cos(glm::radians(3.0)));

    const auto die = extractDie(prep, det->line, click, &err);
    INFO(err);
    REQUIRE(die);
    CHECK(die->boundary.size() == loop.size());
    for (const auto& p : die->mesh.positions)
        CHECK(f.local(glm::dvec3(p)).z > -0.35);
    // Clicking outside the preparation leaks into the rest of the scan.
    CHECK_FALSE(extractDie(prep, det->line, glm::vec3(transformPoint(f.toScan, glm::dvec3(0, 8, -2))), &err));

    // Insertion axis: tapered walls, no undercuts along the true axis; tilting creates some.
    const glm::dvec3 cen = centroid(pts);
    const glm::dvec3 best = optimizeInsertionAxis(die->mesh, det->occlusalDirection);
    CHECK(glm::dot(best, f.axisInScan()) > std::cos(glm::radians(3.0)));
    const auto straight = undercutReport(die->mesh, computeBlockout(die->mesh, f.axisInScan(), cen));
    glm::dvec3 u, v;
    perpendicularBasis(f.axisInScan(), u, v);
    const auto tilted = undercutReport(die->mesh, computeBlockout(die->mesh, tiltAxis(f.axisInScan(), u, v, 15.0, 0.0), cen));
    MESSAGE("undercut area straight " << straight.undercutArea << " tilted " << tilted.undercutArea << " max depth " << tilted.maxDepth);
    CHECK(straight.undercutArea < 0.05 * straight.dieArea);
    CHECK(tilted.undercutArea > straight.undercutArea + 2.0);
    CHECK(tilted.maxDepth > 0.3);
}

TEST_CASE("Manual margin tracing closes the loop through control points")
{
    const Fixture& f = fixture();
    const PrepScan prep(f.scan);
    std::vector<std::uint32_t> controls;
    for (int k = 0; k < 6; ++k) {
        const double a = k * 2.0 * 3.14159265 / 6.0;
        controls.push_back(prep.nearestVertex(glm::vec3(transformPoint(f.toScan, glm::dvec3(kMarginRx * std::cos(a), kMarginRy * std::sin(a), 0.0)))));
    }
    const MarginLine line = traceMargin(prep, controls, true);
    REQUIRE(line.closed);
    double worst = 0.0;
    for (const auto& p : line.points(*f.scan))
        worst = std::max(worst, marginError(f, p));
    CHECK(worst < 0.35); // the curvature-weighted path follows the shoulder rim between the clicks
    CHECK(polylineLength(line.points(*f.scan), true) == doctest::Approx(2 * 3.14159 * std::sqrt((25.0 + 20.25) / 2)).epsilon(0.12));
}

TEST_CASE("Crown design: fit, contacts, thickness and watertightness")
{
    const Fixture& f = fixture();
    const PrepScan prep(f.scan);
    const glm::vec3 click(transformPoint(f.toScan, glm::dvec3(0.0, 0.0, 4.5)));
    const auto det = detectMargin(prep, click);
    REQUIRE(det);
    auto die = extractDie(prep, det->line, click);
    REQUIRE(die);

    CrownFrame frame;
    frame.origin = centroid(det->line.points(*f.scan));
    frame.axis = optimizeInsertionAxis(die->mesh, det->occlusalDirection);
    estimateToothOrientation(*f.scan, det->line.points(*f.scan), frame);
    const glm::dvec3 localX = glm::normalize(transformVector(f.toScan, glm::dvec3(1, 0, 0)));
    CHECK(std::abs(glm::dot(frame.mesial, localX)) > 0.95); // neighbours sit along x

    CrownParameters params;
    params.kind = ToothKind::FirstMolar;
    const CrownBase base = makeCrownBase(*die, frame, params);
    REQUIRE(base.intaglio);
    // Cement gap: away from the margin the intaglio is offset by cement + extra gap.
    for (std::size_t i = 0; i < base.die.mesh.vertexCount(); ++i)
        if (base.marginDistance[i] > 3.0f && base.blockoutDepth[i] < 1e-4f)
            CHECK(glm::length(base.intaglio->positions[i] - base.die.mesh.positions[i]) == doctest::Approx(0.07).epsilon(0.05));
    for (std::uint32_t b : base.die.boundary)
        CHECK(base.intaglio->positions[b] == base.die.mesh.positions[b]);

    const ContactScene contacts = makeContactScene(*f.scan, base, &f.antagonist, glm::dmat4(1.0));
    REQUIRE(contacts.neighbors);
    REQUIRE(contacts.antagonist);
    fitProximal(base, contacts, params);
    MESSAGE("half widths " << params.halfMesial << " " << params.halfDistal << " " << params.halfBuccal << " " << params.halfLingual);
    CHECK(params.halfMesial == doctest::Approx(5.55).epsilon(0.03));
    CHECK(params.halfDistal == doctest::Approx(5.55).epsilon(0.03));
    REQUIRE(fitOcclusalHeight(base, contacts, params));

    std::vector<float> displacement;
    CrownMesh crown = buildCrown(base, params, displacement);
    CHECK(crown.watertight);
    CHECK(crown.volume > 100.0);
    CHECK(crown.minThickness > 0.45);
    double top = -1e9;
    for (const auto& p : crown.mesh.positions)
        top = std::max(top, f.local(glm::dvec3(p)).z);
    MESSAGE("crown top " << top << " (antagonist at 7.5), volume " << crown.volume << " mm3");
    CHECK(top == doctest::Approx(7.45).epsilon(0.01));

    adaptContacts(base, params, contacts, displacement, true, true);
    crown = buildCrown(base, params, displacement);
    CHECK(crown.watertight);
    const auto dist = contactDistances(crown, contacts);
    float minProx = 1e9f;
    for (std::size_t v = crown.outerBegin; v < dist.size(); ++v)
        if (!std::isnan(dist[v]))
            minProx = std::min(minProx, dist[v]);
    MESSAGE("closest contact after adaptation " << minProx);
    CHECK(minProx > -0.07f);
    CHECK(minProx < 0.06f);

    // Sculpting keeps the margin and the intaglio fixed and the minimum thickness.
    const glm::vec3 cusp = crown.mesh.positions[crown.mesh.vertexCount() - 1];
    applyBrush(crown, displacement, cusp, 2.0f, 2.0f, BrushMode::Remove);
    const CrownMesh carved = buildCrown(base, params, displacement);
    CHECK(carved.watertight);
    CHECK(carved.thickenedVertices > 0);
    CHECK(carved.minThickness > 0.45);
    for (std::uint32_t v = 0; v < carved.outerBegin; ++v)
        CHECK(carved.mesh.positions[v] == crown.mesh.positions[v]);

    CrownParameters coping = params;
    coping.coping = true;
    const CrownMesh cp = buildCrown(base, coping, {});
    CHECK(cp.watertight);
    CHECK(cp.volume > 20.0);
    CHECK(cp.volume < crown.volume);
}



TEST_CASE("Crown occlusal surface does not fold, even when it sits low over the preparation")
{
    const Fixture& f = fixture();
    const PrepScan prep(f.scan);
    const glm::vec3 click(transformPoint(f.toScan, glm::dvec3(0.0, 0.0, 4.5)));
    const auto det = detectMargin(prep, click);
    REQUIRE(det);
    auto die = extractDie(prep, det->line, click);
    REQUIRE(die);
    CrownFrame frame;
    frame.origin = centroid(det->line.points(*f.scan));
    frame.axis = det->occlusalDirection;
    estimateToothOrientation(*f.scan, det->line.points(*f.scan), frame);
    CrownParameters params;
    const CrownBase base = makeCrownBase(*die, frame, params);
    const ContactScene contacts = makeContactScene(*f.scan, base, &f.antagonist, glm::dmat4(1.0));
    fitProximal(base, contacts, params);
    REQUIRE(fitOcclusalHeight(base, contacts, params));
    // Faces of the occlusal cap must face occlusally; none may flip relative to the unedited shell.
    auto badFaces = [&](const CrownMesh& c) {
        int bad = 0;
        for (std::size_t t = 0; t < c.mesh.indices.size(); t += 3) {
            const auto a = c.mesh.indices[t], b = c.mesh.indices[t + 1], d = c.mesh.indices[t + 2];
            if (a < c.outerBegin || b < c.outerBegin || d < c.outerBegin)
                continue;
            const glm::vec3 n = glm::cross(c.mesh.positions[b] - c.mesh.positions[a], c.mesh.positions[d] - c.mesh.positions[a]);
            const glm::vec3 n0 = glm::cross(c.basePositions[b] - c.basePositions[a], c.basePositions[d] - c.basePositions[a]);
            const int ring = 1 + static_cast<int>((a - c.outerBegin) / static_cast<std::uint32_t>(c.columns));
            if ((ring > 22 && glm::dot(glm::dvec3(n), frame.axis) < 0.0) || glm::dot(n, n0) < 0.0f)
                ++bad;
        }
        return bad;
    };
    CHECK(badFaces(buildCrown(base, params, {})) == 0);
    std::vector<float> disp;
    adaptContacts(base, params, contacts, disp, true, true);
    CHECK(badFaces(buildCrown(base, params, disp)) == 0);
    CrownParameters low = params;
    low.crownHeight = params.crownHeight - 1.5; // occlusal surface down at the preparation top
    const CrownMesh lowCrown = buildCrown(base, low, {});
    CHECK(badFaces(lowCrown) == 0);
    CHECK(lowCrown.thickenedVertices > 0);
    CHECK(lowCrown.watertight);
}
