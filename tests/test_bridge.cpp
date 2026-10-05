#include "TestUtil.h"

#include "core/IsoSurface.h"
#include "core/MeshBvh.h"
#include "core/MeshTopology.h"
#include "core/crown/Bridge.h"
#include "core/crown/InsertionAxis.h"
#include "core/crown/Margin.h"

#include <doctest.h>

#include <cmath>
#include <memory>

using namespace occlusa;
using namespace occlusa::crown;

namespace {

// Two shoulder preparations (47 at x = -11, 45 at x = +11, local frame z occlusal) on a
// gingiva slab whose top (the edentulous ridge of 46) is at z = -1.5.
double ellipse2d(double x, double y, double rx, double ry)
{
    return (std::sqrt((x / rx) * (x / rx) + (y / ry) * (y / ry)) - 1.0) * std::min(rx, ry);
}

double smin(double a, double b, double k)
{
    const double h = std::clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
    return glm::mix(b, a, h) - k * h * (1.0 - h);
}

double prepAt(const glm::dvec3& p, double cx)
{
    const double x = p.x - cx, y = p.y;
    const double top = 4.5, rho = 0.6;
    const double f = std::clamp(p.z, 0.0, top) / top;
    const double e = ellipse2d(x, y, 4.0 - 0.6 * f, 3.6 - 0.6 * f);
    const double qx = std::max(e + rho, 0.0), qz = std::max(p.z - top + rho, 0.0);
    double prep = std::sqrt(qx * qx + qz * qz) - rho + std::min(std::max(e + rho, p.z - top + rho), 0.0);
    prep = std::max(prep, -6.0 - p.z);
    const double stump = std::max({ellipse2d(x, y, 4.8, 4.4), p.z, -6.0 - p.z});
    return std::min(prep, stump);
}

struct BridgeFixture {
    std::shared_ptr<Mesh> scan;
    Mesh antagonist;

    BridgeFixture()
    {
        Volume v;
        const double res = 0.2;
        const glm::dvec3 lo(-18.0, -9.0, -6.0), hi(18.0, 9.0, 7.0);
        v.geometry.spacing = glm::dvec3(res);
        v.geometry.dims = glm::ivec3(glm::ceil((hi - lo) / res));
        v.geometry.origin = lo;
        v.voxels.resize(v.geometry.voxelCount());
        for (int z = 0; z < v.geometry.dims.z; ++z)
            for (int y = 0; y < v.geometry.dims.y; ++y)
                for (int x = 0; x < v.geometry.dims.x; ++x) {
                    const glm::dvec3 p = lo + glm::dvec3(x, y, z) * res;
                    const double gum = std::max({p.z + 1.5 + 0.02 * p.y * p.y, -8.0 - p.z, std::abs(p.y) - 7.0});
                    const double sd = smin(std::min(prepAt(p, -11.0), prepAt(p, 11.0)), gum, 0.12);
                    v.voxels[v.index(x, y, z)] = static_cast<std::int16_t>(std::clamp(-sd * 1000.0, -32000.0, 32000.0));
                }
        IsoSurfaceOptions opt;
        opt.isoValue = 0.0;
        Mesh m = extractIsoSurface(v, opt);
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
        open.computeVertexNormals();
        scan = std::make_shared<Mesh>(std::move(open));
        const int n = 60;
        for (int j = 0; j <= n; ++j)
            for (int i = 0; i <= n; ++i)
                antagonist.positions.emplace_back(-20.0f + 40.0f * i / n, -12.0f + 24.0f * j / n, 7.0f);
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                const std::uint32_t a = j * (n + 1) + i, b = a + 1, c = a + n + 1, d = c + 1;
                antagonist.indices.insert(antagonist.indices.end(), {a, c, b, b, c, d});
            }
        antagonist.computeVertexNormals();
    }
};

} // namespace

TEST_CASE("Bridges are found from runs of adjacent restorations with a pontic")
{
    const auto bridges = findBridges({{45, "anatomic_crown"}, {46, "pontic"}, {47, "anatomic_crown"}, {36, "anatomic_crown"}, {11, "pontic"},
                                      {14, "coping"}, {15, "anatomic_crown"}, {16, "pontic"}, {24, "veneer"}});
    REQUIRE(bridges.size() == 2);
    CHECK(bridges[0] == std::vector<int>{16, 15, 14}); // upper arch first, in arch order (cantilever)
    CHECK(bridges[1] == std::vector<int>{47, 46, 45});
    CHECK(adjacentTeeth(11, 21));
    CHECK_FALSE(adjacentTeeth(11, 41));
    CHECK(findBridges({{36, "anatomic_crown"}, {37, "anatomic_crown"}}).empty()); // splinted crowns, no pontic
}

TEST_CASE("Pontics are laid out between abutments by tooth width, cantilevers along the arch")
{
    std::vector<BridgeLayoutUnit> units = {{47, false, {-11, 0, 0}, {1, 0, 0}}, {46, true, {}, {}}, {45, false, {11, 0, 0}, {-1, 0, 0}}};
    REQUIRE(layoutPontics(units, {0, 0, 1}));
    const double expected = -11.0 + 22.0 * (10.5 / 2 + 11.0 / 2) / (10.5 / 2 + 11.0 + 7.1 / 2);
    CHECK(units[1].center.x == doctest::Approx(expected));
    CHECK(ponticMesial(units, 1, {0, 0, 1}).x == doctest::Approx(1.0)); // towards 45 (mesial)

    std::vector<BridgeLayoutUnit> cant = {{16, true, {}, {}}, {15, false, {0, 0, 0}, {0, 1, 0}}};
    REQUIRE(layoutPontics(cant, {0, 0, 1}));
    CHECK(cant[0].center.y == doctest::Approx(-(6.8 / 2 + 10.2 / 2))); // distal of the abutment
}

TEST_CASE("Connector geometry has the requested cross-section")
{
    ConnectorSpec c = placeConnector({-5, 0, 0}, {5, 0, 0}, {0, 0, 1}, {0, 0, 0}, 1.0, 6.0, 9.0, 1.3);
    CHECK(c.width() * c.height() * 3.14159265 / 4.0 == doctest::Approx(9.0));
    const Mesh m = makeConnectorMesh(c, 64);
    CHECK(isClosedManifold(m));
    const double area = crossSectionArea(m, Plane{c.center, c.direction}, c.center, 5.0);
    CHECK(area >= 9.0 - 1e-4);
    CHECK(area == doctest::Approx(9.0).epsilon(0.01));
    std::string warning;
    placeConnector({-5, 0, 0}, {5, 0, 0}, {0, 0, 1}, {0, 0, 0}, 1.0, 3.0, 9.0, 1.3, &warning);
    CHECK_FALSE(warning.empty()); // 3.4 mm high connector in 2 mm
}

TEST_CASE("Connector edits move and resize a connector in its own frame")
{
    const ConnectorSpec base = placeConnector({-5, 0, 0}, {5, 0, 0}, {0, 0, 1}, {0, 0, 0}, 0.0, 8.0, 9.0, 1.3);
    CHECK_FALSE(checkConnectorFit(base, {0, 0, 0}, 0.0, 8.0));
    ConnectorEdit e;
    CHECK(e.isDefault());
    e.offset = {0.5, -0.4, 1.0};
    e.area = 12.0;
    e.heightRatio = 1.0;
    e.length = 4.0;
    CHECK_FALSE(e.isDefault());
    const ConnectorSpec c = applyConnectorEdit(base, e);
    CHECK(c.center.x == doctest::Approx(base.center.x + 0.5));
    CHECK(c.center.y == doctest::Approx(base.center.y - 0.4)); // y = z x x = world +y here
    CHECK(c.center.z == doctest::Approx(base.center.z + 1.0));
    CHECK(c.width() == doctest::Approx(c.height()));
    CHECK(c.length == doctest::Approx(4.0));
    const Mesh m = makeConnectorMesh(c, 64);
    CHECK(crossSectionArea(m, Plane{c.center, c.direction}, c.center, 5.0) >= 12.0 - 1e-4);
    // Moved up against the marginal ridges / down into the embrasure.
    ConnectorEdit up;
    up.offset.z = 3.0;
    const auto high = checkConnectorFit(applyConnectorEdit(base, up), {0, 0, 0}, 0.0, 8.0);
    REQUIRE(high);
    CHECK(high->find("above the marginal ridges") != std::string::npos);
    up.offset.z = -3.0;
    const auto low = checkConnectorFit(applyConnectorEdit(base, up), {0, 0, 0}, 0.0, 8.0);
    REQUIRE(low);
    CHECK(low->find("embrasure") != std::string::npos);
    ConnectorEdit huge;
    huge.area = 60.0;
    CHECK(checkConnectorFit(applyConnectorEdit(base, huge), {0, 0, 0}, 0.0, 8.0)->find("only") != std::string::npos);
}

TEST_CASE("Three-unit bridge: common axis, pontic, connectors and a watertight union")
{
    const BridgeFixture f;
    const PrepScan prep(f.scan);
    std::vector<BridgeLayoutUnit> layout = {{47, false, {}, {}}, {46, true, {}, {}}, {45, false, {}, {}}};
    std::vector<std::shared_ptr<DieRegion>> dies(3);
    std::vector<glm::dvec3> margins(3);
    for (std::size_t i : {std::size_t{0}, std::size_t{2}}) {
        const double cx = i == 0 ? -11.0 : 11.0;
        const glm::vec3 click(static_cast<float>(cx), 0.0f, 4.5f);
        std::string err;
        const auto det = detectMargin(prep, click, {}, &err);
        INFO(err);
        REQUIRE(det);
        auto die = extractDie(prep, det->line, click, &err);
        REQUIRE(die);
        dies[i] = std::make_shared<DieRegion>(std::move(*die));
        layout[i].center = centroid(det->line.points(*f.scan));
        layout[i].mesial = glm::dvec3(i == 0 ? 1.0 : -1.0, 0.0, 0.0);
    }
    const glm::dvec3 axis = optimizeCommonAxis({&dies[0]->mesh, &dies[2]->mesh}, glm::dvec3(0, 0, 1));
    CHECK(axisDivergence(axis, {0, 0, 1}) < 2.0);
    REQUIRE(layoutPontics(layout, axis));

    // Bases: abutments on their dies, the pontic on the ridge.
    std::vector<CrownBase> bases;
    std::vector<CrownParameters> params(3);
    for (std::size_t i = 0; i < 3; ++i) {
        CrownFrame fr;
        fr.axis = axis;
        params[i].kind = toothKindFromFdi(layout[i].tooth);
        if (layout[i].pontic) {
            fr.mesial = ponticMesial(layout, i, axis);
            std::string err;
            auto base = makePonticBase(prep.bvh(), layout[i].center, axis, fr.mesial, 3.2, 2.8, 0.0, &err);
            INFO(err);
            REQUIRE(base);
            CHECK(base->mesh.positions[0].z == doctest::Approx(-1.5).epsilon(0.1)); // on the ridge
            fr.origin = centroid([&] {
                std::vector<glm::vec3> b;
                for (auto v : base->boundary)
                    b.push_back(base->mesh.positions[v]);
                return b;
            }());
            params[i] = ponticParameters(params[i]);
            fr.buccal = glm::cross(axis, fr.mesial);
            bases.push_back(makeCrownBase(*base, fr, params[i]));
        } else {
            fr.origin = layout[i].center;
            fr.mesial = layout[i].mesial;
            fr.buccal = glm::cross(axis, fr.mesial);
            bases.push_back(makeCrownBase(*dies[i], fr, params[i]));
        }
    }
    std::vector<BridgeFitUnit> fit;
    for (std::size_t i = 0; i < 3; ++i)
        fit.push_back({bases[i].frame, &params[i]});
    fitBridgeWidths(fit);
    CHECK(params[0].halfMesial == doctest::Approx(glm::length(layout[1].center - layout[0].center) * 0.5 + 0.15).epsilon(0.02));

    std::vector<CrownMesh> crowns;
    for (std::size_t i = 0; i < 3; ++i) {
        const ContactScene contacts = makeContactScene(*f.scan, bases[i], &f.antagonist, glm::dmat4(1.0));
        REQUIRE(fitOcclusalHeight(bases[i], contacts, params[i]));
        crowns.push_back(buildCrown(bases[i], params[i]));
        CHECK(crowns.back().watertight);
    }
    // The pontic's basal surface rests on the ridge.
    double lowest = 1e9;
    for (const auto& p : crowns[1].mesh.positions)
        lowest = std::min(lowest, static_cast<double>(p.z));
    CHECK(lowest == doctest::Approx(-1.5).epsilon(0.15));

    // Connectors between neighbours, clipped by the preparation spaces.
    std::vector<ConnectorSpec> connectors;
    for (std::size_t i = 0; i + 1 < 3; ++i) {
        std::string warning;
        connectors.push_back(placeConnector(bases[i].frame.origin, bases[i + 1].frame.origin, axis, bases[0].frame.origin, 1.0, 5.5, 9.0, 1.3, &warning));
        CHECK(warning.empty());
    }
    const std::vector<Mesh> cavities = {makeCavity(bases[0]), makeCavity(bases[2])};
    for (const auto& c : cavities)
        CHECK(isClosedManifold(c));
    const BridgeUnion u = uniteBridge({{&crowns[0].mesh, 0}, {&crowns[1].mesh, -1}, {&crowns[2].mesh, 1}}, connectors, cavities);
    INFO(u.error);
    REQUIRE(u.ok);
    CHECK(u.watertight);
    double sum = 0.0;
    for (const auto& c : crowns)
        sum += c.volume;
    MESSAGE("bridge volume " << u.volume << " (units " << sum << "), connector areas " << u.connectorAreas[0] << ", " << u.connectorAreas[1]);
    CHECK(u.volume > sum * 0.98);
    CHECK(u.volume < sum + 2.0 * 60.0);
    for (double a : u.connectorAreas)
        CHECK(a >= 9.0);

    // Units standing 1 mm apart are joined by the connectors alone: one solid, full connector area.
    {
        std::vector<CrownParameters> apart = params;
        std::vector<BridgeFitUnit> fitApart;
        for (std::size_t i = 0; i < 3; ++i)
            fitApart.push_back({bases[i].frame, &apart[i]});
        fitBridgeWidths(fitApart, -0.5);
        std::vector<CrownMesh> separate;
        for (std::size_t i = 0; i < 3; ++i)
            separate.push_back(buildCrown(bases[i], apart[i]));
        const std::vector<BridgeUnionPart> meshes = {{&separate[0].mesh, 0}, {&separate[1].mesh, -1}, {&separate[2].mesh, 1}};
        const BridgeUnion joined = uniteBridge(meshes, connectors, cavities);
        REQUIRE(joined.ok);
        CHECK(joined.watertight);
        // Connected components of the result.
        const MeshAdjacency adj(joined.mesh);
        std::vector<int> comp(joined.mesh.vertexCount(), -1);
        int count = 0;
        for (std::size_t s0 = 0; s0 < comp.size(); ++s0) {
            if (comp[s0] >= 0)
                continue;
            std::vector<std::uint32_t> stack{static_cast<std::uint32_t>(s0)};
            comp[s0] = count;
            while (!stack.empty()) {
                const auto v = stack.back();
                stack.pop_back();
                for (auto w : adj.neighbors(v))
                    if (comp[w] < 0) {
                        comp[w] = count;
                        stack.push_back(w);
                    }
            }
            ++count;
        }
        CHECK(count == 1);
        double unitSum = 0.0;
        for (const auto& c : separate)
            unitSum += c.volume;
        MESSAGE("separate units " << unitSum << " mm3 -> bridge " << joined.volume << " mm3, connectors " << joined.connectorAreas[0] << ", "
                                  << joined.connectorAreas[1] << " mm2");
        CHECK(joined.volume > unitSum + 5.0);
        for (double a : joined.connectorAreas)
            CHECK(a >= 9.0 * 0.995);
    }

    // An enlarged, raised connector is honoured by the merge.
    {
        ConnectorEdit e;
        e.area = 14.0;
        e.offset.z = 0.3;
        std::vector<ConnectorSpec> edited = connectors;
        edited[0] = applyConnectorEdit(edited[0], e);
        const BridgeUnion bigger = uniteBridge({{&crowns[0].mesh, 0}, {&crowns[1].mesh, -1}, {&crowns[2].mesh, 1}}, edited, cavities);
        REQUIRE(bigger.ok);
        CHECK(bigger.watertight);
        CHECK(bigger.connectorAreas[0] >= 14.0 - 1e-3);
    }

    // The connectors never fill a preparation: every intaglio point is still on the bridge surface.
    auto shared = std::make_shared<const Mesh>(u.mesh);
    const MeshBvh bvh(shared);
    for (std::size_t i : {std::size_t{0}, std::size_t{2}}) {
        float worst = 0.0f;
        for (const auto& p : bases[i].intaglio->positions)
            if (auto cp = bvh.closestPoint(p, 1.0f))
                worst = std::max(worst, cp->distance);
            else
                worst = 1.0f;
        CHECK(worst < 0.01f);
    }
}
