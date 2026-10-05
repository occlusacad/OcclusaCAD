#include "TestUtil.h"

#include "core/Geometry.h"
#include "core/IsoSurface.h"

#include <doctest.h>

#include <map>

using namespace occlusa;

TEST_CASE("Iso-surface of a sphere is accurate and closed")
{
    Volume v;
    v.geometry.dims = {48, 48, 48};
    v.geometry.spacing = {0.5, 0.5, 0.5};
    v.voxels.resize(v.geometry.voxelCount());
    const glm::dvec3 c = v.geometry.worldCenter();
    const double r = 8.0;
    for (int k = 0; k < 48; ++k)
        for (int j = 0; j < 48; ++j)
            for (int i = 0; i < 48; ++i) {
                const double d = glm::length(transformPoint(v.geometry.voxelToWorld(), glm::dvec3(i, j, k)) - c) - r;
                v.voxels[v.index(i, j, k)] = static_cast<std::int16_t>(std::clamp(500.0 - 500.0 * d, 0.0, 1000.0));
            }
    IsoSurfaceOptions opt;
    opt.isoValue = 500.0;
    const Mesh m = extractIsoSurface(v, opt);
    REQUIRE(m.triangleCount() > 500);
    double maxErr = 0.0;
    for (const auto& p : m.positions)
        maxErr = std::max(maxErr, std::abs(glm::length(glm::dvec3(p) - c) - r));
    CHECK(maxErr < 0.25); // half a voxel

    // Closed 2-manifold: every edge used exactly twice, with opposite orientation.
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> edges;
    for (std::size_t t = 0; t < m.indices.size(); t += 3)
        for (int e = 0; e < 3; ++e)
            edges[{m.indices[t + e], m.indices[t + (e + 1) % 3]}]++;
    std::size_t bad = 0;
    for (const auto& [e, n] : edges)
        if (n != 1 || !edges.count({e.second, e.first}))
            ++bad;
    CHECK(bad == 0);

    // Normals point outwards.
    std::size_t inward = 0;
    for (std::size_t i = 0; i < m.positions.size(); ++i)
        if (glm::dot(glm::dvec3(m.normals[i]), glm::dvec3(m.positions[i]) - c) < 0)
            ++inward;
    CHECK(inward == 0);
}

TEST_CASE("Ray casting meshes and volumes")
{
    const Mesh cube = test::makeCube(10.0f);
    const glm::dmat4 model = glm::translate(glm::dmat4(1.0), glm::dvec3(100, 0, 0));
    auto hit = raycastMesh(cube, model, Ray{{100, 0, -50}, {0, 0, 1}});
    REQUIRE(hit.has_value());
    CHECK(hit->point.z == doctest::Approx(-5.0));
    CHECK(hit->normal.z == doctest::Approx(-1.0));
    CHECK_FALSE(raycastMesh(cube, model, Ray{{0, 0, -50}, {0, 0, 1}}).has_value());

    const Volume vol = test::makeBlobVolume({60, 60, 50}, {0.5, 0.5, 0.5});
    const glm::dvec3 c = vol.geometry.worldCenter();
    auto p = vol.raycastIso(Ray{c + glm::dvec3(0, 0, -40), {0, 0, 1}}, 500.0);
    REQUIRE(p.has_value());
    // Main sphere radius 7 centred at c; the small sphere at (1,-4,4) does not cross the axis.
    CHECK(p->z == doctest::Approx(c.z - 7.0).epsilon(0.01));
}

TEST_CASE("Plane slicing a cube yields a square outline")
{
    const Mesh cube = test::makeCube(10.0f);
    const auto segs = slicePlane(cube, glm::dmat4(1.0), Plane{{0, 0, 1.0}, {0, 0, 1}});
    double length = 0.0;
    for (const auto& s : segs)
        length += glm::length(s.b - s.a);
    CHECK(length == doctest::Approx(40.0));
}
