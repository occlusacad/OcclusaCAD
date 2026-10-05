#include "TestUtil.h"

#include "core/IsoSurface.h"
#include "core/Registration.h"

#include <doctest.h>

#include <random>

using namespace occlusa;

namespace {

glm::dmat4 makeRigid(double angleDeg, glm::dvec3 axis, glm::dvec3 t)
{
    glm::dmat4 m = glm::rotate(glm::dmat4(1.0), glm::radians(angleDeg), glm::normalize(axis));
    m[3] = glm::dvec4(t, 1.0);
    return m;
}

double maxMatrixDiff(const glm::dmat4& a, const glm::dmat4& b)
{
    double d = 0.0;
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
            d = std::max(d, std::abs(a[c][r] - b[c][r]));
    return d;
}

} // namespace

TEST_CASE("Horn point-pair registration recovers a rigid transform")
{
    const glm::dmat4 truth = makeRigid(37.0, {0.3, -0.8, 0.5}, {12.0, -4.0, 30.0});
    std::vector<glm::dvec3> src = {{0, 0, 0}, {10, 0, 0}, {0, 12, 0}, {3, 4, 9}, {-5, 2, 1}};
    std::vector<glm::dvec3> dst;
    for (const auto& p : src)
        dst.push_back(transformPoint(truth, p));
    auto res = rigidFromPointPairs(src, dst);
    REQUIRE(res.has_value());
    CHECK(maxMatrixDiff(res->transform, truth) < 1e-9);
    CHECK(res->rms < 1e-9);
    CHECK(isRigid(res->transform));
}

TEST_CASE("Horn works with exactly three points and noise")
{
    const glm::dmat4 truth = makeRigid(-120.0, {1, 1, 0}, {-3.0, 8.0, 1.0});
    std::vector<glm::dvec3> src = {{0, 0, 0}, {20, 0, 0}, {0, 15, 3}};
    std::vector<glm::dvec3> dst;
    for (const auto& p : src)
        dst.push_back(transformPoint(truth, p) + glm::dvec3(0.01, -0.01, 0.005));
    auto res = rigidFromPointPairs(src, dst);
    REQUIRE(res.has_value());
    CHECK(maxMatrixDiff(res->transform, truth) < 0.02);
}

TEST_CASE("Collinear points are rejected")
{
    std::vector<glm::dvec3> src = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}};
    CHECK_FALSE(rigidFromPointPairs(src, src).has_value());
}

TEST_CASE("ICP refines a perturbed surface onto the CBCT iso-surface")
{
    const Volume vol = test::makeBlobVolume({80, 80, 64}, {0.5, 0.5, 0.5});
    IsoSurfaceOptions iso;
    iso.isoValue = 500.0;
    const Mesh surface = extractIsoSurface(vol, iso);
    REQUIRE(surface.triangleCount() > 1000);
    const IcpTarget target = IcpTarget::fromMesh(surface);

    // The "scan" is the same surface expressed in its own coordinate system.
    const glm::dmat4 truth = makeRigid(25.0, {0.2, 0.9, -0.3}, {5.0, -7.0, 3.0}); // scan local -> world
    Mesh scan = surface;
    const glm::dmat4 worldToLocal = glm::inverse(truth);
    for (auto& p : scan.positions)
        p = glm::vec3(transformPoint(worldToLocal, glm::dvec3(p)));
    scan.computeVertexNormals();

    // Start from a moderately wrong alignment (as after point-pair picking).
    const glm::dmat4 initial = makeRigid(4.0, {1, 0, 0}, {0.8, -0.6, 0.5}) * truth;
    IcpOptions opt;
    opt.maxCorrespondenceDistance = 4.0;
    opt.trimFraction = 0.9;
    const IcpResult res = refineIcp(scan, initial, target, opt);
    CHECK(res.rms < 0.05);
    // Compare transforms by the displacement of points on the surface.
    double maxErr = 0.0;
    for (std::size_t i = 0; i < scan.positions.size(); i += 97)
        maxErr = std::max(maxErr, glm::length(transformPoint(res.transform, glm::dvec3(scan.positions[i])) -
                                              transformPoint(truth, glm::dvec3(scan.positions[i]))));
    CHECK(maxErr < 0.05);

    const DeviationStats st = measureDeviation(scan, res.transform, target);
    CHECK(st.fractionWithin > 0.99);
}
