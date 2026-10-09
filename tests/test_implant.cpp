#include "TestUtil.h"

#include "core/MeshTopology.h"
#include "core/implant/Abutment.h"
#include "core/implant/ImplantLibrary.h"
#include "core/implant/ScanBodyFit.h"

#include <doctest.h>

#include <cmath>
#include <numbers>

using namespace occlusa;
using namespace occlusa::implant;

namespace {

std::shared_ptr<const Connection> generic(const char* id = "rp41")
{
    return ImplantLibraryRegistry::instance().connection(kGenericLibrary, id);
}

double radiusAt(const glm::dvec3& p)
{
    return std::hypot(p.x, p.y);
}

} // namespace

TEST_CASE("Generic implant library: closed interface and scan body with a top circle")
{
    const LibraryInfo lib = genericLibraryInfo();
    REQUIRE(lib.connections.size() == 3);
    for (const auto& ci : lib.connections) {
        INFO(ci.name);
        const auto c = generic(ci.id.c_str());
        CHECK(c->interfaceClosed);
        CHECK(isClosedManifold(*c->scanBody));
        CHECK(signedVolume(*c->interfaceMesh) > 0.0);
        CHECK(c->topRadius == doctest::Approx(ci.platformDiameter * 0.5).epsilon(0.01));
        CHECK(c->topHeight == doctest::Approx(0.6).epsilon(0.01));
    }
}

TEST_CASE("Implant library folders round-trip through library.json")
{
    test::TempDir dir;
    const LibraryInfo lib = genericLibraryInfo();
    std::vector<std::shared_ptr<const Connection>> conns;
    for (const auto& ci : lib.connections)
        conns.push_back(generic(ci.id.c_str()));
    LibraryInfo out = lib;
    out.id = "lab-generic";
    out.name = "Lab Generic";
    writeImplantLibrary(dir.path() / "lab", out, conns);

    auto& reg = ImplantLibraryRegistry::instance();
    CHECK(reg.scanDirectory(dir.path()) == 1);
    const auto info = reg.find("lab-generic");
    REQUIRE(info);
    CHECK(!info->builtIn);
    CHECK(info->connections.size() == 3);
    const auto c = reg.connection("lab-generic", "np35");
    CHECK(c->interfaceClosed);
    CHECK(c->topRadius == doctest::Approx(1.75).epsilon(0.01));
    CHECK_THROWS(reg.connection("lab-generic", "nope"));
}

TEST_CASE("Default abutment: margin 0.5 mm wider than the interface, straight emergence, closed solid")
{
    const auto c = generic();
    const AbutmentShape s = defaultAbutmentShape(*c, 1.5);
    const AbutmentGeometry g = buildAbutment(s, *c);
    CHECK(isClosedManifold(g.designed));
    CHECK(signedVolume(g.designed) > 0.0);
    for (int k = 0; k < kControlPoints; ++k) {
        const auto& m = g.marginPoints[static_cast<std::size_t>(k)];
        CHECK(2.0 * radiusAt(m) == doctest::Approx(2.0 * c->topRadius + 0.5).epsilon(1e-6));
        CHECK(m.z == doctest::Approx(c->topHeight + 1.5).epsilon(1e-6));
        // Mid points on the straight line: halfway in radius and height.
        const auto& mid = g.midPoints[static_cast<std::size_t>(k)];
        CHECK(radiusAt(mid) == doctest::Approx((c->topRadius + radiusAt(m)) * 0.5).epsilon(1e-6));
        CHECK(mid.z == doctest::Approx(c->topHeight + 0.75).epsilon(1e-6));
    }
    // The control points are evenly spaced.
    const double a0 = std::atan2(g.marginPoints[0].y, g.marginPoints[0].x);
    const double a1 = std::atan2(g.marginPoints[1].y, g.marginPoints[1].x);
    CHECK(std::remainder(a1 - a0, 2.0 * std::numbers::pi) == doctest::Approx(2.0 * std::numbers::pi / 9.0));

    const AbutmentSolid solid = finishAbutment(g, s, *c);
    CHECK(solid.error.empty());
    CHECK(solid.merged);
    CHECK(isClosedManifold(solid.mesh));
    // The interface is part of the result unchanged: its lowest point is still there.
    CHECK(solid.mesh.bounds().min.z == doctest::Approx(c->interfaceMesh->bounds().min.z).epsilon(1e-4));
}

TEST_CASE("Mid points control convexity; the locked core follows the margin; unlocking keeps the shape")
{
    const auto c = generic();
    AbutmentShape s = defaultAbutmentShape(*c, 2.0);
    s.marginRadius.fill(3.2);
    const AbutmentGeometry straight = buildAbutment(s, *c);
    s.midOffset[3] = 0.4;
    const AbutmentGeometry convex = buildAbutment(s, *c);
    CHECK(radiusAt(convex.midPoints[3]) > radiusAt(straight.midPoints[3]) + 0.2);
    CHECK(radiusAt(convex.midPoints[0]) == doctest::Approx(radiusAt(straight.midPoints[0])).epsilon(1e-6));

    // Locked core: raising a margin point raises the core above it.
    s.midOffset.fill(0.0);
    const double before = buildAbutment(s, *c).corePoints[2].z;
    s.marginHeight[2] += 1.0;
    const AbutmentGeometry raised = buildAbutment(s, *c);
    CHECK(raised.corePoints[2].z == doctest::Approx(before + 1.0).epsilon(1e-6));

    // Unlocking copies the outline; then a core point moves on its own.
    const double minR = minCoreRadius(s, *c);
    s.unlockCore(minR);
    const AbutmentGeometry unlocked = buildAbutment(s, *c);
    for (int k = 0; k < kControlPoints; ++k)
        CHECK(glm::length(unlocked.corePoints[static_cast<std::size_t>(k)] - raised.corePoints[static_cast<std::size_t>(k)]) < 1e-6);
    s.marginHeight[2] -= 1.0;
    CHECK(buildAbutment(s, *c).corePoints[2].z == doctest::Approx(raised.corePoints[2].z).epsilon(1e-6));
    s.coreTop[5] += 1.5;
    CHECK(buildAbutment(s, *c).corePoints[5].z == doctest::Approx(unlocked.corePoints[5].z + 1.5).epsilon(1e-6));
    CHECK(isClosedManifold(buildAbutment(s, *c).designed));
}

TEST_CASE("Thin walls around the screw channel are reported")
{
    const auto c = generic();
    AbutmentShape s = defaultAbutmentShape(*c, 1.5);
    s.marginRadius.fill(3.0);
    CHECK(buildAbutment(s, *c).warnings.empty());
    s.screwChannelDiameter = 3.9; // leaves less than 0.4 mm
    const AbutmentGeometry g = buildAbutment(s, *c);
    CHECK(g.minWall < c->info.minWall);
    CHECK(!g.warnings.empty());
}

TEST_CASE("A scan body is found from a click on its top")
{
    const auto c = generic();
    // Synthetic scan: the visible scan body plus a flat "gingiva" disc around it, in a rotated pose.
    glm::dmat4 truth = glm::rotate(glm::dmat4(1.0), glm::radians(23.0), glm::normalize(glm::dvec3(0.3, 1.0, 0.2)));
    truth = glm::rotate(truth, glm::radians(70.0), glm::dvec3(0, 0, 1));
    truth[3] = glm::dvec4(4.0, -7.0, 12.0, 1.0);
    Mesh scan = sampleSurface(*c->scanBody, 0.1);
    Mesh visible;
    for (std::size_t i = 0; i < scan.positions.size(); ++i)
        if (scan.positions[i].z > 4.5f) {
            visible.positions.push_back(glm::vec3(transformPoint(truth, glm::dvec3(scan.positions[i]))));
            visible.normals.push_back(glm::vec3(transformVector(truth, glm::dvec3(scan.normals[i]))));
        }
    for (double r = 2.6; r < 9.0; r += 0.15)
        for (double a = 0.0; a < 2.0 * std::numbers::pi; a += 0.15 / r) {
            visible.positions.push_back(glm::vec3(transformPoint(truth, glm::dvec3(r * std::cos(a), r * std::sin(a), 4.5))));
            visible.normals.push_back(glm::vec3(transformVector(truth, glm::dvec3(0, 0, 1))));
        }
    const glm::dvec3 click = transformPoint(truth, glm::dvec3(0.3, -0.2, 10.0));
    const auto fit = fitScanBody(visible, *c->scanBody, click);
    REQUIRE(fit);
    const glm::dvec3 platform = transformPoint(fit->implantToScan, glm::dvec3(0.0));
    const glm::dvec3 axis = transformVector(fit->implantToScan, glm::dvec3(0, 0, 1));
    const glm::dvec3 flat = transformVector(fit->implantToScan, glm::dvec3(1, 0, 0));
    CHECK(glm::length(platform - glm::dvec3(truth[3])) < 0.05);
    CHECK(glm::degrees(std::acos(std::clamp(glm::dot(axis, glm::dvec3(truth[2])), -1.0, 1.0))) < 0.5);
    CHECK(glm::degrees(std::acos(std::clamp(glm::dot(flat, glm::dvec3(truth[0])), -1.0, 1.0))) < 2.0);
    CHECK(fit->fractionWithin > 0.9);
}
