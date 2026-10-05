#include "TestUtil.h"

#include "core/Dental.h"
#include "core/MeshBvh.h"
#include "core/MeshTopology.h"
#include "core/StlIO.h"
#include "core/crown/AnatomyGenerator.h"
#include "core/crown/ToothLibrary.h"

#include <doctest.h>

#include <cmath>
#include <fstream>
#include <memory>

using namespace occlusa;
using namespace occlusa::crown;

TEST_CASE("Generated teeth are closed and have their nominal dimensions")
{
    for (auto style : {LibraryStyle::Natural, LibraryStyle::Young, LibraryStyle::Mature})
        for (bool upper : {false, true})
            for (int k = 0; k < 7; ++k) {
                const auto kind = static_cast<ToothKind>(k);
                const Mesh m = generateToothCrown(kind, upper, style, 96);
                INFO(toString(style) << " " << (upper ? "upper " : "lower ") << toString(kind));
                CHECK(isClosedManifold(m));
                CHECK(signedVolume(m) > 50.0);
                const Aabb box = m.bounds();
                CHECK(box.min.z == doctest::Approx(0.0).epsilon(1e-6)); // cervical line at z = 0
                if (style == LibraryStyle::Natural) {
                    const ToothTemplate& t = toothTemplate(kind, upper);
                    CHECK(box.size().x == doctest::Approx(t.mesioDistal).epsilon(0.08));
                    CHECK(box.max.z == doctest::Approx(t.crownHeight).epsilon(0.01));
                }
            }
    // Style changes the relief: worn teeth are flatter than young ones.
    auto relief = [](LibraryStyle s) {
        const auto shape = sampleToothShape(generateToothCrown(ToothKind::FirstMolar, false, s));
        return shape->height - shape->occlusalLevel;
    };
    CHECK(relief(LibraryStyle::Young) > relief(LibraryStyle::Natural));
    CHECK(relief(LibraryStyle::Natural) > relief(LibraryStyle::Mature));
}

TEST_CASE("Sampled tooth shapes lie on the tooth surface")
{
    for (int k : {0, 2, 5}) {
        auto mesh = std::make_shared<Mesh>(generateToothCrown(static_cast<ToothKind>(k), k == 0, LibraryStyle::Natural));
        const auto shape = sampleToothShape(*mesh, 0.0);
        const MeshBvh bvh(mesh);
        // Rebuild 3D points from the profiles: (r, z) around the shape's sampling centre.
        glm::dvec2 lo(1e9), hi(-1e9);
        for (const auto& p : mesh->positions)
            if (p.z > 0.3 * mesh->bounds().max.z) {
                lo = glm::min(lo, glm::dvec2(p.x, p.y));
                hi = glm::max(hi, glm::dvec2(p.x, p.y));
            }
        const glm::dvec2 c = 0.5 * (lo + hi);
        double worst = 0.0, sum = 0.0;
        int count = 0;
        for (int a = 0; a < ToothShape::kAzimuths; a += 7)
            for (int j = 0; j < ToothShape::kProfile; ++j) {
                const double th = 2.0 * 3.14159265358979 * a / ToothShape::kAzimuths;
                const glm::dvec2 q = shape->profiles[static_cast<std::size_t>(a * ToothShape::kProfile + j)];
                const glm::vec3 p(static_cast<float>(c.x + q.x * std::cos(th)), static_cast<float>(c.y + q.x * std::sin(th)), static_cast<float>(q.y));
                const double d = bvh.closestPoint(p, 5.0f)->distance;
                worst = std::max(worst, d);
                sum += d;
                ++count;
            }
        INFO(toString(static_cast<ToothKind>(k)));
        CHECK(sum / count < 0.02);
        CHECK(worst < 0.2); // narrow fissures are bridged where rays graze their walls
        CHECK(shape->profiles[ToothShape::kProfile - 1].x == doctest::Approx(0.0)); // ends on the axis
        CHECK(shape->halfMesial + shape->halfDistal == doctest::Approx(mesh->bounds().size().x).epsilon(0.03));
    }
}

TEST_CASE("Tooth libraries round-trip through library folders, with tooth frames")
{
    test::TempDir dir;
    const auto folder = dir.path() / "lab";
    // A molar stored in an arbitrary pose; the manifest frame maps it back into the tooth frame.
    const Mesh molar = generateToothCrown(ToothKind::FirstMolar, false, LibraryStyle::Natural, 64);
    const glm::dmat4 pose = glm::translate(glm::dmat4(1.0), glm::dvec3(30, -12, 8)) * glm::rotate(glm::dmat4(1.0), 1.1, glm::normalize(glm::dvec3(1, 2, 0.5)));
    Mesh posed = molar;
    for (auto& p : posed.positions)
        p = glm::vec3(transformPoint(pose, glm::dvec3(p)));
    std::filesystem::create_directories(folder);
    writeStlBinary(folder / "36.stl", posed, glm::dmat4(1.0), "test");
    writeStlBinary(folder / "11.stl", generateToothCrown(ToothKind::CentralIncisor, true, LibraryStyle::Young, 64), glm::dmat4(1.0), "test");

    ToothLibraryManifest m = manifestFromStlFolder(folder, "Lab Library 2026");
    CHECK(m.id == "lab-library-2026");
    REQUIRE(m.teeth.size() == 2);
    for (auto& t : m.teeth)
        if (t.fdi == 36) {
            t.origin = transformPoint(pose, glm::dvec3(0.0));
            t.mesial = transformVector(pose, glm::dvec3(1, 0, 0));
            t.buccal = transformVector(pose, glm::dvec3(0, 1, 0));
            t.occlusal = transformVector(pose, glm::dvec3(0, 0, 1));
            t.cervicalZ = 0.0;
        }
    m.license = "CC0-1.0";
    writeToothLibraryManifest(folder, m);

    const ToothLibraryManifest back = readToothLibraryManifest(folder);
    CHECK(back.name == "Lab Library 2026");
    CHECK(back.license == "CC0-1.0");
    const auto entry = std::find_if(back.teeth.begin(), back.teeth.end(), [](const auto& t) { return t.fdi == 36; });
    REQUIRE(entry != back.teeth.end());
    const Mesh local = loadLibraryTooth(folder, *entry);
    REQUIRE(local.vertexCount() == molar.vertexCount());
    const MeshBvh original(std::make_shared<const Mesh>(molar));
    double worst = 0.0;
    for (const auto& q : local.positions)
        worst = std::max(worst, static_cast<double>(original.closestPoint(q, 5.0f)->distance));
    CHECK(worst < 1e-3);
    CHECK(signedVolume(local) > 0.0);

    // Registered libraries serve their teeth (both sides of the jaw) and fall back to the default.
    auto& reg = ToothLibraryRegistry::instance();
    CHECK(reg.scanDirectory(dir.path()) == 1);
    const auto info = reg.find("lab-library-2026");
    REQUIRE(info);
    CHECK(info->teeth == std::vector<int>{11, 36});
    const auto right = reg.toothMesh("lab-library-2026", ToothKind::FirstMolar, false); // 46 uses the 36 shape
    REQUIRE(right);
    CHECK(right->vertexCount() == molar.vertexCount());
    const auto canine = reg.shape("lab-library-2026", ToothKind::Canine, true); // not in the library
    const auto fallback = reg.shape(ToothLibraryRegistry::kDefaultLibrary, ToothKind::Canine, true);
    CHECK(canine->height == doctest::Approx(fallback->height));
    CHECK(reg.find("occlusacad-natural")->builtIn);
    CHECK_THROWS(reg.addFolder(dir.path() / "missing"));

    // Export: a built-in library written to disk is a valid library folder.
    std::vector<std::pair<int, Mesh>> teeth = {{16, generateToothCrown(ToothKind::FirstMolar, true, LibraryStyle::Mature, 64)}};
    ToothLibraryManifest em;
    em.name = "Export";
    writeToothLibrary(dir.path() / "export", em, teeth);
    const auto exported = readToothLibraryManifest(dir.path() / "export");
    REQUIRE(exported.teeth.size() == 1);
    CHECK(isClosedManifold(loadLibraryTooth(dir.path() / "export", exported.teeth[0])));
}
