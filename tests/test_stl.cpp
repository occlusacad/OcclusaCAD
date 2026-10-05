#include "TestUtil.h"

#include "core/StlIO.h"

#include <doctest.h>

#include <fstream>

using namespace occlusa;

TEST_CASE("STL binary round trip welds vertices")
{
    test::TempDir tmp;
    const Mesh cube = test::makeCube(10.0f);
    const auto path = tmp.path() / "cube.stl";
    writeStlBinary(path, cube);
    CHECK(std::filesystem::file_size(path) == 84 + 50 * 12);

    const Mesh back = readStl(path);
    CHECK(back.triangleCount() == 12);
    CHECK(back.vertexCount() == 8);
    CHECK(back.normals.size() == 8);
    const Aabb b = back.bounds();
    CHECK(b.min.x == doctest::Approx(-5.0));
    CHECK(b.max.z == doctest::Approx(5.0));
}

TEST_CASE("STL write applies transform")
{
    test::TempDir tmp;
    const Mesh cube = test::makeCube(2.0f);
    const auto path = tmp.path() / "moved.stl";
    writeStlBinary(path, cube, glm::translate(glm::dmat4(1.0), glm::dvec3(10, 20, 30)));
    const Mesh back = readStl(path);
    CHECK(back.bounds().center().x == doctest::Approx(10.0));
    CHECK(back.bounds().center().y == doctest::Approx(20.0));
    CHECK(back.bounds().center().z == doctest::Approx(30.0));
}

TEST_CASE("STL ASCII parsing")
{
    const std::string ascii = R"(solid test
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 0 1 0
    endloop
  endfacet
  facet normal 0 0 1
    outer loop
      vertex 1 0 0
      vertex 1 1 0
      vertex 0 1 0
    endloop
  endfacet
endsolid test
)";
    const Mesh m = readStlFromMemory(std::span<const unsigned char>(reinterpret_cast<const unsigned char*>(ascii.data()), ascii.size()));
    CHECK(m.triangleCount() == 2);
    CHECK(m.vertexCount() == 4);
}

TEST_CASE("Binary STL whose header starts with 'solid' is read as binary")
{
    test::TempDir tmp;
    const auto path = tmp.path() / "solid_header.stl";
    writeStlBinary(path, test::makeCube(), glm::dmat4(1.0), "solid but actually binary");
    CHECK(readStl(path).triangleCount() == 12);
}

TEST_CASE("Garbage is rejected")
{
    const unsigned char junk[20] = {1, 2, 3};
    CHECK_THROWS_AS(readStlFromMemory(junk), MeshIOError);
}
