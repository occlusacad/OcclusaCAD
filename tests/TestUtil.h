#pragma once

#include "core/Mesh.h"
#include "core/Volume.h"

#include <filesystem>
#include <random>
#include <string>

namespace occlusa::test {

// Unique temporary directory removed on destruction.
class TempDir {
public:
    TempDir()
    {
        std::random_device rd;
        path_ = std::filesystem::temp_directory_path() / ("occlusacad_test_" + std::to_string(rd()) + std::to_string(rd()));
        std::filesystem::create_directories(path_);
    }
    ~TempDir()
    {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }
    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

inline Mesh makeCube(float size = 10.0f)
{
    Mesh m;
    const float s = size * 0.5f;
    m.positions = {{-s, -s, -s}, {s, -s, -s}, {s, s, -s}, {-s, s, -s}, {-s, -s, s}, {s, -s, s}, {s, s, s}, {-s, s, s}};
    m.indices = {0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 2, 3, 7, 2, 7, 6, 1, 2, 6, 1, 6, 5, 3, 0, 4, 3, 4, 7};
    m.computeVertexNormals();
    return m;
}

// Volume containing a union of spheres (an asymmetric "blob"); voxel value 1000 inside, 0 outside,
// with a linear ramp across the boundary so iso-surfaces are sub-voxel accurate.
inline Volume makeBlobVolume(glm::ivec3 dims, glm::dvec3 spacing, const glm::dmat3& direction = glm::dmat3(1.0),
                             glm::dvec3 origin = glm::dvec3(-20.0, -20.0, -15.0))
{
    Volume v;
    v.geometry.dims = dims;
    v.geometry.spacing = spacing;
    v.geometry.direction = direction;
    v.geometry.origin = origin;
    v.voxels.resize(v.geometry.voxelCount());
    const glm::dvec3 c = v.geometry.worldCenter();
    struct Sphere {
        glm::dvec3 c;
        double r;
    };
    const Sphere spheres[] = {{c + glm::dvec3(0, 0, 0), 7.0}, {c + glm::dvec3(6, 2, 1), 4.5}, {c + glm::dvec3(-3, 5, -2), 3.5},
                              {c + glm::dvec3(1, -4, 4), 3.0}};
    const glm::dmat4 v2w = v.geometry.voxelToWorld();
    for (int k = 0; k < dims.z; ++k)
        for (int j = 0; j < dims.y; ++j)
            for (int i = 0; i < dims.x; ++i) {
                const glm::dvec3 w = transformPoint(v2w, glm::dvec3(i, j, k));
                double sd = 1e9;
                for (const auto& s : spheres)
                    sd = std::min(sd, glm::length(w - s.c) - s.r);
                const double val = std::clamp(500.0 - sd * 500.0, 0.0, 1000.0); // iso 500 at the surface
                v.voxels[v.index(i, j, k)] = static_cast<std::int16_t>(std::lround(val));
            }
    return v;
}

} // namespace occlusa::test
