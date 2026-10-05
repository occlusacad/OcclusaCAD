#include "core/IsoSurface.h"

#include <cmath>
#include <format>

namespace occlusa {

Mesh extractIsoSurface(const Volume& vol, const IsoSurfaceOptions& opt, const ProgressFn& progress)
{
    Mesh mesh;
    if (vol.empty())
        return mesh;
    const auto& d = vol.geometry.dims;
    const int s = std::max(1, opt.step);
    glm::ivec3 lo = glm::clamp(opt.roiMin, glm::ivec3(0), d);
    glm::ivec3 hi = (opt.roiMax == glm::ivec3(0)) ? d : glm::clamp(opt.roiMax, glm::ivec3(0), d);
    if (glm::any(glm::lessThanEqual(hi - lo, glm::ivec3(1))))
        return mesh;

    // Grid of sample points (subsampled).
    const glm::ivec3 n = (hi - lo - 1) / s + 1;
    if (n.x < 2 || n.y < 2 || n.z < 2)
        return mesh;

    // Work in stored units to avoid a multiply per sample. Slope is positive in practice.
    const double slope = vol.rescaleSlope != 0.0 ? vol.rescaleSlope : 1.0;
    const float iso = static_cast<float>((opt.isoValue - vol.rescaleIntercept) / slope);
    const float sign = slope > 0 ? 1.0f : -1.0f;

    auto sample = [&](int gx, int gy, int gz) -> float {
        const int x = lo.x + gx * s, y = lo.y + gy * s, z = lo.z + gz * s;
        return sign * (static_cast<float>(vol.voxels[vol.index(std::min(x, d.x - 1), std::min(y, d.y - 1), std::min(z, d.z - 1))]) - iso);
    };

    // Rolling buffers: grid values for layers z and z+1, cell vertex indices for cell layers z-1 and z.
    const std::size_t layerPts = static_cast<std::size_t>(n.x) * n.y;
    const int cx = n.x - 1, cy = n.y - 1;
    const std::size_t layerCells = static_cast<std::size_t>(cx) * cy;
    std::vector<float> v0(layerPts), v1(layerPts);
    std::vector<std::int32_t> cellPrev(layerCells, -1), cellCur(layerCells, -1);

    auto loadLayer = [&](std::vector<float>& dst, int gz) {
        for (int y = 0; y < n.y; ++y)
            for (int x = 0; x < n.x; ++x)
                dst[static_cast<std::size_t>(y) * n.x + x] = sample(x, y, gz);
    };

    const glm::dmat4 v2w = vol.geometry.voxelToWorld();
    auto toWorld = [&](const glm::dvec3& g) {
        const glm::dvec3 voxel = glm::dvec3(lo) + g * static_cast<double>(s);
        return glm::vec3(transformPoint(v2w, voxel));
    };

    static constexpr int kCorner[8][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};
    static constexpr int kEdge[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3}, {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

    // Quads are passed counter-clockwise around their grid edge (right-handed: the quad normal is the
    // edge's +axis). Orientation comes from the grid topology, never from the (possibly folded)
    // quad geometry, so the surface is consistently oriented.
    const bool leftHanded = glm::determinant(vol.geometry.direction) < 0.0;
    auto addQuad = [&](std::int32_t a, std::int32_t b, std::int32_t c, std::int32_t e, bool reverse) {
        if (a < 0 || b < 0 || c < 0 || e < 0)
            return;
        std::uint32_t q[4] = {static_cast<std::uint32_t>(a), static_cast<std::uint32_t>(b), static_cast<std::uint32_t>(c),
                              static_cast<std::uint32_t>(e)};
        if (reverse != leftHanded)
            std::swap(q[1], q[3]);
        // Split along the shorter diagonal.
        const float d02 = glm::length(mesh.positions[q[0]] - mesh.positions[q[2]]);
        const float d13 = glm::length(mesh.positions[q[1]] - mesh.positions[q[3]]);
        if (d02 <= d13)
            mesh.indices.insert(mesh.indices.end(), {q[0], q[1], q[2], q[0], q[2], q[3]});
        else
            mesh.indices.insert(mesh.indices.end(), {q[1], q[2], q[3], q[1], q[3], q[0]});
    };

    loadLayer(v0, 0);
    for (int z = 0; z < n.z - 1; ++z) {
        if ((z & 7) == 0)
            reportProgress(progress, static_cast<float>(z) / static_cast<float>(n.z - 1), std::format("Extracting surface {}%", 100 * z / (n.z - 1)));
        loadLayer(v1, z + 1);
        std::fill(cellCur.begin(), cellCur.end(), -1);

        // 1. Cell vertices for cell layer z.
        for (int y = 0; y < cy; ++y) {
            for (int x = 0; x < cx; ++x) {
                float c[8];
                int mask = 0;
                for (int k = 0; k < 8; ++k) {
                    const auto& o = kCorner[k];
                    const std::vector<float>& layer = o[2] ? v1 : v0;
                    c[k] = layer[static_cast<std::size_t>(y + o[1]) * n.x + (x + o[0])];
                    if (c[k] >= 0.0f)
                        mask |= 1 << k;
                }
                if (mask == 0 || mask == 0xFF)
                    continue;
                glm::dvec3 acc(0.0);
                int count = 0;
                for (const auto& e : kEdge) {
                    const float a = c[e[0]], b = c[e[1]];
                    if ((a >= 0.0f) == (b >= 0.0f))
                        continue;
                    const double t = a / (a - b);
                    const auto& p0 = kCorner[e[0]];
                    const auto& p1 = kCorner[e[1]];
                    acc += glm::dvec3(p0[0] + t * (p1[0] - p0[0]), p0[1] + t * (p1[1] - p0[1]), p0[2] + t * (p1[2] - p0[2]));
                    ++count;
                }
                const glm::dvec3 local = acc / static_cast<double>(count);
                cellCur[static_cast<std::size_t>(y) * cx + x] = static_cast<std::int32_t>(mesh.positions.size());
                mesh.positions.push_back(toWorld(glm::dvec3(x, y, z) + local));
            }
        }

        // 2. Faces. z-edges at grid layer z use cells (x-1..x, y-1..y) in cell layer z.
        auto cellAt = [&](const std::vector<std::int32_t>& layer, int x, int y) -> std::int32_t {
            if (x < 0 || y < 0 || x >= cx || y >= cy)
                return -1;
            return layer[static_cast<std::size_t>(y) * cx + x];
        };
        for (int y = 1; y < cy; ++y) {
            for (int x = 1; x < cx; ++x) {
                const float a = v0[static_cast<std::size_t>(y) * n.x + x];
                const float b = v1[static_cast<std::size_t>(y) * n.x + x];
                if ((a >= 0.0f) == (b >= 0.0f))
                    continue;
                // Outward (inside -> outside) is +z when the lower sample is inside.
                addQuad(cellAt(cellCur, x - 1, y - 1), cellAt(cellCur, x, y - 1), cellAt(cellCur, x, y), cellAt(cellCur, x - 1, y), a < 0.0f);
            }
        }
        // x- and y-edges at grid layer z (z >= 1) use cells in layers z-1 and z.
        if (z >= 1) {
            for (int y = 0; y < n.y; ++y) {
                for (int x = 0; x < n.x; ++x) {
                    const float a = v0[static_cast<std::size_t>(y) * n.x + x];
                    // x-edge (x,y,z)-(x+1,y,z): cells (x, y-1..y, z-1..z)
                    if (x + 1 < n.x && y >= 1 && y < cy + 1) {
                        const float b = v0[static_cast<std::size_t>(y) * n.x + x + 1];
                        if ((a >= 0.0f) != (b >= 0.0f) && x < cx) {
                            addQuad(cellAt(cellPrev, x, y - 1), cellAt(cellPrev, x, y), cellAt(cellCur, x, y), cellAt(cellCur, x, y - 1), a < 0.0f);
                        }
                    }
                    // y-edge (x,y,z)-(x,y+1,z): cells (x-1..x, y, z-1..z)
                    if (y + 1 < n.y && x >= 1) {
                        const float b = v0[static_cast<std::size_t>(y + 1) * n.x + x];
                        if ((a >= 0.0f) != (b >= 0.0f) && y < cy) {
                            // This cell order winds around -y.
                            addQuad(cellAt(cellPrev, x - 1, y), cellAt(cellPrev, x, y), cellAt(cellCur, x, y), cellAt(cellCur, x - 1, y), a >= 0.0f);
                        }
                    }
                }
            }
        }
        std::swap(v0, v1);
        std::swap(cellPrev, cellCur);
    }
    mesh.computeVertexNormals();
    reportProgress(progress, 1.0f, "Surface extracted");
    return mesh;
}

} // namespace occlusa
