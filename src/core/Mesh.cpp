#include "core/Mesh.h"

#include <cmath>
#include <unordered_map>

namespace occlusa {

Aabb Mesh::bounds() const
{
    Aabb box;
    for (const auto& p : positions)
        box.expand(glm::dvec3(p));
    return box;
}

glm::dvec3 Mesh::centroid() const
{
    glm::dvec3 sum(0.0);
    double totalArea = 0.0;
    for (std::size_t t = 0; t + 2 < indices.size(); t += 3) {
        const glm::dvec3 a(positions[indices[t]]), b(positions[indices[t + 1]]), c(positions[indices[t + 2]]);
        const double area = 0.5 * glm::length(glm::cross(b - a, c - a));
        sum += area * (a + b + c) / 3.0;
        totalArea += area;
    }
    if (totalArea <= 0.0)
        return bounds().center();
    return sum / totalArea;
}

void Mesh::computeVertexNormals()
{
    normals.assign(positions.size(), glm::vec3(0.0f));
    for (std::size_t t = 0; t + 2 < indices.size(); t += 3) {
        const auto i0 = indices[t], i1 = indices[t + 1], i2 = indices[t + 2];
        // Cross product magnitude is twice the area: area weighting for free.
        const glm::vec3 n = glm::cross(positions[i1] - positions[i0], positions[i2] - positions[i0]);
        normals[i0] += n;
        normals[i1] += n;
        normals[i2] += n;
    }
    for (auto& n : normals) {
        const float len = glm::length(n);
        n = len > 0.0f ? n / len : glm::vec3(0.0f, 0.0f, 1.0f);
    }
}

namespace {
struct QuantKey {
    std::int64_t x, y, z;
    bool operator==(const QuantKey&) const = default;
};
struct QuantKeyHash {
    std::size_t operator()(const QuantKey& k) const noexcept
    {
        std::uint64_t h = static_cast<std::uint64_t>(k.x) * 73856093ULL;
        h ^= static_cast<std::uint64_t>(k.y) * 19349663ULL;
        h ^= static_cast<std::uint64_t>(k.z) * 83492791ULL;
        return static_cast<std::size_t>(h ^ (h >> 29));
    }
};
} // namespace

void Mesh::weldVertices(float tolerance)
{
    if (positions.empty())
        return;
    const double inv = 1.0 / std::max(tolerance, 1e-9f);
    std::unordered_map<QuantKey, std::uint32_t, QuantKeyHash> lookup;
    lookup.reserve(positions.size() / 4);
    std::vector<std::uint32_t> remap(positions.size());
    std::vector<glm::vec3> newPositions;
    newPositions.reserve(positions.size() / 4);

    for (std::size_t i = 0; i < positions.size(); ++i) {
        const auto& p = positions[i];
        const QuantKey key{static_cast<std::int64_t>(std::llround(p.x * inv)), static_cast<std::int64_t>(std::llround(p.y * inv)),
                           static_cast<std::int64_t>(std::llround(p.z * inv))};
        auto [it, inserted] = lookup.try_emplace(key, static_cast<std::uint32_t>(newPositions.size()));
        if (inserted)
            newPositions.push_back(p);
        remap[i] = it->second;
    }
    for (auto& idx : indices)
        idx = remap[idx];
    positions = std::move(newPositions);
    normals.clear();
    removeDegenerateTriangles();
}

void Mesh::removeDegenerateTriangles()
{
    std::size_t out = 0;
    for (std::size_t t = 0; t + 2 < indices.size(); t += 3) {
        const auto a = indices[t], b = indices[t + 1], c = indices[t + 2];
        if (a == b || b == c || a == c)
            continue;
        const glm::vec3 n = glm::cross(positions[b] - positions[a], positions[c] - positions[a]);
        if (glm::dot(n, n) <= 0.0f)
            continue;
        indices[out++] = a;
        indices[out++] = b;
        indices[out++] = c;
    }
    indices.resize(out);
}

std::vector<std::uint32_t> subsampleVertices(const Mesh& mesh, std::size_t maxCount)
{
    std::vector<std::uint32_t> out;
    const std::size_t n = mesh.positions.size();
    if (n == 0 || maxCount == 0)
        return out;
    if (n <= maxCount) {
        out.resize(n);
        for (std::size_t i = 0; i < n; ++i)
            out[i] = static_cast<std::uint32_t>(i);
        return out;
    }
    out.reserve(maxCount);
    const double stride = static_cast<double>(n) / static_cast<double>(maxCount);
    for (std::size_t i = 0; i < maxCount; ++i)
        out.push_back(static_cast<std::uint32_t>(static_cast<double>(i) * stride));
    return out;
}

} // namespace occlusa
