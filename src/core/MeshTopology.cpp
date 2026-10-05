#include "core/MeshTopology.h"

#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>

namespace occlusa {

namespace {

std::uint64_t edgeKey(std::uint32_t a, std::uint32_t b)
{
    return (static_cast<std::uint64_t>(a) << 32) | b;
}

} // namespace

MeshAdjacency::MeshAdjacency(const Mesh& mesh)
{
    const std::size_t n = mesh.vertexCount();
    std::vector<std::uint32_t> degree(n, 0);
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3)
        for (int k = 0; k < 3; ++k)
            degree[mesh.indices[t + k]] += 2;
    offsets_.assign(n + 1, 0);
    for (std::size_t v = 0; v < n; ++v)
        offsets_[v + 1] = offsets_[v] + degree[v];
    std::vector<std::uint32_t> raw(offsets_[n]);
    std::vector<std::uint32_t> fill(offsets_.begin(), offsets_.end() - 1);
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        for (int k = 0; k < 3; ++k) {
            const std::uint32_t a = mesh.indices[t + k];
            raw[fill[a]++] = mesh.indices[t + (k + 1) % 3];
            raw[fill[a]++] = mesh.indices[t + (k + 2) % 3];
        }
    }
    // Deduplicate each row.
    neighbors_.reserve(raw.size() / 2);
    std::vector<std::uint32_t> newOffsets(n + 1, 0);
    for (std::size_t v = 0; v < n; ++v) {
        auto b = raw.begin() + offsets_[v], e = raw.begin() + offsets_[v + 1];
        std::sort(b, e);
        e = std::unique(b, e);
        neighbors_.insert(neighbors_.end(), b, e);
        newOffsets[v + 1] = static_cast<std::uint32_t>(neighbors_.size());
    }
    offsets_ = std::move(newOffsets);
}

std::vector<float> vertexConvexity(const Mesh& mesh, const MeshAdjacency& adj, int smoothing)
{
    const std::size_t n = mesh.vertexCount();
    std::vector<float> k(n, 0.0f);
    if (mesh.normals.size() != n)
        return k;
    for (std::size_t v = 0; v < n; ++v) {
        const auto nb = adj.neighbors(static_cast<std::uint32_t>(v));
        if (nb.empty())
            continue;
        const glm::vec3 p = mesh.positions[v], nv = mesh.normals[v];
        float sum = 0.0f;
        for (std::uint32_t j : nb) {
            const glm::vec3 d = mesh.positions[j] - p;
            const float len2 = glm::dot(d, d);
            if (len2 > 1e-12f)
                sum += -2.0f * glm::dot(nv, d) / len2;
        }
        k[v] = sum / static_cast<float>(nb.size());
    }
    for (int it = 0; it < smoothing; ++it) {
        std::vector<float> s(n);
        for (std::size_t v = 0; v < n; ++v) {
            const auto nb = adj.neighbors(static_cast<std::uint32_t>(v));
            float sum = k[v];
            for (std::uint32_t j : nb)
                sum += k[j];
            s[v] = sum / static_cast<float>(nb.size() + 1);
        }
        k.swap(s);
    }
    return k;
}

std::vector<std::uint32_t> shortestPath(const Mesh& mesh, const MeshAdjacency& adj, std::span<const float> cost, std::uint32_t from,
                                        std::uint32_t to, float maxCost)
{
    if (from == to)
        return {from};
    // Sparse bookkeeping so short paths on large scans stay cheap.
    std::unordered_map<std::uint32_t, std::pair<float, std::uint32_t>> best; // vertex -> (distance, predecessor)
    using Item = std::pair<float, std::uint32_t>;
    std::priority_queue<Item, std::vector<Item>, std::greater<>> open;
    best[from] = {0.0f, from};
    open.push({0.0f, from});
    const bool weighted = cost.size() == mesh.vertexCount();
    while (!open.empty()) {
        const auto [d, v] = open.top();
        open.pop();
        if (d > best[v].first)
            continue;
        if (v == to)
            break;
        for (std::uint32_t w : adj.neighbors(v)) {
            float edge = glm::length(mesh.positions[w] - mesh.positions[v]);
            if (weighted)
                edge *= 0.5f * (cost[v] + cost[w]);
            const float nd = d + edge;
            if (nd > maxCost)
                continue;
            auto it = best.find(w);
            if (it == best.end() || nd < it->second.first) {
                best[w] = {nd, v};
                open.push({nd, w});
            }
        }
    }
    if (!best.count(to))
        return {};
    std::vector<std::uint32_t> path;
    for (std::uint32_t v = to;; v = best[v].second) {
        path.push_back(v);
        if (v == from)
            break;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<float> geodesicDistances(const Mesh& mesh, const MeshAdjacency& adj, std::uint32_t source, float maxDistance)
{
    std::vector<float> dist(mesh.vertexCount(), std::numeric_limits<float>::infinity());
    using Item = std::pair<float, std::uint32_t>;
    std::priority_queue<Item, std::vector<Item>, std::greater<>> open;
    dist[source] = 0.0f;
    open.push({0.0f, source});
    while (!open.empty()) {
        const auto [d, v] = open.top();
        open.pop();
        if (d > dist[v])
            continue;
        for (std::uint32_t w : adj.neighbors(v)) {
            const float nd = d + glm::length(mesh.positions[w] - mesh.positions[v]);
            if (nd < dist[w] && nd <= maxDistance) {
                dist[w] = nd;
                open.push({nd, w});
            }
        }
    }
    return dist;
}

bool isClosedManifold(const Mesh& mesh)
{
    std::unordered_map<std::uint64_t, int> directed;
    directed.reserve(mesh.indices.size());
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3)
        for (int e = 0; e < 3; ++e)
            if (++directed[edgeKey(mesh.indices[t + e], mesh.indices[t + (e + 1) % 3])] > 1)
                return false; // same directed edge twice: non-manifold or inconsistent winding
    for (const auto& [key, count] : directed) {
        const auto a = static_cast<std::uint32_t>(key >> 32), b = static_cast<std::uint32_t>(key & 0xffffffffu);
        if (!directed.count(edgeKey(b, a)))
            return false;
    }
    return !mesh.indices.empty();
}

std::size_t boundaryEdgeCount(const Mesh& mesh)
{
    std::unordered_map<std::uint64_t, int> undirected;
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3)
        for (int e = 0; e < 3; ++e) {
            std::uint32_t a = mesh.indices[t + e], b = mesh.indices[t + (e + 1) % 3];
            if (a > b)
                std::swap(a, b);
            ++undirected[edgeKey(a, b)];
        }
    std::size_t count = 0;
    for (const auto& [key, c] : undirected)
        count += c == 1 ? 1 : 0;
    return count;
}

double signedVolume(const Mesh& mesh)
{
    double v = 0.0;
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const glm::dvec3 a(mesh.positions[mesh.indices[t]]), b(mesh.positions[mesh.indices[t + 1]]), c(mesh.positions[mesh.indices[t + 2]]);
        v += glm::dot(a, glm::cross(b, c));
    }
    return v / 6.0;
}

double surfaceArea(const Mesh& mesh)
{
    double area = 0.0;
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const glm::dvec3 a(mesh.positions[mesh.indices[t]]), b(mesh.positions[mesh.indices[t + 1]]), c(mesh.positions[mesh.indices[t + 2]]);
        area += 0.5 * glm::length(glm::cross(b - a, c - a));
    }
    return area;
}

} // namespace occlusa
