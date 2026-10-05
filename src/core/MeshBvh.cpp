#include "core/MeshBvh.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace occlusa {

namespace {

constexpr std::uint32_t kLeafSize = 8;

float boxDistanceSq(const glm::vec3& p, const glm::vec3& bmin, const glm::vec3& bmax)
{
    const glm::vec3 d = glm::max(glm::max(bmin - p, glm::vec3(0.0f)), p - bmax);
    return glm::dot(d, d);
}

bool rayBox(const glm::vec3& o, const glm::vec3& invD, const glm::vec3& bmin, const glm::vec3& bmax, float tMax, float& tEntry)
{
    const glm::vec3 t0 = (bmin - o) * invD;
    const glm::vec3 t1 = (bmax - o) * invD;
    const glm::vec3 lo = glm::min(t0, t1), hi = glm::max(t0, t1);
    const float tNear = std::max(std::max(lo.x, lo.y), std::max(lo.z, 0.0f));
    const float tFar = std::min(std::min(hi.x, hi.y), std::min(hi.z, tMax));
    tEntry = tNear;
    return tNear <= tFar;
}

} // namespace

glm::vec3 closestPointOnTriangle(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, glm::vec3* bary)
{
    // Ericson, Real-Time Collision Detection 5.1.5.
    auto out = [&](float u, float v, float w) {
        if (bary)
            *bary = {u, v, w};
        return a * u + b * v + c * w;
    };
    const glm::vec3 ab = b - a, ac = c - a, ap = p - a;
    const float d1 = glm::dot(ab, ap), d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
        return out(1, 0, 0);
    const glm::vec3 bp = p - b;
    const float d3 = glm::dot(ab, bp), d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3)
        return out(0, 1, 0);
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        const float v = d1 / (d1 - d3);
        return out(1 - v, v, 0);
    }
    const glm::vec3 cp = p - c;
    const float d5 = glm::dot(ab, cp), d6 = glm::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6)
        return out(0, 0, 1);
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        const float w = d2 / (d2 - d6);
        return out(1 - w, 0, w);
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return out(0, 1 - w, w);
    }
    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom, w = vc * denom;
    return out(1 - v - w, v, w);
}

MeshBvh::MeshBvh(std::shared_ptr<const Mesh> mesh) : mesh_(std::move(mesh))
{
    if (!mesh_ || mesh_->empty())
        return;
    const auto n = static_cast<std::uint32_t>(mesh_->triangleCount());
    order_.resize(n);
    std::iota(order_.begin(), order_.end(), 0u);
    std::vector<glm::vec3> centroids(n);
    for (std::uint32_t t = 0; t < n; ++t)
        centroids[t] = (vertex(t, 0) + vertex(t, 1) + vertex(t, 2)) / 3.0f;
    nodes_.reserve(2 * n / kLeafSize + 2);
    build(0, n, centroids);
}

std::uint32_t MeshBvh::build(std::uint32_t begin, std::uint32_t end, std::vector<glm::vec3>& centroids)
{
    const auto index = static_cast<std::uint32_t>(nodes_.size());
    nodes_.push_back({});
    glm::vec3 bmin(1e30f), bmax(-1e30f), cmin(1e30f), cmax(-1e30f);
    for (std::uint32_t i = begin; i < end; ++i) {
        const std::uint32_t t = order_[i];
        for (int k = 0; k < 3; ++k) {
            bmin = glm::min(bmin, vertex(t, k));
            bmax = glm::max(bmax, vertex(t, k));
        }
        cmin = glm::min(cmin, centroids[t]);
        cmax = glm::max(cmax, centroids[t]);
    }
    nodes_[index].bmin = bmin;
    nodes_[index].bmax = bmax;
    if (end - begin <= kLeafSize) {
        nodes_[index].first = begin;
        nodes_[index].count = end - begin;
        return index;
    }
    const glm::vec3 ext = cmax - cmin;
    const int axis = ext.x >= ext.y && ext.x >= ext.z ? 0 : (ext.y >= ext.z ? 1 : 2);
    const std::uint32_t mid = begin + (end - begin) / 2;
    std::nth_element(order_.begin() + begin, order_.begin() + mid, order_.begin() + end,
                     [&](std::uint32_t a, std::uint32_t b) { return centroids[a][axis] < centroids[b][axis]; });
    build(begin, mid, centroids);
    const std::uint32_t right = build(mid, end, centroids);
    nodes_[index].first = right;
    nodes_[index].count = 0;
    return index;
}

std::optional<BvhRayHit> MeshBvh::raycast(const glm::vec3& origin, const glm::vec3& dirIn, float tMax) const
{
    if (empty())
        return std::nullopt;
    const glm::vec3 d = glm::normalize(dirIn);
    const glm::vec3 invD(1.0f / (std::abs(d.x) > 1e-12f ? d.x : 1e-12f), 1.0f / (std::abs(d.y) > 1e-12f ? d.y : 1e-12f),
                         1.0f / (std::abs(d.z) > 1e-12f ? d.z : 1e-12f));
    std::optional<BvhRayHit> best;
    float bestT = tMax;
    std::uint32_t stack[64];
    int sp = 0;
    stack[sp++] = 0;
    while (sp > 0) {
        const Node& node = nodes_[stack[--sp]];
        float tEntry = 0.0f;
        if (!rayBox(origin, invD, node.bmin, node.bmax, bestT, tEntry))
            continue;
        if (node.count == 0) {
            const auto self = static_cast<std::uint32_t>(&node - nodes_.data());
            if (sp + 2 <= 64) {
                stack[sp++] = node.first;
                stack[sp++] = self + 1;
            }
            continue;
        }
        for (std::uint32_t i = node.first; i < node.first + node.count; ++i) {
            const std::uint32_t t = order_[i];
            const glm::vec3 v0 = vertex(t, 0), v1 = vertex(t, 1), v2 = vertex(t, 2);
            const glm::vec3 e1 = v1 - v0, e2 = v2 - v0;
            const glm::vec3 p = glm::cross(d, e2);
            const float det = glm::dot(e1, p);
            if (std::abs(det) < 1e-12f)
                continue;
            const float inv = 1.0f / det;
            const glm::vec3 s = origin - v0;
            const float u = glm::dot(s, p) * inv;
            if (u < 0.0f || u > 1.0f)
                continue;
            const glm::vec3 q = glm::cross(s, e1);
            const float v = glm::dot(d, q) * inv;
            if (v < 0.0f || u + v > 1.0f)
                continue;
            const float tt = glm::dot(e2, q) * inv;
            if (tt > 1e-6f && tt < bestT) {
                bestT = tt;
                BvhRayHit h;
                h.t = tt;
                h.triangle = t;
                h.point = origin + d * tt;
                const glm::vec3 n = glm::cross(e1, e2);
                const float len = glm::length(n);
                h.faceNormal = len > 0.0f ? n / len : glm::vec3(0, 0, 1);
                best = h;
            }
        }
    }
    return best;
}

std::optional<BvhClosestPoint> MeshBvh::closestPoint(const glm::vec3& p, float maxDistance) const
{
    if (empty())
        return std::nullopt;
    float bestSq = maxDistance * maxDistance;
    std::optional<BvhClosestPoint> best;
    glm::vec3 bestBary(0.0f);
    std::uint32_t stack[64];
    int sp = 0;
    stack[sp++] = 0;
    while (sp > 0) {
        const Node& node = nodes_[stack[--sp]];
        if (boxDistanceSq(p, node.bmin, node.bmax) > bestSq)
            continue;
        if (node.count == 0) {
            const auto self = static_cast<std::uint32_t>(&node - nodes_.data());
            const Node& l = nodes_[self + 1];
            const Node& r = nodes_[node.first];
            // Visit the nearer child first (pushed last).
            const bool leftFirst = boxDistanceSq(p, l.bmin, l.bmax) <= boxDistanceSq(p, r.bmin, r.bmax);
            if (sp + 2 <= 64) {
                stack[sp++] = leftFirst ? node.first : self + 1;
                stack[sp++] = leftFirst ? self + 1 : node.first;
            }
            continue;
        }
        for (std::uint32_t i = node.first; i < node.first + node.count; ++i) {
            const std::uint32_t t = order_[i];
            glm::vec3 bary;
            const glm::vec3 q = closestPointOnTriangle(p, vertex(t, 0), vertex(t, 1), vertex(t, 2), &bary);
            const glm::vec3 dv = p - q;
            const float dsq = glm::dot(dv, dv);
            if (dsq <= bestSq) {
                bestSq = dsq;
                BvhClosestPoint c;
                c.triangle = t;
                c.point = q;
                best = c;
                bestBary = bary;
            }
        }
    }
    if (best) {
        const std::uint32_t t = best->triangle;
        glm::vec3 n(0.0f);
        if (mesh_->normals.size() == mesh_->positions.size()) {
            for (int k = 0; k < 3; ++k)
                n += mesh_->normals[mesh_->indices[3 * t + k]] * bestBary[k];
        }
        if (glm::dot(n, n) < 1e-12f)
            n = glm::cross(vertex(t, 1) - vertex(t, 0), vertex(t, 2) - vertex(t, 0));
        const float len = glm::length(n);
        best->normal = len > 0.0f ? n / len : glm::vec3(0, 0, 1);
        best->distance = std::sqrt(bestSq);
        best->signedDistance = glm::dot(p - best->point, best->normal) < 0.0f ? -best->distance : best->distance;
    }
    return best;
}

} // namespace occlusa
