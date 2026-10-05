#include "core/Geometry.h"

#include "core/Parallel.h"

#include <algorithm>
#include <cmath>
#include <mutex>

namespace occlusa {

std::optional<MeshHit> raycastMesh(const Mesh& mesh, const glm::dmat4& model, const Ray& worldRay)
{
    if (mesh.empty())
        return std::nullopt;
    const glm::dmat4 inv = glm::inverse(model);
    const glm::dvec3 o = transformPoint(inv, worldRay.origin);
    const glm::dvec3 dRaw = transformVector(inv, worldRay.direction);
    const double dScale = glm::length(dRaw);
    if (dScale <= 0.0)
        return std::nullopt;
    const glm::dvec3 d = dRaw / dScale;

    // Bounding box rejection.
    double t0 = 0, t1 = 0;
    if (!intersectRayAabb(Ray{o, d}, mesh.bounds(), t0, t1))
        return std::nullopt;

    // Moller-Trumbore over all triangles, split across threads.
    const std::size_t triCount = mesh.triangleCount();
    const std::size_t chunks = std::min<std::size_t>(workerCount(), std::max<std::size_t>(1, triCount / 20000));
    struct Best {
        double t = std::numeric_limits<double>::max();
        std::uint32_t tri = 0;
        bool hit = false;
    };
    std::vector<Best> best(chunks);
    parallelFor(chunks, [&](std::size_t c) {
        const std::size_t begin = triCount * c / chunks, end = triCount * (c + 1) / chunks;
        Best b;
        for (std::size_t t = begin; t < end; ++t) {
            const glm::dvec3 v0(mesh.positions[mesh.indices[3 * t]]);
            const glm::dvec3 v1(mesh.positions[mesh.indices[3 * t + 1]]);
            const glm::dvec3 v2(mesh.positions[mesh.indices[3 * t + 2]]);
            const glm::dvec3 e1 = v1 - v0, e2 = v2 - v0;
            const glm::dvec3 p = glm::cross(d, e2);
            const double det = glm::dot(e1, p);
            if (std::abs(det) < 1e-14)
                continue;
            const double invDet = 1.0 / det;
            const glm::dvec3 s = o - v0;
            const double u = glm::dot(s, p) * invDet;
            if (u < 0.0 || u > 1.0)
                continue;
            const glm::dvec3 q = glm::cross(s, e1);
            const double v = glm::dot(d, q) * invDet;
            if (v < 0.0 || u + v > 1.0)
                continue;
            const double tt = glm::dot(e2, q) * invDet;
            if (tt > 1e-9 && tt < b.t) {
                b.t = tt;
                b.tri = static_cast<std::uint32_t>(t);
                b.hit = true;
            }
        }
        best[c] = b;
    });
    Best overall;
    for (const auto& b : best)
        if (b.hit && b.t < overall.t)
            overall = b;
    if (!overall.hit)
        return std::nullopt;

    MeshHit hit;
    hit.triangle = overall.tri;
    const glm::dvec3 localPoint = o + d * overall.t;
    hit.point = transformPoint(model, localPoint);
    hit.distance = glm::length(hit.point - worldRay.origin);
    const glm::dvec3 v0(mesh.positions[mesh.indices[3 * overall.tri]]);
    const glm::dvec3 v1(mesh.positions[mesh.indices[3 * overall.tri + 1]]);
    const glm::dvec3 v2(mesh.positions[mesh.indices[3 * overall.tri + 2]]);
    glm::dvec3 n = glm::normalize(glm::transpose(glm::inverse(glm::dmat3(model))) * glm::cross(v1 - v0, v2 - v0));
    if (glm::dot(n, worldRay.direction) > 0.0)
        n = -n;
    hit.normal = n;
    return hit;
}

std::vector<LineSegment> slicePlane(const Mesh& mesh, const glm::dmat4& model, const Plane& plane)
{
    std::vector<LineSegment> out;
    if (mesh.empty())
        return out;
    // Express the plane in mesh-local coordinates so vertices need not be transformed.
    const glm::dmat4 inv = glm::inverse(model);
    const glm::dvec3 lp = transformPoint(inv, plane.point);
    const glm::dvec3 ln = glm::normalize(glm::transpose(glm::dmat3(model)) * plane.normal);

    std::vector<float> dist(mesh.positions.size());
    for (std::size_t i = 0; i < mesh.positions.size(); ++i)
        dist[i] = static_cast<float>(glm::dot(glm::dvec3(mesh.positions[i]) - lp, ln));

    const glm::mat4 m(model);
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const std::uint32_t idx[3] = {mesh.indices[t], mesh.indices[t + 1], mesh.indices[t + 2]};
        const float d[3] = {dist[idx[0]], dist[idx[1]], dist[idx[2]]};
        if ((d[0] > 0 && d[1] > 0 && d[2] > 0) || (d[0] < 0 && d[1] < 0 && d[2] < 0))
            continue;
        glm::vec3 pts[2];
        int count = 0;
        for (int e = 0; e < 3 && count < 2; ++e) {
            const int a = e, b = (e + 1) % 3;
            const bool sa = d[a] >= 0, sb = d[b] >= 0;
            if (sa == sb)
                continue;
            const float s = d[a] / (d[a] - d[b]);
            pts[count++] = mesh.positions[idx[a]] + s * (mesh.positions[idx[b]] - mesh.positions[idx[a]]);
        }
        if (count == 2)
            out.push_back(LineSegment{glm::vec3(m * glm::vec4(pts[0], 1.0f)), glm::vec3(m * glm::vec4(pts[1], 1.0f))});
    }
    return out;
}

Mesh makeTube(const std::vector<glm::vec3>& points, bool closed, float radius, int sides)
{
    Mesh m;
    const std::size_t n = points.size();
    if (n < 2)
        return m;
    sides = std::max(sides, 3);
    glm::vec3 prevNormal(0.0f);
    for (std::size_t i = 0; i < n; ++i) {
        const glm::vec3 a = points[i == 0 ? (closed ? n - 1 : 0) : i - 1];
        const glm::vec3 b = points[i + 1 == n ? (closed ? 0 : n - 1) : i + 1];
        glm::vec3 t = b - a;
        if (glm::dot(t, t) < 1e-12f)
            t = glm::vec3(1, 0, 0);
        t = glm::normalize(t);
        // Parallel transport of the frame keeps the tube from twisting.
        glm::vec3 nrm = prevNormal - t * glm::dot(prevNormal, t);
        if (glm::dot(nrm, nrm) < 1e-8f) {
            nrm = glm::cross(t, std::abs(t.x) < 0.9f ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0));
        }
        nrm = glm::normalize(nrm);
        prevNormal = nrm;
        const glm::vec3 bin = glm::cross(t, nrm);
        for (int k = 0; k < sides; ++k) {
            const float ang = 6.2831853f * static_cast<float>(k) / static_cast<float>(sides);
            const glm::vec3 dir = nrm * std::cos(ang) + bin * std::sin(ang);
            m.positions.push_back(points[i] + dir * radius);
            m.normals.push_back(dir);
        }
    }
    const std::size_t segs = closed ? n : n - 1;
    for (std::size_t i = 0; i < segs; ++i) {
        const auto r0 = static_cast<std::uint32_t>(i * sides), r1 = static_cast<std::uint32_t>(((i + 1) % n) * sides);
        for (int k = 0; k < sides; ++k) {
            const auto k1 = static_cast<std::uint32_t>((k + 1) % sides);
            const auto kk = static_cast<std::uint32_t>(k);
            m.indices.insert(m.indices.end(), {r0 + kk, r1 + kk, r1 + k1, r0 + kk, r1 + k1, r0 + k1});
        }
    }
    return m;
}

Mesh makeSphere(const glm::vec3& center, float radius, int segments)
{
    Mesh m;
    const int rings = std::max(segments / 2, 3), sectors = std::max(segments, 4);
    for (int r = 0; r <= rings; ++r) {
        const float phi = 3.14159265f * static_cast<float>(r) / static_cast<float>(rings);
        for (int s = 0; s <= sectors; ++s) {
            const float th = 6.2831853f * static_cast<float>(s) / static_cast<float>(sectors);
            const glm::vec3 d(std::sin(phi) * std::cos(th), std::sin(phi) * std::sin(th), std::cos(phi));
            m.positions.push_back(center + d * radius);
            m.normals.push_back(d);
        }
    }
    for (int r = 0; r < rings; ++r)
        for (int s = 0; s < sectors; ++s) {
            const auto a = static_cast<std::uint32_t>(r * (sectors + 1) + s), b = a + static_cast<std::uint32_t>(sectors + 1);
            m.indices.insert(m.indices.end(), {a, b, a + 1, a + 1, b, b + 1});
        }
    return m;
}

} // namespace occlusa
