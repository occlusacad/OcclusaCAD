#include "core/crown/Margin.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <unordered_map>
#include <unordered_set>

namespace occlusa::crown {

namespace {

std::uint64_t undirectedKey(std::uint32_t a, std::uint32_t b)
{
    if (a > b)
        std::swap(a, b);
    return (static_cast<std::uint64_t>(a) << 32) | b;
}

float percentile(std::vector<float> values, float q)
{
    if (values.empty())
        return 0.0f;
    const auto k = static_cast<std::size_t>(std::clamp(q, 0.0f, 1.0f) * static_cast<float>(values.size() - 1));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(k), values.end());
    return values[k];
}

// Remove sub-loops: whenever a vertex repeats, drop the shorter of the two cycles it closes.
std::vector<std::uint32_t> removeSelfTouches(std::vector<std::uint32_t> seq, bool closed)
{
    for (int guard = 0; guard < 10000; ++guard) {
        std::unordered_map<std::uint32_t, std::size_t> first;
        std::size_t i = 0, j = 0;
        bool found = false;
        for (std::size_t k = 0; k < seq.size(); ++k) {
            auto [it, inserted] = first.emplace(seq[k], k);
            if (!inserted) {
                i = it->second;
                j = k;
                found = true;
                break;
            }
        }
        if (!found)
            break;
        const std::size_t inner = j - i;
        if (!closed || inner <= seq.size() - inner) {
            seq.erase(seq.begin() + static_cast<std::ptrdiff_t>(i + 1), seq.begin() + static_cast<std::ptrdiff_t>(j + 1));
        } else {
            seq = std::vector<std::uint32_t>(seq.begin() + static_cast<std::ptrdiff_t>(i), seq.begin() + static_cast<std::ptrdiff_t>(j));
        }
    }
    return seq;
}

} // namespace

// ---------------------------------------------------------------------------
// PrepScan
// ---------------------------------------------------------------------------

PrepScan::PrepScan(std::shared_ptr<const Mesh> mesh) : mesh_(std::move(mesh)), adjacency_(*mesh_), bvh_(mesh_)
{
    convexity_ = vertexConvexity(*mesh_, adjacency_, 1);
    std::vector<float> positive;
    positive.reserve(convexity_.size() / 2);
    for (float k : convexity_)
        if (k > 0.0f)
            positive.push_back(k);
    convexityScale_ = std::max(percentile(std::move(positive), 0.97f), 0.2f);
    marginCost_.resize(convexity_.size());
    for (std::size_t v = 0; v < convexity_.size(); ++v) {
        const float s = std::clamp(convexity_[v] / convexityScale_, 0.0f, 1.5f);
        marginCost_[v] = 1.0f / (1.0f + 12.0f * s * s);
    }
    vertexTree_.build(mesh_->positions);
}

std::uint32_t PrepScan::nearestVertex(const glm::vec3& p) const
{
    const std::int64_t i = vertexTree_.nearest(p, 1e6f);
    return i < 0 ? 0u : static_cast<std::uint32_t>(i);
}

std::vector<glm::vec3> MarginLine::points(const Mesh& mesh) const
{
    std::vector<glm::vec3> out;
    out.reserve(vertices.size());
    for (std::uint32_t v : vertices)
        out.push_back(mesh.positions[v]);
    return out;
}

// ---------------------------------------------------------------------------
// Tracing
// ---------------------------------------------------------------------------

MarginLine traceMargin(const PrepScan& scan, const std::vector<std::uint32_t>& controls, bool closed)
{
    MarginLine out;
    out.closed = closed && controls.size() >= 3;
    if (controls.size() < 2) {
        out.vertices = controls;
        return out;
    }
    const Mesh& m = scan.mesh();
    const std::size_t n = controls.size();
    const std::size_t segments = out.closed ? n : n - 1;
    std::vector<std::uint32_t> raw;
    for (std::size_t i = 0; i < segments; ++i) {
        const std::uint32_t a = controls[i], b = controls[(i + 1) % n];
        const float straight = glm::length(m.positions[b] - m.positions[a]);
        auto path = shortestPath(m, scan.adjacency(), scan.marginCost(), a, b, 4.0f * straight + 5.0f);
        if (path.empty())
            path = {a, b};
        raw.insert(raw.end(), path.begin(), path.end() - 1);
    }
    if (!out.closed)
        raw.push_back(controls.back());
    out.vertices = removeSelfTouches(std::move(raw), out.closed);
    if (out.closed && out.vertices.size() < 3)
        out.closed = false;
    return out;
}

// ---------------------------------------------------------------------------
// Automatic detection
// ---------------------------------------------------------------------------

std::optional<MarginDetection> detectMargin(const PrepScan& scan, const glm::vec3& click, const MarginDetectOptions& opt, std::string* error)
{
    auto fail = [&](const std::string& msg) -> std::optional<MarginDetection> {
        if (error)
            *error = msg;
        return std::nullopt;
    };
    const Mesh& m = scan.mesh();
    if (m.normals.size() != m.positions.size() || m.empty())
        return fail("The scan has no surface normals.");

    const std::uint32_t seed = scan.nearestVertex(click);
    const glm::vec3 c = m.positions[seed];
    const float reach = (opt.searchRadius + opt.maxDepth) * 1.8f;
    const std::vector<float> geo = geodesicDistances(m, scan.adjacency(), seed, reach);

    // Occlusal direction: mean normal around the clicked point.
    glm::dvec3 up(0.0);
    for (std::size_t v = 0; v < geo.size(); ++v)
        if (geo[v] <= 2.0f)
            up += glm::dvec3(m.normals[v]);
    if (glm::length(up) < 1e-9)
        return fail("Could not determine the preparation direction at the clicked point.");
    up = glm::normalize(up);
    const glm::dvec3 helper = std::abs(up.x) < 0.9 ? glm::dvec3(1, 0, 0) : glm::dvec3(0, 1, 0);
    const glm::dvec3 e1 = glm::normalize(glm::cross(up, helper));
    const glm::dvec3 e2 = glm::cross(up, e1);

    struct Sample {
        float g;
        float conv;
        std::uint32_t v;
    };
    const int nb = std::max(opt.bins, 8);
    std::vector<std::vector<Sample>> bins(static_cast<std::size_t>(nb));
    std::vector<float> positive;
    for (std::size_t v = 0; v < geo.size(); ++v) {
        if (!std::isfinite(geo[v]))
            continue;
        const glm::dvec3 d = glm::dvec3(m.positions[v] - c);
        const double h = glm::dot(d, up);
        if (h < -opt.maxDepth || h > 1.5)
            continue;
        const glm::dvec3 rv = d - h * up;
        if (glm::length(rv) > opt.searchRadius)
            continue;
        const double ang = std::atan2(glm::dot(rv, e2), glm::dot(rv, e1));
        const int b = static_cast<int>(std::floor((ang + std::numbers::pi) / (2.0 * std::numbers::pi) * nb)) % nb;
        const float k = scan.convexity()[v];
        bins[static_cast<std::size_t>(b)].push_back({geo[v], k, static_cast<std::uint32_t>(v)});
        if (k > 0.0f)
            positive.push_back(k);
    }
    const float scale = percentile(std::move(positive), 0.95f);
    if (scale <= 0.0f)
        return fail("No preparation edges were found near the clicked point.");
    const float tau = std::max(0.3f * scale, 0.4f);
    const float tauConcave = std::max(0.2f * scale, 0.3f);

    // Walk outwards (increasing surface distance) in every angular bin.
    struct Found {
        bool valid = false;
        std::uint32_t v = 0;
        double h = 0.0, r = 0.0;
    };
    std::vector<Found> found(static_cast<std::size_t>(nb));
    constexpr float kBucket = 0.25f;
    for (int b = 0; b < nb; ++b) {
        auto& s = bins[static_cast<std::size_t>(b)];
        if (s.empty())
            continue;
        std::sort(s.begin(), s.end(), [](const Sample& x, const Sample& y) { return x.g < y.g; });
        int phase = 0;
        float bestConv = 0.0f;
        std::uint32_t bestV = 0;
        std::size_t i = 0;
        while (i < s.size()) {
            const float start = s[i].g;
            float maxK = -1e9f, minK = 1e9f;
            std::uint32_t maxV = s[i].v;
            for (; i < s.size() && s[i].g < start + kBucket; ++i) {
                if (s[i].conv > maxK) {
                    maxK = s[i].conv;
                    maxV = s[i].v;
                }
                minK = std::min(minK, s[i].conv);
            }
            if (phase == 0) {
                if (minK < -tauConcave)
                    phase = 1; // inner angle of the chamfer / shoulder
            } else if (phase == 1) {
                if (maxK > tau) {
                    phase = 2;
                    bestConv = maxK;
                    bestV = maxV;
                }
            } else {
                if (maxK > bestConv) {
                    bestConv = maxK;
                    bestV = maxV;
                } else if (maxK < 0.7f * tau) {
                    break;
                }
            }
        }
        if (phase == 2) {
            const glm::dvec3 d = glm::dvec3(m.positions[bestV] - c);
            const double h = glm::dot(d, up);
            found[static_cast<std::size_t>(b)] = {true, bestV, h, glm::length(d - h * up)};
        }
    }

    // Reject outliers against angular neighbours (robust median of height and radius).
    std::vector<Found> filtered = found;
    int valid = 0;
    for (int b = 0; b < nb; ++b) {
        if (!found[static_cast<std::size_t>(b)].valid)
            continue;
        std::vector<double> hs, rs;
        for (int o = -4; o <= 4; ++o) {
            const auto& f = found[static_cast<std::size_t>((b + o + nb) % nb)];
            if (f.valid) {
                hs.push_back(f.h);
                rs.push_back(f.r);
            }
        }
        std::nth_element(hs.begin(), hs.begin() + static_cast<std::ptrdiff_t>(hs.size() / 2), hs.end());
        std::nth_element(rs.begin(), rs.begin() + static_cast<std::ptrdiff_t>(rs.size() / 2), rs.end());
        const auto& f = found[static_cast<std::size_t>(b)];
        if (hs.size() < 3 || std::abs(f.h - hs[hs.size() / 2]) > 0.8 || std::abs(f.r - rs[rs.size() / 2]) > 1.2)
            filtered[static_cast<std::size_t>(b)].valid = false;
        else
            ++valid;
    }
    if (valid < nb * 6 / 10)
        return fail("Could not find a clear margin around the preparation. Click on the top of the preparation, or draw the margin manually.");

    MarginDetection det;
    det.bins = nb;
    det.binsFound = valid;
    const int controlCount = std::clamp(opt.controlCount, 4, nb);
    const double step = static_cast<double>(nb) / controlCount;
    for (int k = 0; k < controlCount; ++k) {
        const int target = static_cast<int>(std::lround(k * step)) % nb;
        for (int o = 0; o <= static_cast<int>(step / 2) + 1; ++o) {
            const auto& a = filtered[static_cast<std::size_t>((target + o) % nb)];
            const auto& b = filtered[static_cast<std::size_t>((target - o + nb) % nb)];
            const Found* f = a.valid ? &a : (b.valid ? &b : nullptr);
            if (f) {
                if (det.controls.empty() || det.controls.back() != f->v)
                    det.controls.push_back(f->v);
                break;
            }
        }
    }
    if (det.controls.size() >= 2 && det.controls.front() == det.controls.back())
        det.controls.pop_back();
    if (det.controls.size() < 4)
        return fail("Too few margin points were found. Draw the margin manually.");
    det.line = traceMargin(scan, det.controls, true);
    if (!det.line.closed)
        return fail("The detected margin could not be closed. Draw it manually.");
    glm::dvec3 va = vectorArea(det.line.points(m));
    if (glm::dot(va, up) < 0.0)
        va = -va;
    det.occlusalDirection = glm::length(va) > 1e-9 ? glm::normalize(va) : up;
    return det;
}

// ---------------------------------------------------------------------------
// Die extraction
// ---------------------------------------------------------------------------

std::optional<DieRegion> extractDie(const PrepScan& scan, const MarginLine& margin, const glm::vec3& insidePoint, std::string* error)
{
    auto fail = [&](const std::string& msg) -> std::optional<DieRegion> {
        if (error)
            *error = msg;
        return std::nullopt;
    };
    if (!margin.closed || margin.vertices.size() < 6)
        return fail("The margin line is not closed.");
    const Mesh& m = scan.mesh();
    const std::size_t n = margin.vertices.size();
    const auto pts = margin.points(m);
    const glm::vec3 cen = glm::vec3(centroid(pts));
    float lateral = 0.0f;
    for (const auto& p : pts)
        lateral = std::max(lateral, glm::length(p - cen));
    const float radius = lateral + 14.0f;

    std::unordered_set<std::uint64_t> marginEdges;
    std::unordered_set<std::uint32_t> marginVerts(margin.vertices.begin(), margin.vertices.end());
    for (std::size_t i = 0; i < n; ++i)
        marginEdges.insert(undirectedKey(margin.vertices[i], margin.vertices[(i + 1) % n]));

    const std::size_t triCount = m.triangleCount();
    std::unordered_map<std::uint64_t, std::array<std::uint32_t, 2>> edgeFaces;
    std::vector<std::uint8_t> nearFace(triCount, 0);
    auto vertexNear = [&](std::uint32_t v) { return glm::length(m.positions[v] - cen) < radius; };
    for (std::size_t t = 0; t < triCount; ++t) {
        const std::uint32_t* f = &m.indices[3 * t];
        if (!vertexNear(f[0]) && !vertexNear(f[1]) && !vertexNear(f[2]))
            continue;
        nearFace[t] = 1;
        for (int e = 0; e < 3; ++e) {
            auto [it, inserted] = edgeFaces.try_emplace(undirectedKey(f[e], f[(e + 1) % 3]), std::array<std::uint32_t, 2>{~0u, ~0u});
            if (it->second[0] == ~0u)
                it->second[0] = static_cast<std::uint32_t>(t);
            else
                it->second[1] = static_cast<std::uint32_t>(t);
        }
    }

    const std::uint32_t seedV = scan.nearestVertex(insidePoint);
    if (marginVerts.count(seedV))
        return fail("Pick a point on the preparation, not on the margin line.");
    std::uint32_t seedFace = ~0u;
    for (std::size_t t = 0; t < triCount && seedFace == ~0u; ++t)
        if (nearFace[t])
            for (int k = 0; k < 3; ++k)
                if (m.indices[3 * t + k] == seedV)
                    seedFace = static_cast<std::uint32_t>(t);
    if (seedFace == ~0u)
        return fail("The preparation point is not on the scan.");

    std::vector<std::uint8_t> visited(triCount, 0);
    std::vector<std::uint32_t> stack{seedFace}, faces;
    visited[seedFace] = 1;
    while (!stack.empty()) {
        const std::uint32_t t = stack.back();
        stack.pop_back();
        faces.push_back(t);
        const std::uint32_t* f = &m.indices[3 * t];
        for (int k = 0; k < 3; ++k)
            if (glm::length(m.positions[f[k]] - cen) > radius - 1.0f)
                return fail("The margin line does not separate the preparation from the rest of the scan. Check that it is closed "
                            "and that the clicked point is on the preparation.");
        for (int e = 0; e < 3; ++e) {
            const std::uint64_t key = undirectedKey(f[e], f[(e + 1) % 3]);
            if (marginEdges.count(key))
                continue;
            const auto it = edgeFaces.find(key);
            if (it == edgeFaces.end())
                continue;
            for (std::uint32_t nbF : it->second)
                if (nbF != ~0u && !visited[nbF]) {
                    visited[nbF] = 1;
                    stack.push_back(nbF);
                }
        }
    }

    DieRegion die;
    std::unordered_map<std::uint32_t, std::uint32_t> remap;
    for (std::uint32_t t : faces) {
        for (int k = 0; k < 3; ++k) {
            const std::uint32_t v = m.indices[3 * t + k];
            auto [it, inserted] = remap.try_emplace(v, static_cast<std::uint32_t>(die.scanVertex.size()));
            if (inserted) {
                die.scanVertex.push_back(v);
                die.mesh.positions.push_back(m.positions[v]);
                die.mesh.normals.push_back(m.normals[v]);
            }
            die.mesh.indices.push_back(it->second);
        }
    }
    for (std::uint32_t v : margin.vertices) {
        const auto it = remap.find(v);
        if (it == remap.end())
            return fail("The margin line does not border the preparation everywhere.");
        die.boundary.push_back(it->second);
    }
    if (boundaryEdgeCount(die.mesh) != n)
        return fail("The scan has holes or defects inside the margin line.");
    return die;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

glm::dvec3 vectorArea(const std::vector<glm::vec3>& loop)
{
    glm::dvec3 a(0.0);
    if (loop.size() < 3)
        return a;
    const glm::dvec3 o = centroid(loop);
    for (std::size_t i = 0; i < loop.size(); ++i)
        a += glm::cross(glm::dvec3(loop[i]) - o, glm::dvec3(loop[(i + 1) % loop.size()]) - o);
    return a * 0.5;
}

glm::dvec3 centroid(const std::vector<glm::vec3>& points)
{
    glm::dvec3 c(0.0);
    for (const auto& p : points)
        c += glm::dvec3(p);
    return points.empty() ? c : c / static_cast<double>(points.size());
}

double polylineLength(const std::vector<glm::vec3>& p, bool closed)
{
    double len = 0.0;
    for (std::size_t i = 1; i < p.size(); ++i)
        len += glm::length(p[i] - p[i - 1]);
    if (closed && p.size() > 2)
        len += glm::length(p.front() - p.back());
    return len;
}

std::vector<glm::vec3> smoothPolyline(std::vector<glm::vec3> p, bool closed, int iterations)
{
    const std::size_t n = p.size();
    if (n < 3)
        return p;
    for (int it = 0; it < iterations; ++it) {
        std::vector<glm::vec3> q = p;
        for (std::size_t i = 0; i < n; ++i) {
            if (!closed && (i == 0 || i + 1 == n))
                continue;
            const glm::vec3& a = p[(i + n - 1) % n];
            const glm::vec3& b = p[(i + 1) % n];
            q[i] = p[i] * 0.5f + (a + b) * 0.25f;
        }
        p.swap(q);
    }
    return p;
}

} // namespace occlusa::crown
