#include "core/implant/ScanBodyFit.h"

#include "core/Registration.h"

#include <cmath>
#include <numbers>

namespace occlusa::implant {

namespace {

constexpr double kVisibleDepth = 6.0;  // only the top of the scan body is above the gingiva
constexpr double kRegionRadius = 13.0; // scan region around the click used as the ICP target

// Scan vertices (with normals) within `radius` of `center`.
Mesh scanRegion(const Mesh& scan, const glm::dvec3& center, double radius)
{
    Mesh r;
    const auto c = glm::vec3(center);
    const float r2 = static_cast<float>(radius * radius);
    for (std::size_t i = 0; i < scan.positions.size(); ++i) {
        const glm::vec3 d = scan.positions[i] - c;
        if (glm::dot(d, d) <= r2) {
            r.positions.push_back(scan.positions[i]);
            r.normals.push_back(i < scan.normals.size() ? scan.normals[i] : glm::vec3(0.0f));
        }
    }
    return r;
}

// Visible part of the scan body samples (the top kVisibleDepth mm).
Mesh visibleSamples(const Mesh& scanBody)
{
    const Mesh all = sampleSurface(scanBody, 0.15);
    float top = -1e30f;
    for (const auto& p : all.positions)
        top = std::max(top, p.z);
    Mesh v;
    for (std::size_t i = 0; i < all.positions.size(); ++i)
        if (all.positions[i].z >= top - static_cast<float>(kVisibleDepth)) {
            v.positions.push_back(all.positions[i]);
            v.normals.push_back(all.normals[i]);
        }
    return v;
}

glm::dmat4 frameFromAxis(const glm::dvec3& axis, double spin, const glm::dvec3& topCenterScan, double topZ)
{
    const glm::dvec3 z = glm::normalize(axis);
    const glm::dvec3 ref = std::abs(z.x) < 0.9 ? glm::dvec3(1, 0, 0) : glm::dvec3(0, 1, 0);
    glm::dvec3 x = glm::normalize(glm::cross(ref, z));
    glm::dvec3 y = glm::cross(z, x);
    const glm::dvec3 xs = x * std::cos(spin) + y * std::sin(spin);
    const glm::dvec3 ys = glm::cross(z, xs);
    glm::dmat4 m(1.0);
    m[0] = glm::dvec4(xs, 0.0);
    m[1] = glm::dvec4(ys, 0.0);
    m[2] = glm::dvec4(z, 0.0);
    m[3] = glm::dvec4(topCenterScan - z * topZ, 1.0);
    return m;
}

// Fit quality: point-to-plane distances of the scan body samples to the scan (the scan is a
// point set about 0.1-0.2 mm apart, so point-to-point distances would mostly measure its spacing).
void measureFit(const Mesh& samples, const IcpTarget& target, ScanBodyFitResult& r)
{
    std::size_t near = 0, within = 0, total = 0;
    double sum2 = 0.0, sum = 0.0;
    const std::size_t stride = std::max<std::size_t>(1, samples.positions.size() / 8000);
    for (std::size_t i = 0; i < samples.positions.size(); i += stride) {
        ++total;
        const glm::vec3 p(transformPoint(r.implantToScan, glm::dvec3(samples.positions[i])));
        const std::int64_t j = target.tree.nearest(p, 0.3f);
        if (j < 0)
            continue;
        const auto ji = static_cast<std::size_t>(j);
        const double d = std::abs(glm::dot(glm::dvec3(p - target.tree.points()[ji]), glm::dvec3(target.normals[ji])));
        ++near;
        sum += d;
        sum2 += d * d;
        if (d < 0.05)
            ++within;
    }
    r.inlierFraction = total ? static_cast<double>(near) / static_cast<double>(total) : 0.0;
    r.rms = near ? std::sqrt(sum2 / static_cast<double>(near)) : 1e9;
    r.meanDistance = near ? sum / static_cast<double>(near) : 1e9;
    r.fractionWithin = near ? static_cast<double>(within) / static_cast<double>(near) : 0.0;
}

ScanBodyFitResult evaluate(const Mesh& samples, const IcpTarget& target, const glm::dmat4& initial)
{
    IcpOptions opt;
    opt.maxIterations = 50;
    opt.maxCorrespondenceDistance = 1.2;
    opt.finalCorrespondenceDistance = 0.15;
    opt.trimFraction = 0.9;
    opt.maxSourcePoints = 6000;
    const IcpResult icp = refineIcp(samples, initial, target, opt);
    ScanBodyFitResult r;
    r.implantToScan = icp.transform;
    measureFit(samples, target, r);
    r.ok = icp.iterations > 0;
    return r;
}

} // namespace

Mesh sampleSurface(const Mesh& mesh, double spacing)
{
    Mesh out;
    const double cell = spacing * spacing;
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const glm::dvec3 a(mesh.positions[mesh.indices[t]]), b(mesh.positions[mesh.indices[t + 1]]), c(mesh.positions[mesh.indices[t + 2]]);
        const glm::dvec3 cr = glm::cross(b - a, c - a);
        const double area = glm::length(cr) * 0.5;
        if (area <= 1e-12)
            continue;
        const glm::vec3 n(cr / (2.0 * area));
        // Regular barycentric grid with about one sample per spacing^2.
        const int k = std::max(1, static_cast<int>(std::ceil(std::sqrt(2.0 * area / cell))));
        for (int i = 0; i < k; ++i)
            for (int j = 0; j < k - i; ++j) {
                const double u = (i + 1.0 / 3.0) / k, v = (j + 1.0 / 3.0) / k;
                out.positions.emplace_back(a + (b - a) * u + (c - a) * v);
                out.normals.push_back(n);
            }
    }
    return out;
}

std::optional<ScanBodyFitResult> fitScanBody(const Mesh& scan, const Mesh& scanBody, const glm::dvec3& click, const ProgressFn& progress)
{
    const Mesh region = scanRegion(scan, click, kRegionRadius);
    if (region.positions.size() < 50)
        return std::nullopt;
    const IcpTarget target = IcpTarget::fromMesh(region);
    const Mesh samples = visibleSamples(scanBody);
    double topZ = -1e30;
    for (const auto& p : scanBody.positions)
        topZ = std::max(topZ, static_cast<double>(p.z));

    // Axis guess: mean normal of the scan around the click (the flat top of the scan body).
    glm::dvec3 axis(0.0);
    for (std::size_t i = 0; i < region.positions.size(); ++i)
        if (glm::length(glm::dvec3(region.positions[i]) - click) < 1.0)
            axis += glm::dvec3(region.normals[i]);
    if (glm::length(axis) < 1e-6)
        return std::nullopt;
    axis = glm::normalize(axis);

    std::optional<ScanBodyFitResult> best;
    constexpr int kSpins = 12;
    for (int s = 0; s < kSpins; ++s) {
        reportProgress(progress, static_cast<float>(s) / kSpins, "Matching the scan body");
        const glm::dmat4 init = frameFromAxis(axis, 2.0 * std::numbers::pi * s / kSpins, click, topZ);
        ScanBodyFitResult r = evaluate(samples, target, init);
        auto score = [](const ScanBodyFitResult& x) { return x.inlierFraction * x.fractionWithin - x.rms; };
        if (r.ok && (!best || score(r) > score(*best)))
            best = r;
    }
    return best;
}

ScanBodyFitResult refineScanBody(const Mesh& scan, const Mesh& scanBody, const glm::dmat4& implantToScan)
{
    double topZ = -1e30;
    for (const auto& p : scanBody.positions)
        topZ = std::max(topZ, static_cast<double>(p.z));
    const glm::dvec3 top = transformPoint(implantToScan, glm::dvec3(0.0, 0.0, topZ));
    const Mesh region = scanRegion(scan, top, kRegionRadius);
    if (region.positions.size() < 50)
        return {};
    return evaluate(visibleSamples(scanBody), IcpTarget::fromMesh(region), implantToScan);
}

} // namespace occlusa::implant
