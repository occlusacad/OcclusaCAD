#include "core/Volume.h"

#include <cmath>

namespace occlusa {

glm::dmat4 VolumeGeometry::voxelToWorld() const
{
    glm::dmat4 m(1.0);
    for (int c = 0; c < 3; ++c)
        m[c] = glm::dvec4(direction[c] * spacing[c], 0.0);
    m[3] = glm::dvec4(origin, 1.0);
    return m;
}

glm::dmat4 VolumeGeometry::worldToVoxel() const
{
    return glm::inverse(voxelToWorld());
}

glm::dmat4 VolumeGeometry::worldToTexture() const
{
    // tex = (voxel + 0.5) / dims
    const glm::dvec3 d(dims);
    glm::dmat4 voxelToTex = glm::scale(glm::dmat4(1.0), 1.0 / d) * glm::translate(glm::dmat4(1.0), glm::dvec3(0.5));
    return voxelToTex * worldToVoxel();
}

Aabb VolumeGeometry::worldBounds() const
{
    Aabb voxelBox;
    voxelBox.min = glm::dvec3(-0.5);
    voxelBox.max = glm::dvec3(dims) - 0.5;
    return voxelBox.transformed(voxelToWorld());
}

glm::dvec3 VolumeGeometry::worldCenter() const
{
    return transformPoint(voxelToWorld(), (glm::dvec3(dims) - 1.0) * 0.5);
}

double Volume::sampleVoxel(const glm::dvec3& v) const
{
    const auto& d = geometry.dims;
    if (voxels.empty())
        return 0.0;
    const double x = std::clamp(v.x, 0.0, static_cast<double>(d.x - 1));
    const double y = std::clamp(v.y, 0.0, static_cast<double>(d.y - 1));
    const double z = std::clamp(v.z, 0.0, static_cast<double>(d.z - 1));
    const int x0 = static_cast<int>(x), y0 = static_cast<int>(y), z0 = static_cast<int>(z);
    const int x1 = std::min(x0 + 1, d.x - 1), y1 = std::min(y0 + 1, d.y - 1), z1 = std::min(z0 + 1, d.z - 1);
    const double fx = x - x0, fy = y - y0, fz = z - z0;

    auto s = [&](int i, int j, int k) { return static_cast<double>(voxels[index(i, j, k)]); };
    const double c00 = s(x0, y0, z0) * (1 - fx) + s(x1, y0, z0) * fx;
    const double c10 = s(x0, y1, z0) * (1 - fx) + s(x1, y1, z0) * fx;
    const double c01 = s(x0, y0, z1) * (1 - fx) + s(x1, y0, z1) * fx;
    const double c11 = s(x0, y1, z1) * (1 - fx) + s(x1, y1, z1) * fx;
    const double c0 = c00 * (1 - fy) + c10 * fy;
    const double c1 = c01 * (1 - fy) + c11 * fy;
    return (c0 * (1 - fz) + c1 * fz) * rescaleSlope + rescaleIntercept;
}

double Volume::sampleWorld(const glm::dvec3& w) const
{
    return sampleVoxel(transformPoint(geometry.worldToVoxel(), w));
}

bool Volume::containsWorld(const glm::dvec3& w) const
{
    const glm::dvec3 v = transformPoint(geometry.worldToVoxel(), w);
    const auto& d = geometry.dims;
    return v.x >= -0.5 && v.y >= -0.5 && v.z >= -0.5 && v.x <= d.x - 0.5 && v.y <= d.y - 0.5 && v.z <= d.z - 0.5;
}

std::pair<double, double> Volume::valueRange() const
{
    if (cachedRange_)
        return *cachedRange_;
    if (voxels.empty())
        return {0.0, 0.0};
    const auto [mn, mx] = std::minmax_element(voxels.begin(), voxels.end());
    double a = *mn * rescaleSlope + rescaleIntercept;
    double b = *mx * rescaleSlope + rescaleIntercept;
    if (a > b)
        std::swap(a, b);
    cachedRange_ = std::make_pair(a, b);
    return *cachedRange_;
}

std::vector<std::uint64_t> Volume::histogram(int bins) const
{
    std::vector<std::uint64_t> h(static_cast<std::size_t>(std::max(bins, 1)), 0);
    const auto [lo, hi] = valueRange();
    if (hi <= lo)
        return h;
    const double scale = bins / (hi - lo);
    // Subsample large volumes; the histogram is used for display heuristics only.
    const std::size_t step = std::max<std::size_t>(1, voxels.size() / 4'000'000);
    for (std::size_t i = 0; i < voxels.size(); i += step) {
        const double v = voxels[i] * rescaleSlope + rescaleIntercept;
        const int b = std::clamp(static_cast<int>((v - lo) * scale), 0, bins - 1);
        ++h[static_cast<std::size_t>(b)];
    }
    return h;
}

double Volume::suggestBoneThreshold() const
{
    const int bins = 512;
    const auto h = histogram(bins);
    const auto [lo, hi] = valueRange();
    if (hi <= lo)
        return lo;
    const double binWidth = (hi - lo) / bins;

    // Otsu on the upper part of the histogram (exclude air/background), which separates
    // soft tissue from hard tissue on both calibrated CT and uncalibrated CBCT.
    auto otsu = [&](int first, int last) {
        double total = 0, sum = 0;
        for (int i = first; i < last; ++i) {
            total += static_cast<double>(h[i]);
            sum += i * static_cast<double>(h[i]);
        }
        double wB = 0, sumB = 0, best = -1;
        int bestIdx = first;
        for (int i = first; i < last; ++i) {
            wB += static_cast<double>(h[i]);
            if (wB == 0)
                continue;
            const double wF = total - wB;
            if (wF == 0)
                break;
            sumB += i * static_cast<double>(h[i]);
            const double mB = sumB / wB, mF = (sum - sumB) / wF;
            const double between = wB * wF * (mB - mF) * (mB - mF);
            if (between > best) {
                best = between;
                bestIdx = i;
            }
        }
        return bestIdx;
    };
    const int airSplit = otsu(0, bins);
    const int tissueSplit = otsu(airSplit + 1, bins);
    double threshold = lo + (tissueSplit + 0.5) * binWidth;

    // Calibrated CT: hard tissue is reliably above ~300 HU, clamp into a sane band.
    if (lo < -900 && hi > 1000)
        threshold = std::clamp(threshold, 250.0, 1200.0);
    return threshold;
}

std::pair<double, double> Volume::suggestWindow() const
{
    if (info.windowCenter && info.windowWidth && *info.windowWidth > 1.0)
        return {*info.windowCenter, *info.windowWidth};
    const int bins = 1024;
    const auto h = histogram(bins);
    const auto [lo, hi] = valueRange();
    std::uint64_t total = 0;
    for (auto c : h)
        total += c;
    if (total == 0 || hi <= lo)
        return {(lo + hi) * 0.5, std::max(hi - lo, 1.0)};
    auto percentile = [&](double p) {
        const auto target = static_cast<std::uint64_t>(p * static_cast<double>(total));
        std::uint64_t acc = 0;
        for (int i = 0; i < bins; ++i) {
            acc += h[i];
            if (acc >= target)
                return lo + (i + 0.5) * (hi - lo) / bins;
        }
        return hi;
    };
    const double a = percentile(0.01), b = percentile(0.995);
    return {(a + b) * 0.5, std::max(b - a, 1.0)};
}

std::optional<glm::dvec3> Volume::raycastIso(const Ray& worldRay, double iso, double maxDistance) const
{
    if (voxels.empty())
        return std::nullopt;
    double tNear = 0, tFar = 0;
    if (!intersectRayAabb(worldRay, geometry.worldBounds(), tNear, tFar))
        return std::nullopt;
    tNear = std::max(tNear, 0.0);
    tFar = std::min(tFar, maxDistance);
    const double minSpacing = std::min({geometry.spacing.x, geometry.spacing.y, geometry.spacing.z});
    const double step = std::max(minSpacing * 0.5, 1e-3);
    const glm::dmat4 w2v = geometry.worldToVoxel();

    double prevT = tNear;
    double prevV = sampleVoxel(transformPoint(w2v, worldRay.at(tNear)));
    if (prevV >= iso)
        return worldRay.at(tNear);
    for (double t = tNear + step; t <= tFar; t += step) {
        const double v = sampleVoxel(transformPoint(w2v, worldRay.at(t)));
        if (v >= iso) {
            // Bisection refinement between prevT and t.
            double a = prevT, b = t;
            for (int it = 0; it < 12; ++it) {
                const double m = 0.5 * (a + b);
                if (sampleVoxel(transformPoint(w2v, worldRay.at(m))) >= iso)
                    b = m;
                else
                    a = m;
            }
            return worldRay.at(b);
        }
        prevT = t;
        prevV = v;
    }
    (void)prevV;
    return std::nullopt;
}

} // namespace occlusa
