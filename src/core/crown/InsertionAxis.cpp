#include "core/crown/InsertionAxis.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <numeric>

namespace occlusa::crown {

namespace {

constexpr int kBins = 180;          // 2 degree sectors
constexpr double kHeightTol = 0.05; // ignore "overhangs" closer than this along the axis (scan noise, flat shoulders)

double faceAreaWeightedCost(const Mesh& die, const std::vector<glm::dvec3>& faceN, const std::vector<double>& faceA, const glm::dvec3& a)
{
    double c = 0.0;
    for (std::size_t f = 0; f < faceN.size(); ++f) {
        const double d = glm::dot(faceN[f], a);
        if (d < 0.0)
            c += faceA[f] * d * d;
    }
    (void)die;
    return c;
}

} // namespace

void perpendicularBasis(const glm::dvec3& axis, glm::dvec3& u, glm::dvec3& v)
{
    const glm::dvec3 helper = std::abs(axis.x) < 0.9 ? glm::dvec3(1, 0, 0) : glm::dvec3(0, 1, 0);
    u = glm::normalize(glm::cross(axis, helper));
    v = glm::cross(axis, u);
}

Blockout computeBlockout(const Mesh& die, const glm::dvec3& axisIn, const glm::dvec3& center)
{
    const glm::dvec3 axis = glm::normalize(axisIn);
    glm::dvec3 u, v;
    perpendicularBasis(axis, u, v);
    const std::size_t n = die.vertexCount();
    struct Cyl {
        double h, r;
        int bin;
        glm::dvec3 radial;
    };
    std::vector<Cyl> cyl(n);
    for (std::size_t i = 0; i < n; ++i) {
        const glm::dvec3 d = glm::dvec3(die.positions[i]) - center;
        const double h = glm::dot(d, axis);
        const glm::dvec3 rv = d - h * axis;
        const double r = glm::length(rv);
        const double ang = std::atan2(glm::dot(rv, v), glm::dot(rv, u));
        const int b = static_cast<int>(std::floor((ang + std::numbers::pi) / (2.0 * std::numbers::pi) * kBins)) % kBins;
        cyl[i] = {h, r, b, r > 1e-9 ? rv / r : glm::dvec3(0.0)};
    }
    std::vector<std::size_t> order(n);
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return cyl[a].h > cyl[b].h; });

    Blockout out;
    out.positions = die.positions;
    out.depth.assign(n, 0.0f);
    std::vector<double> runMax(kBins, 0.0);
    std::size_t added = 0; // vertices (in `order`) already folded into runMax
    for (std::size_t k = 0; k < n; ++k) {
        const std::size_t i = order[k];
        while (added < k && cyl[order[added]].h > cyl[i].h + kHeightTol) {
            const auto& c = cyl[order[added]];
            runMax[static_cast<std::size_t>(c.bin)] = std::max(runMax[static_cast<std::size_t>(c.bin)], c.r);
            ++added;
        }
        const auto& c = cyl[i];
        if (c.r < 0.3)
            continue;
        const int b = c.bin;
        const double above = std::max({runMax[static_cast<std::size_t>(b)], runMax[static_cast<std::size_t>((b + 1) % kBins)],
                                       runMax[static_cast<std::size_t>((b + kBins - 1) % kBins)]});
        if (above > c.r) {
            out.depth[i] = static_cast<float>(above - c.r);
            out.positions[i] = glm::vec3(glm::dvec3(die.positions[i]) + c.radial * (above - c.r));
        }
    }
    return out;
}

UndercutReport undercutReport(const Mesh& die, const Blockout& blockout)
{
    UndercutReport rep;
    for (std::size_t t = 0; t + 2 < die.indices.size(); t += 3) {
        const glm::dvec3 a(die.positions[die.indices[t]]), b(die.positions[die.indices[t + 1]]), c(die.positions[die.indices[t + 2]]);
        const double area = 0.5 * glm::length(glm::cross(b - a, c - a));
        rep.dieArea += area;
        const double d = (blockout.depth[die.indices[t]] + blockout.depth[die.indices[t + 1]] + blockout.depth[die.indices[t + 2]]) / 3.0;
        if (d > 0.01)
            rep.undercutArea += area;
    }
    for (float d : blockout.depth)
        rep.maxDepth = std::max(rep.maxDepth, static_cast<double>(d));
    return rep;
}

glm::dvec3 tiltAxis(const glm::dvec3& axis, const glm::dvec3& u, const glm::dvec3& v, double tiltUDeg, double tiltVDeg)
{
    glm::dmat4 r = glm::rotate(glm::dmat4(1.0), glm::radians(tiltUDeg), u);
    r = glm::rotate(r, glm::radians(tiltVDeg), v);
    return glm::normalize(glm::dvec3(r * glm::dvec4(axis, 0.0)));
}

glm::dvec3 optimizeInsertionAxis(const Mesh& die, const glm::dvec3& initialIn, double maxTiltDeg)
{
    const glm::dvec3 initial = glm::normalize(initialIn);
    std::vector<glm::dvec3> faceN;
    std::vector<double> faceA;
    double totalArea = 0.0;
    for (std::size_t t = 0; t + 2 < die.indices.size(); t += 3) {
        const glm::dvec3 a(die.positions[die.indices[t]]), b(die.positions[die.indices[t + 1]]), c(die.positions[die.indices[t + 2]]);
        const glm::dvec3 cr = glm::cross(b - a, c - a);
        const double len = glm::length(cr);
        if (len < 1e-12)
            continue;
        faceN.push_back(cr / len);
        faceA.push_back(0.5 * len);
        totalArea += 0.5 * len;
    }
    glm::dvec3 u, v;
    perpendicularBasis(initial, u, v);
    // Small preference for the initial direction so flat cost landscapes keep it.
    const double regularization = totalArea * 2e-6;
    auto cost = [&](const glm::dvec3& a) {
        const double tilt = glm::degrees(std::acos(std::clamp(glm::dot(a, initial), -1.0, 1.0)));
        return faceAreaWeightedCost(die, faceN, faceA, a) + regularization * tilt * tilt;
    };
    glm::dvec3 best = initial;
    double bestCost = cost(initial);
    for (double tilt = 2.0; tilt <= maxTiltDeg + 1e-9; tilt += 2.0) {
        for (double az = 0.0; az < 360.0; az += 10.0) {
            const double t = glm::radians(tilt), z = glm::radians(az);
            const glm::dvec3 a = glm::normalize(initial * std::cos(t) + (u * std::cos(z) + v * std::sin(z)) * std::sin(t));
            const double c = cost(a);
            if (c < bestCost) {
                bestCost = c;
                best = a;
            }
        }
    }
    // Local refinement.
    for (double step = 1.0; step >= 0.124; step *= 0.5) {
        bool improved = true;
        while (improved) {
            improved = false;
            glm::dvec3 bu, bv;
            perpendicularBasis(best, bu, bv);
            for (const glm::dvec3& dir : {bu, -bu, bv, -bv}) {
                const glm::dvec3 a = glm::normalize(best * std::cos(glm::radians(step)) + dir * std::sin(glm::radians(step)));
                if (glm::degrees(std::acos(std::clamp(glm::dot(a, initial), -1.0, 1.0))) > maxTiltDeg)
                    continue;
                const double c = cost(a);
                if (c < bestCost - 1e-15) {
                    bestCost = c;
                    best = a;
                    improved = true;
                }
            }
        }
    }
    return best;
}

} // namespace occlusa::crown
