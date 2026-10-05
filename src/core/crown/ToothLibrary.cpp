#include "core/crown/ToothLibrary.h"

#include <algorithm>
#include <cmath>

namespace occlusa::crown {

ToothKind toothKindFromFdi(int fdi)
{
    switch (fdi % 10) {
    case 1: return ToothKind::CentralIncisor;
    case 2: return ToothKind::LateralIncisor;
    case 3: return ToothKind::Canine;
    case 4: return ToothKind::FirstPremolar;
    case 5: return ToothKind::SecondPremolar;
    case 6: return ToothKind::FirstMolar;
    default: return ToothKind::SecondMolar; // 7 and 8
    }
}

std::string_view toString(ToothKind kind)
{
    switch (kind) {
    case ToothKind::CentralIncisor: return "Central incisor";
    case ToothKind::LateralIncisor: return "Lateral incisor";
    case ToothKind::Canine: return "Canine";
    case ToothKind::FirstPremolar: return "First premolar";
    case ToothKind::SecondPremolar: return "Second premolar";
    case ToothKind::FirstMolar: return "First molar";
    case ToothKind::SecondMolar: return "Second molar";
    }
    return "";
}

const ToothTemplate& toothTemplate(ToothKind kind, bool upper)
{
    // Average permanent tooth dimensions (mm); cusp layouts simplified from standard dental anatomy.
    static const ToothTemplate lower[] = {
        {ToothKind::CentralIncisor, "Lower central incisor", 5.3, 5.9, 9.0, 0.30, 2.2, 0.95, 0.18, 0.4, 0.0,
         {{0.0, 0.1, 0.9, 0.9}}},
        {ToothKind::LateralIncisor, "Lower lateral incisor", 5.9, 6.2, 9.4, 0.30, 2.2, 0.95, 0.18, 0.4, 0.0,
         {{0.0, 0.1, 0.9, 0.9}}},
        {ToothKind::Canine, "Lower canine", 6.9, 7.7, 10.5, 0.33, 2.2, 0.80, 0.30, 0.5, 0.0,
         {{0.1, 0.2, 2.2, 0.38}}},
        {ToothKind::FirstPremolar, "Lower first premolar", 7.0, 7.7, 8.0, 0.35, 2.3, 0.75, 0.62, 0.9, 0.3,
         {{0.0, 0.48, 2.0, 0.40}, {0.0, -0.55, 0.9, 0.35}}},
        {ToothKind::SecondPremolar, "Lower second premolar", 7.1, 8.0, 7.6, 0.35, 2.4, 0.78, 0.66, 0.9, 0.35,
         {{0.0, 0.48, 1.8, 0.40}, {0.35, -0.50, 1.3, 0.32}, {-0.35, -0.50, 1.1, 0.30}}},
        {ToothKind::FirstMolar, "Lower first molar", 11.0, 10.3, 7.4, 0.35, 2.7, 0.80, 0.68, 1.4, 0.45,
         {{0.48, 0.52, 1.6, 0.30}, {-0.08, 0.58, 1.5, 0.30}, {-0.62, 0.38, 1.2, 0.26},
          {0.45, -0.52, 1.9, 0.30}, {-0.35, -0.52, 1.8, 0.30}}},
        {ToothKind::SecondMolar, "Lower second molar", 10.5, 10.0, 7.0, 0.35, 2.8, 0.80, 0.68, 1.3, 0.45,
         {{0.45, 0.52, 1.6, 0.32}, {-0.40, 0.52, 1.5, 0.32}, {0.45, -0.52, 1.8, 0.32}, {-0.40, -0.52, 1.7, 0.32}}},
    };
    static const ToothTemplate upperT[] = {
        {ToothKind::CentralIncisor, "Upper central incisor", 8.6, 7.0, 10.5, 0.30, 2.6, 0.95, 0.18, 0.4, 0.0,
         {{0.0, 0.1, 0.9, 0.9}}},
        {ToothKind::LateralIncisor, "Upper lateral incisor", 6.6, 6.0, 9.0, 0.30, 2.4, 0.92, 0.18, 0.4, 0.0,
         {{0.0, 0.1, 0.9, 0.9}}},
        {ToothKind::Canine, "Upper canine", 7.6, 8.0, 10.0, 0.33, 2.2, 0.80, 0.30, 0.5, 0.0,
         {{0.05, 0.2, 2.3, 0.38}}},
        {ToothKind::FirstPremolar, "Upper first premolar", 7.0, 9.0, 8.2, 0.35, 2.3, 0.75, 0.66, 1.0, 0.35,
         {{0.0, 0.50, 2.0, 0.38}, {0.0, -0.50, 1.6, 0.36}}},
        {ToothKind::SecondPremolar, "Upper second premolar", 6.8, 9.0, 7.6, 0.35, 2.3, 0.75, 0.66, 1.0, 0.35,
         {{0.0, 0.50, 1.8, 0.38}, {0.0, -0.50, 1.7, 0.38}}},
        {ToothKind::FirstMolar, "Upper first molar", 10.2, 11.2, 7.3, 0.35, 2.6, 0.80, 0.68, 1.4, 0.45,
         {{0.45, 0.52, 1.6, 0.30}, {-0.40, 0.55, 1.4, 0.28}, {0.40, -0.48, 2.0, 0.34}, {-0.50, -0.45, 1.2, 0.26}}},
        {ToothKind::SecondMolar, "Upper second molar", 9.0, 11.0, 7.0, 0.35, 2.6, 0.80, 0.68, 1.3, 0.45,
         {{0.45, 0.52, 1.6, 0.30}, {-0.40, 0.50, 1.3, 0.28}, {0.30, -0.48, 1.9, 0.36}}},
    };
    const auto index = static_cast<std::size_t>(kind);
    return upper ? upperT[index] : lower[index];
}

double occlusalHeight(const ToothTemplate& t, double u, double v, double cuspScale)
{
    const double rho2 = std::min(u * u + v * v, 1.5);
    // Shallow bowl (central fossa) rising towards the marginal ridges.
    double h = -t.fossaDepth * std::max(0.0, 1.0 - rho2);
    for (const auto& c : t.cusps) {
        const double du = u - c.u, dv = v - c.v;
        h += c.height * std::exp(-(du * du + dv * dv) / (2.0 * c.radius * c.radius));
    }
    // Central fissure along the mesio-distal direction.
    if (t.grooveDepth > 0.0)
        h -= t.grooveDepth * std::exp(-(v * v) / (2.0 * 0.10 * 0.10)) * std::clamp((0.9 - std::abs(u)) / 0.3, 0.0, 1.0);
    return h * cuspScale;
}

double occlusalMaxHeight(const ToothTemplate& t, double cuspScale)
{
    double best = -1e9;
    for (int i = -20; i <= 20; ++i)
        for (int j = -20; j <= 20; ++j) {
            const double u = i / 20.0, v = j / 20.0;
            if (u * u + v * v <= 1.0)
                best = std::max(best, occlusalHeight(t, u, v, cuspScale));
        }
    return best;
}

double outlineFactor(double c, double s, double n)
{
    // Superellipse |x|^n + |y|^n = 1 along the ray (c, s).
    const double denom = std::pow(std::pow(std::abs(c), n) + std::pow(std::abs(s), n), 1.0 / n);
    return denom > 1e-12 ? 1.0 / denom : 1.0;
}

} // namespace occlusa::crown
