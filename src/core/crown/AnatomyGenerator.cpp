#include "core/crown/AnatomyGenerator.h"

#include "core/MeshTopology.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <vector>

namespace occlusa::crown {

namespace {

// ---------------------------------------------------------------------------
// Anatomy description
// ---------------------------------------------------------------------------

struct Feature {
    enum Type { Cusp, Ridge, Groove, Pit } type;
    glm::dvec2 a; // occlusal table coordinates: u mesial (+) / distal (-), v buccal (+) / lingual (-), in [-1, 1]
    glm::dvec2 b; // segment end (ridges, grooves)
    double h;     // height (cusps, ridges) or depth (grooves, pits), mm
    double w;     // width (Gaussian sigma) in table units
};

// Directions of the 8-entry tables: mesial, mesio-buccal, buccal, disto-buccal, distal,
// disto-lingual, lingual, mesio-lingual (counter-clockwise from mesial, buccal = +90 degrees).
using Dir8 = std::array<double, 8>;

struct Spec {
    const char* name;
    double md, bl, height;    // contour widths and cervical-to-cusp-tip height, mm
    double cervical;          // cervical outline relative to the contour
    double squareness;        // superellipse exponent of the contour outline
    Dir8 outline;             // radius modulation of the contour outline (relative)
    Dir8 contourZ;            // height of contour, fraction of the crown height
    double tableMD, tableBL;  // occlusal table / incisal edge half widths relative to the contour
    double tableShift;        // table centre towards buccal (+) as a fraction of the bucco-lingual half width
    double fossa;             // central fossa depth below the table edge, mm
    double marginal;          // marginal ridge height, mm
    double lingualConcavity;  // lingual fossa of anterior teeth, mm
    std::vector<Feature> features;
};

Feature cusp(double u, double v, double h, double w) { return {Feature::Cusp, {u, v}, {u, v}, h, w}; }
Feature ridge(double u0, double v0, double u1, double v1, double h, double w) { return {Feature::Ridge, {u0, v0}, {u1, v1}, h, w}; }
Feature groove(double u0, double v0, double u1, double v1, double d, double w) { return {Feature::Groove, {u0, v0}, {u1, v1}, d, w}; }
Feature pit(double u, double v, double d, double w) { return {Feature::Pit, {u, v}, {u, v}, d, w}; }

// Triangular ridges from each cusp tip towards the central fossa.
void addTriangularRidges(Spec& s, double h, double w)
{
    std::vector<Feature> ridges;
    for (const auto& f : s.features)
        if (f.type == Feature::Cusp)
            ridges.push_back(ridge(f.a.x, f.a.y, f.a.x * 0.25, f.a.y * 0.2, h, w));
    s.features.insert(s.features.end(), ridges.begin(), ridges.end());
}

std::vector<Spec> makeLowerSpecs()
{
    std::vector<Spec> v(7);
    v[0] = {"Lower central incisor", 5.4, 6.0, 9.0, 0.68, 2.3, {0, -0.02, -0.04, -0.02, 0, -0.08, -0.15, -0.08},
            {0.62, 0.35, 0.22, 0.35, 0.60, 0.30, 0.22, 0.30}, 0.92, 0.12, 0.20, 0.0, 0.0, 0.50,
            {ridge(-0.95, 0, 0.95, 0, 0.25, 0.30)}};
    v[1] = {"Lower lateral incisor", 5.9, 6.2, 9.4, 0.68, 2.3, {0, -0.02, -0.04, -0.02, -0.03, -0.08, -0.15, -0.08},
            {0.62, 0.35, 0.22, 0.35, 0.58, 0.30, 0.22, 0.30}, 0.92, 0.12, 0.20, 0.0, 0.0, 0.50,
            {ridge(-0.95, 0, 0.95, 0, 0.25, 0.30)}};
    v[2] = {"Lower canine", 6.9, 7.7, 10.5, 0.70, 2.2, {0, 0, -0.02, 0, -0.02, -0.06, -0.12, -0.06},
            {0.72, 0.35, 0.25, 0.35, 0.62, 0.30, 0.25, 0.30}, 0.80, 0.28, 0.15, 0.0, 0.0, 0.35,
            {cusp(0.15, 0.05, 0.8, 0.36), ridge(0.15, 0.05, 0.95, -0.05, 0.9, 0.2), ridge(0.15, 0.05, -0.95, -0.1, 0.85, 0.2)}};
    v[3] = {"Lower first premolar", 7.0, 7.7, 8.0, 0.72, 2.3, {0, 0.02, 0, 0.02, 0, -0.04, -0.10, -0.04},
            {0.62, 0.35, 0.25, 0.35, 0.60, 0.42, 0.50, 0.42}, 0.70, 0.62, 0.12, 0.5, 0.35, 0.0,
            {cusp(0, 0.45, 1.5, 0.38), cusp(0.05, -0.60, 0.6, 0.28), ridge(0, 0.45, 0.03, -0.55, 0.55, 0.13), pit(0.42, -0.1, 0.5, 0.11),
             pit(-0.42, -0.1, 0.5, 0.11)}};
    v[4] = {"Lower second premolar", 7.1, 8.0, 7.6, 0.74, 2.4, {0, 0.02, 0, 0.02, 0, 0.02, 0, 0.02},
            {0.62, 0.35, 0.25, 0.35, 0.60, 0.42, 0.45, 0.42}, 0.74, 0.66, 0.05, 0.6, 0.35, 0.0,
            {cusp(0, 0.48, 1.4, 0.36), cusp(0.38, -0.48, 1.05, 0.28), cusp(-0.38, -0.50, 1.05, 0.24), ridge(0, 0.48, 0, 0.05, 0.4, 0.12),
             groove(-0.62, 0.02, 0.62, 0, 0.5, 0.06), groove(0, 0, 0, -0.95, 0.4, 0.055), pit(0, 0, 0.35, 0.09)}};
    v[5] = {"Lower first molar", 11.0, 10.3, 7.4, 0.78, 2.7, {0.02, 0.02, 0, 0, -0.05, -0.03, 0, 0.02},
            {0.65, 0.30, 0.22, 0.30, 0.60, 0.42, 0.45, 0.42}, 0.80, 0.66, 0.04, 0.9, 0.45, 0.0,
            {cusp(0.50, 0.50, 1.5, 0.26), cusp(-0.05, 0.56, 1.4, 0.26), cusp(-0.63, 0.32, 1.1, 0.20), cusp(0.47, -0.50, 1.85, 0.27),
             cusp(-0.35, -0.52, 1.7, 0.27), groove(-0.82, 0.03, 0.82, 0, 0.55, 0.055), groove(0.22, 0.05, 0.26, 0.97, 0.5, 0.05),
             groove(-0.36, 0.08, -0.42, 0.96, 0.45, 0.05), groove(0.06, 0, 0.08, -0.97, 0.45, 0.05), pit(0, 0, 0.4, 0.09), pit(0.66, 0, 0.35, 0.08),
             pit(-0.66, 0.04, 0.35, 0.08)}};
    v[6] = {"Lower second molar", 10.5, 10.0, 7.0, 0.78, 2.8, {0.02, 0.02, 0, 0, -0.04, -0.02, 0, 0.02},
            {0.65, 0.30, 0.22, 0.30, 0.60, 0.42, 0.45, 0.42}, 0.80, 0.68, 0.03, 0.85, 0.45, 0.0,
            {cusp(0.45, 0.50, 1.4, 0.28), cusp(-0.42, 0.50, 1.3, 0.28), cusp(0.45, -0.50, 1.65, 0.28), cusp(-0.42, -0.50, 1.5, 0.28),
             groove(-0.85, 0, 0.85, 0, 0.55, 0.055), groove(0, 0, 0, 0.97, 0.45, 0.05), groove(0, 0, 0, -0.97, 0.45, 0.05), pit(0, 0, 0.4, 0.09),
             pit(0.68, 0, 0.3, 0.08), pit(-0.68, 0, 0.3, 0.08)}};
    for (int k : {3, 4, 5, 6})
        addTriangularRidges(v[static_cast<std::size_t>(k)], 0.35, 0.11);
    return v;
}

std::vector<Spec> makeUpperSpecs()
{
    std::vector<Spec> v(7);
    v[0] = {"Upper central incisor", 8.6, 7.0, 10.5, 0.66, 2.9, {0.02, 0, -0.02, 0, 0, -0.08, -0.16, -0.08},
            {0.80, 0.35, 0.20, 0.35, 0.70, 0.28, 0.18, 0.28}, 0.95, 0.12, 0.30, 0.0, 0.0, 0.85,
            {ridge(-0.95, 0, 0.95, 0, 0.25, 0.30)}};
    v[1] = {"Upper lateral incisor", 6.6, 6.0, 9.0, 0.66, 2.4, {0.02, 0, -0.02, 0, -0.04, -0.08, -0.14, -0.08},
            {0.75, 0.35, 0.20, 0.35, 0.60, 0.28, 0.20, 0.28}, 0.90, 0.13, 0.28, 0.0, 0.0, 0.75,
            {ridge(-0.95, 0, 0.95, 0, 0.25, 0.30)}};
    v[2] = {"Upper canine", 7.6, 8.0, 10.0, 0.70, 2.2, {0, 0, 0, 0, -0.02, -0.06, -0.10, -0.06},
            {0.72, 0.33, 0.22, 0.33, 0.62, 0.30, 0.22, 0.30}, 0.80, 0.30, 0.15, 0.0, 0.0, 0.45,
            {cusp(0.10, 0.10, 0.9, 0.36), ridge(0.10, 0.10, 0.95, -0.05, 0.95, 0.2), ridge(0.10, 0.10, -0.95, -0.1, 0.9, 0.2),
             ridge(0.10, 0.10, 0.05, -0.9, 0.5, 0.2)}};
    v[3] = {"Upper first premolar", 7.0, 9.0, 8.2, 0.72, 2.3, {-0.02, 0.02, 0, 0.02, 0, 0, -0.04, 0},
            {0.62, 0.35, 0.25, 0.35, 0.60, 0.42, 0.45, 0.42}, 0.70, 0.68, 0.0, 0.6, 0.35, 0.0,
            {cusp(0.06, 0.50, 1.5, 0.36), cusp(0.12, -0.50, 1.2, 0.34), groove(-0.7, 0, 0.7, 0, 0.55, 0.06), groove(0.7, 0, 1.0, 0, 0.35, 0.05),
             pit(0.62, 0, 0.35, 0.08), pit(-0.62, 0, 0.35, 0.08)}};
    v[4] = {"Upper second premolar", 6.8, 9.0, 7.6, 0.74, 2.4, {0, 0.02, 0, 0.02, 0, 0.02, 0, 0.02},
            {0.62, 0.35, 0.25, 0.35, 0.60, 0.42, 0.45, 0.42}, 0.72, 0.68, 0.0, 0.55, 0.35, 0.0,
            {cusp(0, 0.48, 1.3, 0.36), cusp(0, -0.48, 1.25, 0.36), groove(-0.45, 0, 0.45, 0, 0.45, 0.06), pit(0.5, 0, 0.35, 0.08),
             pit(-0.5, 0, 0.35, 0.08)}};
    v[5] = {"Upper first molar", 10.2, 11.2, 7.3, 0.78, 2.6, {0, 0.04, 0, -0.03, 0, 0.04, 0, -0.02},
            {0.65, 0.30, 0.22, 0.30, 0.60, 0.42, 0.45, 0.42}, 0.80, 0.68, 0.0, 0.9, 0.45, 0.0,
            {cusp(0.47, 0.52, 1.5, 0.26), cusp(-0.42, 0.55, 1.3, 0.24), cusp(0.32, -0.45, 2.0, 0.33), cusp(-0.55, -0.50, 1.0, 0.22),
             ridge(0.32, -0.45, -0.42, 0.55, 0.5, 0.13), groove(0.78, -0.05, 0.02, 0.05, 0.55, 0.055), groove(0.02, 0.05, 0.05, 0.97, 0.5, 0.05),
             groove(-0.25, -0.1, -0.35, -0.97, 0.45, 0.05), pit(0, 0, 0.4, 0.09), pit(0.62, 0, 0.35, 0.08), pit(-0.35, -0.05, 0.35, 0.08)}};
    v[6] = {"Upper second molar", 9.0, 11.0, 7.0, 0.78, 2.6, {0, 0.03, 0, -0.04, -0.04, 0.02, 0, -0.02},
            {0.65, 0.30, 0.22, 0.30, 0.60, 0.42, 0.45, 0.42}, 0.80, 0.68, 0.0, 0.85, 0.45, 0.0,
            {cusp(0.45, 0.52, 1.5, 0.27), cusp(-0.45, 0.50, 1.2, 0.24), cusp(0.30, -0.45, 1.9, 0.33), cusp(-0.50, -0.50, 0.8, 0.20),
             ridge(0.30, -0.45, -0.45, 0.50, 0.45, 0.13), groove(0.75, 0, 0, 0.05, 0.5, 0.055), groove(0, 0.05, 0.03, 0.97, 0.45, 0.05),
             pit(0, 0, 0.4, 0.09), pit(0.6, 0, 0.3, 0.08)}};
    for (int k : {3, 4, 5, 6})
        addTriangularRidges(v[static_cast<std::size_t>(k)], 0.35, 0.11);
    return v;
}

const Spec& baseSpec(ToothKind kind, bool upper)
{
    static const std::vector<Spec> lower = makeLowerSpecs();
    static const std::vector<Spec> upperS = makeUpperSpecs();
    return (upper ? upperS : lower)[static_cast<std::size_t>(kind)];
}

Spec styled(Spec s, LibraryStyle style)
{
    double cusps = 1.0, grooves = 1.0, fossa = 1.0, table = 1.0, square = 1.0, height = 1.0;
    switch (style) {
    case LibraryStyle::Natural: break;
    case LibraryStyle::Young: cusps = 1.3, grooves = 1.3, fossa = 1.2, table = 0.95, square = 0.93, height = 1.03; break;
    case LibraryStyle::Mature: cusps = 0.55, grooves = 0.6, fossa = 0.7, table = 1.12, square = 1.05, height = 0.94; break;
    }
    for (auto& f : s.features)
        f.h *= (f.type == Feature::Cusp || f.type == Feature::Ridge) ? cusps : grooves;
    s.fossa *= fossa;
    s.marginal *= std::sqrt(cusps);
    s.tableMD = std::min(s.tableMD * table, 0.95);
    s.tableBL = std::min(s.tableBL * (s.tableBL < 0.35 ? std::sqrt(table) : table), 0.9);
    s.squareness = std::max(s.squareness * square, 2.0);
    s.height *= height;
    return s;
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

double interp8(const Dir8& v, double theta)
{
    double t = theta / (std::numbers::pi / 4.0);
    t -= 8.0 * std::floor(t / 8.0);
    const int i = static_cast<int>(std::floor(t)) % 8;
    const double f = t - std::floor(t);
    const double s = 0.5 - 0.5 * std::cos(f * std::numbers::pi); // smooth between the samples
    return v[static_cast<std::size_t>(i)] * (1.0 - s) + v[static_cast<std::size_t>((i + 1) % 8)] * s;
}

double superellipse(double c, double s, double a, double b, double n)
{
    const double d = std::pow(std::pow(std::abs(c) / a, n) + std::pow(std::abs(s) / b, n), 1.0 / n);
    return d > 1e-12 ? 1.0 / d : std::max(a, b);
}

double segmentDistance(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& b, double* t = nullptr)
{
    const glm::dvec2 ab = b - a;
    const double len2 = glm::dot(ab, ab);
    const double tt = len2 > 1e-12 ? std::clamp(glm::dot(p - a, ab) / len2, 0.0, 1.0) : 0.0;
    if (t)
        *t = tt;
    return glm::length(p - (a + ab * tt));
}

double smoothstep(double e0, double e1, double x)
{
    const double t = std::clamp((x - e0) / (e1 - e0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// Occlusal relief (mm) at table coordinates (u, v).
double relief(const Spec& s, double u, double v)
{
    const double rho2 = std::min(u * u + v * v, 1.0);
    double h = -s.fossa * (1.0 - rho2);
    h += s.marginal * std::exp(-std::pow(std::abs(u) - 0.8, 2) / (2.0 * 0.08 * 0.08)) * smoothstep(0.35, 0.65, std::abs(u)) * std::max(0.0, 1.0 - v * v);
    const glm::dvec2 p(u, v);
    for (const auto& f : s.features) {
        double t = 0.0;
        const double d = segmentDistance(p, f.a, f.b, &t);
        const double g = std::exp(-d * d / (2.0 * f.w * f.w));
        switch (f.type) {
        case Feature::Cusp: {
            // Broad, rounded cone: cusps are wide at the base with a blunt tip.
            const double r = d / (3.0 * f.w);
            const double cone = r < 1.0 ? (1.0 - r * r) * (1.0 - r * r) * (1.0 - 0.35 * r) : 0.0;
            h += f.h * (0.75 * cone + 0.25 * g);
            break;
        }
        case Feature::Ridge: h += f.h * g * (1.0 - 0.5 * t); break;
        case Feature::Groove: h -= f.h * g; break;
        case Feature::Pit: h -= f.h * g; break;
        }
    }
    return h;
}

glm::dvec2 bezier(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, double t)
{
    const double u = 1.0 - t;
    return u * u * u * p0 + 3.0 * u * u * t * p1 + 3.0 * u * t * t * p2 + t * t * t * p3;
}

} // namespace

std::string_view toString(LibraryStyle style)
{
    switch (style) {
    case LibraryStyle::Natural: return "Natural";
    case LibraryStyle::Young: return "Young";
    case LibraryStyle::Mature: return "Mature";
    }
    return "";
}

const ToothTemplate& toothTemplate(ToothKind kind, bool upper)
{
    static const auto table = [] {
        std::array<ToothTemplate, 14> t{};
        for (int up = 0; up < 2; ++up)
            for (int k = 0; k < 7; ++k) {
                const Spec& s = baseSpec(static_cast<ToothKind>(k), up == 1);
                ToothTemplate& d = t[static_cast<std::size_t>(up * 7 + k)];
                d.kind = static_cast<ToothKind>(k);
                d.name = s.name;
                d.mesioDistal = s.md;
                d.buccoLingual = s.bl;
                d.crownHeight = s.height;
                d.contourHeight = 0.5 * (s.contourZ[0] + s.contourZ[4]);
            }
        return t;
    }();
    return table[static_cast<std::size_t>((upper ? 7 : 0) + static_cast<int>(kind))];
}

Mesh generateToothCrown(ToothKind kind, bool upper, LibraryStyle style, int azimuthSamples)
{
    const Spec s = styled(baseSpec(kind, upper), style);
    const int N = std::max(azimuthSamples, 24);
    constexpr int kLower = 14, kUpper = 16, kCap = 26;
    const double tableA = s.tableMD * s.md * 0.5, tableB = s.tableBL * s.bl * 0.5;
    // Shift of the table (incisal edges sit labially) while keeping the tooth axis inside it.
    const double tableCy = std::clamp(s.tableShift * s.bl * 0.5, -0.6 * tableB, 0.6 * tableB);

    // Highest relief over the table: the cusp tips reach the crown height.
    double maxRelief = -1e9;
    for (int i = -40; i <= 40; ++i)
        for (int j = -40; j <= 40; ++j) {
            const double u = i / 40.0, v = j / 40.0;
            if (u * u + v * v <= 1.0)
                maxRelief = std::max(maxRelief, relief(s, u, v));
        }
    const double capBase = s.height - maxRelief;

    struct Column {
        std::vector<glm::dvec2> profile; // (r, z) from the cervical line up to just before the centre
    };
    std::vector<Column> cols(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) {
        const double th = 2.0 * std::numbers::pi * i / N;
        const double c = std::cos(th), sn = std::sin(th);
        const double Rc = superellipse(c, sn, s.md * 0.5, s.bl * 0.5, s.squareness) * (1.0 + interp8(s.outline, th));
        const double Rcerv = Rc * s.cervical;
        // Table edge along this direction (ray from the axis to the shifted table ellipse).
        const double A = c * c / (tableA * tableA) + sn * sn / (tableB * tableB);
        const double B = sn * tableCy / (tableB * tableB);
        const double C = tableCy * tableCy / (tableB * tableB) - 1.0;
        double rimR = (B + std::sqrt(std::max(B * B - A * C, 0.0))) / A;
        rimR = std::clamp(rimR, 0.2, Rc - 0.3);
        auto tableUV = [&](double r) { return glm::dvec2(r * c / tableA, (r * sn - tableCy) / tableB); };
        const glm::dvec2 rimUV = tableUV(rimR);
        const double zr = capBase + relief(s, rimUV.x, rimUV.y);
        double zc = s.height * interp8(s.contourZ, th);
        zc = std::clamp(zc, 0.15 * s.height, zr - 0.6);

        auto& prof = cols[static_cast<std::size_t>(i)].profile;
        for (int k = 0; k <= kLower; ++k) {
            const double t = static_cast<double>(k) / kLower;
            prof.push_back(bezier({Rcerv, 0.0}, {Rcerv + 0.6 * (Rc - Rcerv), 0.35 * zc}, {Rc, 0.65 * zc}, {Rc, zc}, t));
        }
        // Contour to the table edge; anterior lingual surfaces are concave (lingual fossa).
        const double lingual = sn < 0.0 ? sn * sn : 0.0;
        for (int k = 1; k <= kUpper; ++k) {
            const double t = static_cast<double>(k) / kUpper;
            glm::dvec2 q = bezier({Rc, zc}, {Rc, zc + 0.5 * (zr - zc)}, {rimR + 0.35 * (Rc - rimR), zr}, {rimR, zr}, t);
            q.x -= s.lingualConcavity * lingual * std::sin(std::numbers::pi * t);
            prof.push_back(q);
        }
        // Occlusal surface towards the axis.
        for (int k = 1; k < kCap; ++k) {
            const double r = rimR * (1.0 - static_cast<double>(k) / kCap);
            const glm::dvec2 uv = tableUV(r);
            prof.push_back({r, capBase + relief(s, uv.x, uv.y)});
        }
    }
    const double apexZ = capBase + relief(s, 0.0, -tableCy / tableB);

    Mesh m;
    const int rings = static_cast<int>(cols[0].profile.size());
    for (int k = 0; k < rings; ++k)
        for (int i = 0; i < N; ++i) {
            const double th = 2.0 * std::numbers::pi * i / N;
            const glm::dvec2 q = cols[static_cast<std::size_t>(i)].profile[static_cast<std::size_t>(k)];
            m.positions.emplace_back(q.x * std::cos(th), q.x * std::sin(th), q.y);
        }
    const auto top = static_cast<std::uint32_t>(m.positions.size());
    m.positions.emplace_back(0.0f, 0.0f, static_cast<float>(apexZ));
    const auto bottom = static_cast<std::uint32_t>(m.positions.size());
    m.positions.emplace_back(0.0f, 0.0f, 0.0f);
    auto idx = [&](int k, int i) { return static_cast<std::uint32_t>(k * N + ((i % N) + N) % N); };
    for (int k = 0; k + 1 < rings; ++k)
        for (int i = 0; i < N; ++i)
            m.indices.insert(m.indices.end(), {idx(k, i), idx(k, i + 1), idx(k + 1, i + 1), idx(k, i), idx(k + 1, i + 1), idx(k + 1, i)});
    for (int i = 0; i < N; ++i) {
        m.indices.insert(m.indices.end(), {idx(rings - 1, i), idx(rings - 1, i + 1), top});
        m.indices.insert(m.indices.end(), {idx(0, i + 1), idx(0, i), bottom});
    }
    if (signedVolume(m) < 0.0)
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
            std::swap(m.indices[t + 1], m.indices[t + 2]);
    m.computeVertexNormals();
    return m;
}

} // namespace occlusa::crown
