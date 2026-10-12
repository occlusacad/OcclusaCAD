#include "core/crown/Brush.h"

#include "core/MeshTopology.h"
#include "core/Platform.h"

#include <json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>

namespace occlusa::crown {

namespace {

double smoothstep(double e0, double e1, double x)
{
    const double t = std::clamp((x - e0) / std::max(e1 - e0, 1e-9), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// Smooth 3D value noise in [-1, 1].
double hash3(int x, int y, int z)
{
    std::uint32_t h = static_cast<std::uint32_t>(x) * 374761393u + static_cast<std::uint32_t>(y) * 668265263u + static_cast<std::uint32_t>(z) * 2147483647u;
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= h >> 16;
    return static_cast<double>(h & 0xFFFFFF) / static_cast<double>(0xFFFFFF) * 2.0 - 1.0;
}

double valueNoise(const glm::dvec3& p)
{
    const glm::dvec3 f = glm::floor(p);
    const glm::dvec3 t = p - f;
    const glm::dvec3 s = t * t * (3.0 - 2.0 * t);
    const int x = static_cast<int>(f.x), y = static_cast<int>(f.y), z = static_cast<int>(f.z);
    auto lerp = [](double a, double b, double u) { return a + (b - a) * u; };
    const double c00 = lerp(hash3(x, y, z), hash3(x + 1, y, z), s.x), c10 = lerp(hash3(x, y + 1, z), hash3(x + 1, y + 1, z), s.x);
    const double c01 = lerp(hash3(x, y, z + 1), hash3(x + 1, y, z + 1), s.x), c11 = lerp(hash3(x, y + 1, z + 1), hash3(x + 1, y + 1, z + 1), s.x);
    return lerp(lerp(c00, c10, s.y), lerp(c01, c11, s.y), s.z);
}

const char* kindKey(BrushKind k)
{
    switch (k) {
    case BrushKind::Ball: return "ball";
    case BrushKind::Knife: return "knife";
    case BrushKind::Texture: return "texture";
    default: return "custom";
    }
}

BrushKind kindFromKey(const std::string& s)
{
    if (s == "ball")
        return BrushKind::Ball;
    if (s == "knife")
        return BrushKind::Knife;
    if (s == "texture")
        return BrushKind::Texture;
    return BrushKind::Custom;
}

} // namespace

double BrushTool::weight(double r, const glm::vec3& p) const
{
    if (r >= 1.0)
        return 0.0;
    r = std::max(r, 0.0);
    switch (kind) {
    case BrushKind::Ball: return std::sqrt(1.0 - r * r);
    case BrushKind::Knife: return std::pow(1.0 - r, 3.0);
    case BrushKind::Texture: {
        const double ball = std::sqrt(1.0 - r * r);
        const double n = valueNoise(glm::dvec3(p) / std::max(textureScale, 0.02)) * 0.7 + valueNoise(glm::dvec3(p) / std::max(textureScale * 0.5, 0.01)) * 0.3;
        return ball * std::clamp(n * 1.6 * textureContrast, -1.0, 1.0);
    }
    case BrushKind::Custom: {
        double h = 0.0;
        if (!profile.empty()) {
            if (r <= profile.front().x)
                h = profile.front().y;
            else if (r >= profile.back().x)
                h = profile.back().y;
            else
                for (std::size_t i = 1; i < profile.size(); ++i)
                    if (r <= profile[i].x) {
                        const double u = (r - profile[i - 1].x) / std::max(profile[i].x - profile[i - 1].x, 1e-9);
                        h = profile[i - 1].y + (profile[i].y - profile[i - 1].y) * u;
                        break;
                    }
        }
        const double fall = r <= falloffStart ? 1.0 : std::pow(1.0 - smoothstep(falloffStart, 1.0, r), std::max(falloffPower, 0.05));
        return std::clamp(h, 0.0, 1.0) * fall;
    }
    }
    return 0.0;
}

std::vector<BrushTool> builtinBrushTools()
{
    BrushTool ball;
    ball.name = "Round ball";
    ball.kind = BrushKind::Ball;
    BrushTool knife;
    knife.name = "Pointed knife";
    knife.kind = BrushKind::Knife;
    BrushTool texture;
    texture.name = "Texture";
    texture.kind = BrushKind::Texture;
    return {ball, knife, texture};
}

std::string brushToolToJson(const BrushTool& t)
{
    nlohmann::json j;
    j["format"] = "occlusacad.brush";
    j["version"] = 1;
    j["name"] = t.name;
    j["kind"] = kindKey(t.kind);
    nlohmann::json prof = nlohmann::json::array();
    for (const auto& q : t.profile)
        prof.push_back({q.x, q.y});
    j["profile"] = prof;
    j["falloffStart"] = t.falloffStart;
    j["falloffPower"] = t.falloffPower;
    j["textureScale"] = t.textureScale;
    j["textureContrast"] = t.textureContrast;
    return j.dump(2);
}

BrushTool brushToolFromJson(const std::string& text)
{
    const nlohmann::json j = nlohmann::json::parse(text);
    if (j.value("format", "") != "occlusacad.brush")
        throw std::runtime_error("Not an OcclusaCAD free-forming tool.");
    BrushTool t;
    t.name = j.value("name", "Custom tool");
    t.kind = kindFromKey(j.value("kind", "custom"));
    if (j.contains("profile")) {
        t.profile.clear();
        for (const auto& q : j["profile"])
            if (q.is_array() && q.size() == 2)
                t.profile.emplace_back(std::clamp(q[0].get<double>(), 0.0, 1.0), std::clamp(q[1].get<double>(), 0.0, 1.0));
        std::sort(t.profile.begin(), t.profile.end(), [](const glm::dvec2& a, const glm::dvec2& b) { return a.x < b.x; });
        if (t.profile.size() < 2)
            throw std::runtime_error("The tool profile needs at least two points.");
    }
    t.falloffStart = std::clamp(j.value("falloffStart", t.falloffStart), 0.0, 1.0);
    t.falloffPower = std::clamp(j.value("falloffPower", t.falloffPower), 0.05, 10.0);
    t.textureScale = std::clamp(j.value("textureScale", t.textureScale), 0.02, 5.0);
    t.textureContrast = std::clamp(j.value("textureContrast", t.textureContrast), 0.0, 4.0);
    return t;
}

void saveBrushTool(const std::filesystem::path& file, const BrushTool& tool)
{
    std::filesystem::create_directories(file.parent_path());
    std::ofstream out(file);
    if (!out)
        throw std::runtime_error("Cannot write " + platform::pathToUtf8(file));
    out << brushToolToJson(tool);
}

BrushTool loadBrushTool(const std::filesystem::path& file)
{
    std::ifstream in(file);
    if (!in)
        throw std::runtime_error("Cannot read " + platform::pathToUtf8(file));
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return brushToolFromJson(text);
}

std::vector<BrushTool> loadBrushTools(const std::filesystem::path& folder)
{
    std::vector<BrushTool> out;
    std::error_code ec;
    if (!std::filesystem::is_directory(folder, ec))
        return out;
    std::vector<std::filesystem::path> files;
    for (const auto& e : std::filesystem::directory_iterator(folder, ec))
        if (e.is_regular_file() && e.path().extension() == ".json")
            files.push_back(e.path());
    std::sort(files.begin(), files.end());
    for (const auto& f : files)
        try {
            out.push_back(loadBrushTool(f));
        } catch (const std::exception&) {
        }
    return out;
}

void applyBrushTool(const CrownMesh& crown, std::vector<float>& displacement, const glm::vec3& center, float radius, float strength, BrushMode mode,
                    const BrushTool& tool)
{
    const std::size_t nv = crown.mesh.vertexCount();
    if (displacement.size() != nv)
        displacement.assign(nv, 0.0f);
    std::vector<std::uint32_t> touched;
    std::vector<float> w;
    for (std::size_t v = crown.outerBegin; v < nv; ++v) {
        const float d = glm::length(crown.mesh.positions[v] - center);
        if (d >= radius || crown.editWeight[v] <= 0.0f)
            continue;
        const double f = mode == BrushMode::Smooth || mode == BrushMode::Flatten ? std::sqrt(1.0 - (d / radius) * (d / radius))
                                                                                 : tool.weight(d / radius, crown.mesh.positions[v]);
        if (f == 0.0)
            continue;
        touched.push_back(static_cast<std::uint32_t>(v));
        w.push_back(static_cast<float>(f) * crown.editWeight[v]);
    }
    if (touched.empty())
        return;
    switch (mode) {
    case BrushMode::Add:
    case BrushMode::Remove: {
        const float sign = mode == BrushMode::Add ? 1.0f : -1.0f;
        for (std::size_t k = 0; k < touched.size(); ++k)
            displacement[touched[k]] += sign * strength * w[k];
        break;
    }
    case BrushMode::Smooth: {
        const MeshAdjacency adj(crown.mesh);
        std::vector<float> delta(touched.size(), 0.0f);
        for (std::size_t k = 0; k < touched.size(); ++k) {
            const std::uint32_t v = touched[k];
            glm::vec3 avg(0.0f);
            int cnt = 0;
            for (std::uint32_t nb : adj.neighbors(v)) {
                avg += crown.mesh.positions[nb];
                ++cnt;
            }
            if (cnt > 0)
                delta[k] = glm::dot(avg / static_cast<float>(cnt) - crown.mesh.positions[v], crown.baseNormals[v]) * w[k] * std::clamp(strength * 4.0f, 0.1f, 1.0f);
        }
        for (std::size_t k = 0; k < touched.size(); ++k)
            displacement[touched[k]] += delta[k];
        break;
    }
    case BrushMode::Flatten: {
        // Plane of the brushed area (weighted centroid and normal); pull the surface onto it.
        glm::dvec3 c(0.0), n(0.0);
        double ws = 0.0;
        for (std::size_t k = 0; k < touched.size(); ++k) {
            c += glm::dvec3(crown.mesh.positions[touched[k]]) * static_cast<double>(w[k]);
            n += glm::dvec3(crown.baseNormals[touched[k]]) * static_cast<double>(w[k]);
            ws += w[k];
        }
        if (ws <= 0.0 || glm::length(n) < 1e-9)
            return;
        c /= ws;
        n = glm::normalize(n);
        const double k0 = std::clamp(static_cast<double>(strength) * 3.0, 0.05, 1.0);
        for (std::size_t k = 0; k < touched.size(); ++k) {
            const std::uint32_t v = touched[k];
            const double along = glm::dot(glm::dvec3(crown.baseNormals[v]), n);
            if (along < 0.2)
                continue;
            const double dist = glm::dot(glm::dvec3(crown.mesh.positions[v]) - c, n);
            displacement[v] -= static_cast<float>(dist / along * w[k] * k0);
        }
        break;
    }
    }
}

} // namespace occlusa::crown
