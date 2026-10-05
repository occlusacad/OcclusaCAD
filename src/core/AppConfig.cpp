#include "core/AppConfig.h"

#include "core/Log.h"
#include "core/Platform.h"

#include <json.hpp>

#include <cstdlib>
#include <fstream>

namespace fs = std::filesystem;
using nlohmann::json;

namespace occlusa {

bool AppConfig::isConfigured() const
{
    if (databaseBackend == "cloud")
        return !cloudEndpoint.empty();
    return !dataRoot.empty();
}

fs::path AppConfig::defaultPath()
{
    if (const char* env = std::getenv("OCCLUSACAD_CONFIG"); env && *env)
        return platform::pathFromUtf8(env);
    return platform::configDir() / "config.json";
}

fs::path AppConfig::suggestedDataRoot()
{
    return platform::documentsDir() / "OcclusaCAD Data";
}

AppConfig AppConfig::load(const fs::path& path)
{
    AppConfig cfg;
    std::ifstream in(path);
    if (in) {
        try {
            json j = json::parse(in);
            cfg.databaseBackend = j.value("databaseBackend", cfg.databaseBackend);
            if (j.contains("dataRoot"))
                cfg.dataRoot = platform::pathFromUtf8(j["dataRoot"].get<std::string>());
            cfg.cloudEndpoint = j.value("cloudEndpoint", cfg.cloudEndpoint);
            cfg.cloudTenant = j.value("cloudTenant", cfg.cloudTenant);
            cfg.theme = j.value("theme", cfg.theme);
            cfg.toothNumbering = j.value("toothNumbering", cfg.toothNumbering);
            cfg.uiScale = j.value("uiScale", cfg.uiScale);
        } catch (const std::exception& e) {
            log::error("Failed to parse config {}: {}", platform::pathToUtf8(path), e.what());
        }
    }
    // Environment override, handy for testing and kiosk deployments.
    if (const char* env = std::getenv("OCCLUSACAD_DATA_ROOT"); env && *env)
        cfg.dataRoot = platform::pathFromUtf8(env);
    return cfg;
}

void AppConfig::save(const fs::path& path) const
{
    json j;
    j["databaseBackend"] = databaseBackend;
    j["dataRoot"] = platform::pathToUtf8(dataRoot);
    j["cloudEndpoint"] = cloudEndpoint;
    j["cloudTenant"] = cloudTenant;
    j["theme"] = theme;
    j["toothNumbering"] = toothNumbering;
    j["uiScale"] = uiScale;
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    // Write atomically so a crash never leaves a truncated config behind.
    const fs::path tmp = path.string() + ".tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        if (!out)
            throw std::runtime_error("Cannot write config: " + platform::pathToUtf8(path));
        out << j.dump(2);
    }
    fs::rename(tmp, path, ec);
    if (ec) {
        fs::remove(path, ec);
        fs::rename(tmp, path);
    }
}

} // namespace occlusa
