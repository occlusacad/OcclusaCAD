#pragma once

#include <filesystem>
#include <string>

namespace occlusa {

// Settings shared by OcclusaCAD DB and OcclusaCAD, stored per user as JSON.
struct AppConfig {
    // Case database backend: "sqlite" (file on a local disk or network share) or "cloud" (TODO).
    std::string databaseBackend = "sqlite";
    // Root of the shared lab data folder. Holds the SQLite database and the case folders:
    //   <dataRoot>/occlusacad.sqlite3
    //   <dataRoot>/cases/<case folder>/...
    std::filesystem::path dataRoot;
    // Cloud backend settings (not implemented yet).
    std::string cloudEndpoint;
    std::string cloudTenant;

    std::string theme = "system";       // "system", "light" or "dark"
    std::string toothNumbering = "fdi"; // "fdi" or "universal"
    float uiScale = 0.0f;               // 0 = automatic (monitor content scale)

    bool isConfigured() const;
    std::filesystem::path databasePath() const { return dataRoot / "occlusacad.sqlite3"; }
    std::filesystem::path casesRoot() const { return dataRoot / "cases"; }

    // Location: $OCCLUSACAD_CONFIG if set, else <platform config dir>/config.json.
    static std::filesystem::path defaultPath();
    static AppConfig load(const std::filesystem::path& path);
    static AppConfig loadDefault() { return load(defaultPath()); }
    void save(const std::filesystem::path& path) const;
    void saveDefault() const { save(defaultPath()); }

    static std::filesystem::path suggestedDataRoot();
};

} // namespace occlusa
