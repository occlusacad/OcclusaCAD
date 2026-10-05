#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace occlusa::platform {

// UTF-8 <-> std::filesystem::path conversion. All strings in OcclusaCAD are UTF-8;
// on Windows paths are natively UTF-16 so the conversion must be explicit.
std::filesystem::path pathFromUtf8(const std::string& utf8);
std::string pathToUtf8(const std::filesystem::path& path);

// Directory containing the running executable.
std::filesystem::path executableDir();

// Per-user configuration directory, e.g.
//   Windows: %APPDATA%\OcclusaCAD
//   macOS:   ~/Library/Application Support/OcclusaCAD
//   Linux:   $XDG_CONFIG_HOME/occlusacad (~/.config/occlusacad)
std::filesystem::path configDir();

// User's documents folder (fallback: home directory).
std::filesystem::path documentsDir();
std::filesystem::path homeDir();

std::string userName();
std::string hostName();

// Best-effort detection of the OS-level dark mode preference.
// Returns std::nullopt when it cannot be determined.
std::optional<bool> systemPrefersDark();

// Start a process detached from the current one. Returns false on failure.
bool launchDetached(const std::filesystem::path& executable, const std::vector<std::string>& args, std::string* error = nullptr);

// Open a folder in the platform file browser (Explorer / Finder / xdg-open).
bool openInFileBrowser(const std::filesystem::path& path);

// Platform specific executable name, e.g. "OcclusaCAD.exe" on Windows.
std::string executableName(const std::string& baseName);

} // namespace occlusa::platform
