#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace occlusa::ui::dialogs {

struct Filter {
    std::string name; // "STL files"
    std::string spec; // "stl" or "stl,ply" (no dots)
};

// Native dialogs (nativefiledialog-extended). All paths are absolute.
void init();
void shutdown();
std::optional<std::filesystem::path> openFile(const std::vector<Filter>& filters, const std::filesystem::path& defaultDir = {});
std::vector<std::filesystem::path> openFiles(const std::vector<Filter>& filters, const std::filesystem::path& defaultDir = {});
std::optional<std::filesystem::path> pickFolder(const std::filesystem::path& defaultDir = {});
std::optional<std::filesystem::path> saveFile(const std::vector<Filter>& filters, const std::string& defaultName,
                                              const std::filesystem::path& defaultDir = {});

} // namespace occlusa::ui::dialogs
