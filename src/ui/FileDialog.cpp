#include "ui/FileDialog.h"

#include "core/Log.h"
#include "core/Platform.h"

#include <nfd.h>

namespace occlusa::ui::dialogs {

namespace {
bool gInitialized = false;

std::vector<nfdu8filteritem_t> toNfd(const std::vector<Filter>& filters)
{
    std::vector<nfdu8filteritem_t> out;
    for (const auto& f : filters)
        out.push_back({f.name.c_str(), f.spec.c_str()});
    return out;
}

void ensureInit()
{
    if (!gInitialized)
        init();
}

std::string defaultPathUtf8(const std::filesystem::path& p)
{
    return p.empty() ? std::string() : platform::pathToUtf8(p);
}
} // namespace

void init()
{
    if (NFD_Init() != NFD_OKAY)
        log::error("File dialogs unavailable: {}", NFD_GetError() ? NFD_GetError() : "unknown");
    else
        gInitialized = true;
}

void shutdown()
{
    if (gInitialized)
        NFD_Quit();
    gInitialized = false;
}

std::optional<std::filesystem::path> openFile(const std::vector<Filter>& filters, const std::filesystem::path& defaultDir)
{
    ensureInit();
    auto items = toNfd(filters);
    const std::string def = defaultPathUtf8(defaultDir);
    nfdu8char_t* out = nullptr;
    const nfdresult_t rc = NFD_OpenDialogU8(&out, items.empty() ? nullptr : items.data(), static_cast<nfdfiltersize_t>(items.size()),
                                            def.empty() ? nullptr : def.c_str());
    if (rc == NFD_OKAY) {
        std::filesystem::path p = platform::pathFromUtf8(out);
        NFD_FreePathU8(out);
        return p;
    }
    if (rc == NFD_ERROR)
        log::error("Open dialog failed: {}", NFD_GetError() ? NFD_GetError() : "");
    return std::nullopt;
}

std::vector<std::filesystem::path> openFiles(const std::vector<Filter>& filters, const std::filesystem::path& defaultDir)
{
    ensureInit();
    std::vector<std::filesystem::path> result;
    auto items = toNfd(filters);
    const std::string def = defaultPathUtf8(defaultDir);
    const nfdpathset_t* set = nullptr;
    const nfdresult_t rc = NFD_OpenDialogMultipleU8(&set, items.empty() ? nullptr : items.data(), static_cast<nfdfiltersize_t>(items.size()),
                                                    def.empty() ? nullptr : def.c_str());
    if (rc == NFD_OKAY) {
        nfdpathsetsize_t count = 0;
        NFD_PathSet_GetCount(set, &count);
        for (nfdpathsetsize_t i = 0; i < count; ++i) {
            nfdu8char_t* path = nullptr;
            if (NFD_PathSet_GetPathU8(set, i, &path) == NFD_OKAY) {
                result.push_back(platform::pathFromUtf8(path));
                NFD_PathSet_FreePathU8(path);
            }
        }
        NFD_PathSet_Free(set);
    } else if (rc == NFD_ERROR) {
        log::error("Open dialog failed: {}", NFD_GetError() ? NFD_GetError() : "");
    }
    return result;
}

std::optional<std::filesystem::path> pickFolder(const std::filesystem::path& defaultDir)
{
    ensureInit();
    const std::string def = defaultPathUtf8(defaultDir);
    nfdu8char_t* out = nullptr;
    const nfdresult_t rc = NFD_PickFolderU8(&out, def.empty() ? nullptr : def.c_str());
    if (rc == NFD_OKAY) {
        std::filesystem::path p = platform::pathFromUtf8(out);
        NFD_FreePathU8(out);
        return p;
    }
    if (rc == NFD_ERROR)
        log::error("Folder dialog failed: {}", NFD_GetError() ? NFD_GetError() : "");
    return std::nullopt;
}

std::optional<std::filesystem::path> saveFile(const std::vector<Filter>& filters, const std::string& defaultName,
                                              const std::filesystem::path& defaultDir)
{
    ensureInit();
    auto items = toNfd(filters);
    const std::string def = defaultPathUtf8(defaultDir);
    nfdu8char_t* out = nullptr;
    const nfdresult_t rc = NFD_SaveDialogU8(&out, items.empty() ? nullptr : items.data(), static_cast<nfdfiltersize_t>(items.size()),
                                            def.empty() ? nullptr : def.c_str(), defaultName.c_str());
    if (rc == NFD_OKAY) {
        std::filesystem::path p = platform::pathFromUtf8(out);
        NFD_FreePathU8(out);
        return p;
    }
    if (rc == NFD_ERROR)
        log::error("Save dialog failed: {}", NFD_GetError() ? NFD_GetError() : "");
    return std::nullopt;
}

} // namespace occlusa::ui::dialogs
