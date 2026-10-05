#include "ui/Fonts.h"

#include <span>
#include <string_view>

namespace occlusa_resources {
std::span<const unsigned char> get(std::string_view name);
}

namespace occlusa::ui::fonts {

namespace {
ImFont* gRegular = nullptr;
ImFont* gSemibold = nullptr;

ImFont* addEmbedded(const char* name)
{
    const auto data = occlusa_resources::get(name);
    if (data.empty())
        return nullptr;
    ImFontConfig cfg;
    cfg.FontDataOwnedByAtlas = false; // static data
    cfg.OversampleH = 2;
    cfg.OversampleV = 1;
    return ImGui::GetIO().Fonts->AddFontFromMemoryTTF(const_cast<unsigned char*>(data.data()), static_cast<int>(data.size()), kBaseSize, &cfg);
}
} // namespace

void load()
{
    ImGuiIO& io = ImGui::GetIO();
    gRegular = addEmbedded("Inter-Regular.ttf");
    gSemibold = addEmbedded("Inter-SemiBold.ttf");
    if (!gRegular)
        gRegular = io.Fonts->AddFontDefault();
    if (!gSemibold)
        gSemibold = gRegular;
    io.FontDefault = gRegular;
}

ImFont* regular()
{
    return gRegular;
}

ImFont* semibold()
{
    return gSemibold;
}

} // namespace occlusa::ui::fonts
