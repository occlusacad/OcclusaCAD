// OcclusaCAD - dental CAD design application.
#include "apps/designer/DesignerApp.h"

#include "core/AppConfig.h"
#include "core/CommandLine.h"
#include "core/Log.h"
#include "core/Platform.h"

int main(int argc, char** argv)
{
    using namespace occlusa;
    const CommandLine cl = CommandLine::fromMain(argc, argv);
    log::setLogFile(platform::configDir() / "logs" / "designer.log");
    log::info("OcclusaCAD starting");

    const auto configPath = cl.get("config") ? platform::pathFromUtf8(*cl.get("config")) : AppConfig::defaultPath();
    AppConfig config = AppConfig::load(configPath);
    if (auto root = cl.get("data-root"))
        config.dataRoot = platform::pathFromUtf8(*root);

    designer::DesignerOptions so;
    so.caseUuid = cl.get("case");
    if (auto d = cl.get("dicom"))
        so.dicomPaths.push_back(platform::pathFromUtf8(*d));
    if (auto s = cl.get("scan"))
        so.scanPaths.push_back(platform::pathFromUtf8(*s));
    // Bare file arguments: folders and .dcm files are CBCT, .stl files are scans.
    for (const auto& arg : cl.positional()) {
        const auto p = platform::pathFromUtf8(arg);
        std::string ext = p.extension().string();
        for (auto& c : ext)
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (ext == ".stl")
            so.scanPaths.push_back(p);
        else
            so.dicomPaths.push_back(p);
    }
    so.startStep = cl.get("step");
    if (auto v = cl.get("view")) {
        static const std::pair<const char*, gfx::ViewPreset> presets[] = {{"front", gfx::ViewPreset::Front}, {"back", gfx::ViewPreset::Back},
                                                                          {"left", gfx::ViewPreset::Left},   {"right", gfx::ViewPreset::Right},
                                                                          {"top", gfx::ViewPreset::Top},     {"bottom", gfx::ViewPreset::Bottom}};
        for (const auto& [name, preset] : presets)
            if (*v == name)
                so.viewPreset = preset;
    }
    so.expertMode = cl.has("expert");
    if (auto gt = cl.get("demo-align")) {
        so.demoAutoAlign = true;
        if (!gt->empty())
            so.demoGroundTruth = platform::pathFromUtf8(*gt);
    }

    if (auto c = cl.get("demo-crown"))
        so.demoCrownTruth = platform::pathFromUtf8(*c);
    if (auto e = cl.get("demo-max-margin-error"))
        so.demoMaxMarginError = std::stod(*e);
    so.demoLibrary = cl.get("demo-library").value_or("");
    so.demoSave = cl.has("demo-save");
    if (auto e = cl.get("demo-max-error"))
        so.demoMaxError = std::stod(*e);

    ui::AppOptions options;
    options.title = "OcclusaCAD";
    options.appId = "org.occlusacad.OcclusaCAD";
    options.width = 1720;
    options.height = 1040;
    options.iniFileName = "designer.ini";
    options.theme = ui::themeModeFromString(cl.get("theme").value_or(config.theme));
    options.uiScale = config.uiScale;
    if (auto shot = cl.get("screenshot")) {
        options.screenshotPath = platform::pathFromUtf8(*shot);
        options.width = 1720;
        options.height = 1040;
        options.screenshotFrames = std::stoi(cl.get("frames").value_or("90"));
    }
    designer::DesignerApp app(options, config, so);
    return app.run();
}
