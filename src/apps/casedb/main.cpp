// OcclusaCAD DB - case management.
#include "apps/casedb/DbApp.h"

#include "core/AppConfig.h"
#include "core/CommandLine.h"
#include "core/Log.h"
#include "core/Platform.h"

int main(int argc, char** argv)
{
    using namespace occlusa;
    const CommandLine cl = CommandLine::fromMain(argc, argv);
    log::setLogFile(platform::configDir() / "logs" / "casedb.log");
    log::info("OcclusaCAD DB starting");

    const auto configPath = cl.get("config") ? platform::pathFromUtf8(*cl.get("config")) : AppConfig::defaultPath();
    AppConfig config = AppConfig::load(configPath);
    if (auto root = cl.get("data-root"))
        config.dataRoot = platform::pathFromUtf8(*root);

    ui::AppOptions options;
    options.title = "OcclusaCAD DB";
    options.appId = "org.occlusacad.OcclusaCADDB";
    options.width = 1500;
    options.height = 940;
    options.iniFileName = "casedb.ini";
    options.theme = ui::themeModeFromString(cl.get("theme").value_or(config.theme));
    options.uiScale = config.uiScale;
    if (auto shot = cl.get("screenshot")) {
        options.screenshotPath = platform::pathFromUtf8(*shot);
        options.width = 1600;
        options.height = 1000;
    }
    casedb::DbApp app(options, config, configPath, cl.get("select").value_or(""));
    return app.run();
}
