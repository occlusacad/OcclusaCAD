#include "core/AppConfig.h"
#include "core/Platform.h"
#include "db/CaseRepository.h"
#include "db/CloudCaseRepository.h"
#include "db/SqliteCaseRepository.h"

namespace occlusa::db {

std::unique_ptr<ICaseRepository> openRepository(const AppConfig& config)
{
    if (config.databaseBackend == "cloud")
        return std::make_unique<CloudCaseRepository>(config.cloudEndpoint, config.cloudTenant);
    if (config.databaseBackend != "sqlite")
        throw DbError("Unknown database backend: " + config.databaseBackend);
    if (config.dataRoot.empty())
        throw DbError("No data folder configured");
    return std::make_unique<SqliteCaseRepository>(config.databasePath(), config.casesRoot());
}

std::string currentUserTag()
{
    return platform::userName() + "@" + platform::hostName();
}

} // namespace occlusa::db
