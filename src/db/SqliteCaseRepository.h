#pragma once

#include "db/CaseRepository.h"
#include "db/LocalFileStore.h"
#include "db/Sqlite.h"

#include <mutex>

namespace occlusa::db {

// Case database in a single SQLite file, designed to live on a network share that
// several workstations open concurrently:
//  * rollback journal (journal_mode=DELETE): WAL requires shared memory and is unsafe on network file systems
//  * short BEGIN IMMEDIATE write transactions with a busy timeout
//  * optimistic concurrency (revision column) so one workstation never silently overwrites another
//  * file paths stored relative to the case folder so different mount points / drive letters work
class SqliteCaseRepository final : public ICaseRepository {
public:
    SqliteCaseRepository(const std::filesystem::path& databaseFile, const std::filesystem::path& casesRoot);

    std::string backendName() const override { return "SQLite"; }
    std::string location() const override;

    std::vector<CaseSummary> listCases(const CaseQuery& query) override;
    std::optional<CaseRecord> loadCase(const std::string& uuid) override;
    CaseRecord createCase(CaseRecord draft) override;
    void updateCase(CaseRecord& record) override;
    void setStatus(const std::string& uuid, CaseStatus status) override;
    void deleteCase(const std::string& uuid) override;
    std::string nextCaseNumber() override;
    std::optional<std::string> loadDesignState(const std::string& uuid) override;
    void saveDesignState(const std::string& uuid, const std::string& json, const std::string& modifiedBy) override;
    LockResult acquireLock(const std::string& uuid, const std::string& owner, bool force) override;
    void releaseLock(const std::string& uuid, const std::string& owner) override;
    ICaseFileStore& files() override { return files_; }

    static constexpr int kSchemaVersion = 1;

private:
    void migrate();
    std::int64_t caseIdForUuid(const std::string& uuid);
    std::string allocateCaseNumber(); // inside a transaction
    void writeChildren(std::int64_t caseId, const CaseRecord& record);

    std::filesystem::path path_;
    sqlite::Database db_;
    LocalFileStore files_;
    std::recursive_mutex mutex_;
};

} // namespace occlusa::db
