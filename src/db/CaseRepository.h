#pragma once

#include "core/Progress.h"
#include "db/CaseModel.h"

#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace occlusa {
struct AppConfig;
}

namespace occlusa::db {

class DbError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Thrown when saving a record that someone else modified since it was loaded.
class ConcurrencyError : public DbError {
public:
    using DbError::DbError;
};

class NotImplementedError : public DbError {
public:
    using DbError::DbError;
};

struct LockResult {
    bool acquired = false;
    std::string holder; // current lock owner when not acquired
    std::string sinceUtc;
};

// Storage for the files that belong to a case (scans, DICOM, design output).
// The local implementation keeps them in a folder per case under the data root
// (typically a network share); a cloud implementation would sync to object storage.
class ICaseFileStore {
public:
    virtual ~ICaseFileStore() = default;

    // Local directory for a case (created on demand). For remote stores this is a cache.
    virtual std::filesystem::path caseDirectory(const CaseRecord& record) = 0;

    // Copy a file / directory into the case and return its case-relative path.
    virtual std::string importFile(const CaseRecord& record, const std::filesystem::path& source, FileRole role,
                                   const ProgressFn& progress = {}) = 0;
    virtual std::string importDirectory(const CaseRecord& record, const std::filesystem::path& source, FileRole role,
                                        const ProgressFn& progress = {}) = 0;

    // Local path of a case-relative path (downloads first for remote stores).
    virtual std::filesystem::path resolve(const CaseRecord& record, const std::string& relativePath) = 0;

    // Remove a file or directory previously imported.
    virtual void remove(const CaseRecord& record, const std::string& relativePath) = 0;

    // Directory where design results are written (case-relative "design/").
    virtual std::filesystem::path designDirectory(const CaseRecord& record) = 0;
    // Make a local path inside the case folder case-relative ('/' separated).
    virtual std::string relativize(const CaseRecord& record, const std::filesystem::path& localPath) = 0;
};

// Case database. All methods may throw DbError.
class ICaseRepository {
public:
    virtual ~ICaseRepository() = default;

    virtual std::string backendName() const = 0;
    virtual std::string location() const = 0;

    virtual std::vector<CaseSummary> listCases(const CaseQuery& query) = 0;
    virtual std::optional<CaseRecord> loadCase(const std::string& uuid) = 0;

    // Insert a new case. Assigns uuid, case number (if empty), folder and timestamps.
    virtual CaseRecord createCase(CaseRecord draft) = 0;

    // Save all fields, restorations and files. Fails with ConcurrencyError if the stored
    // revision differs from record.revision; on success record.revision is incremented.
    virtual void updateCase(CaseRecord& record) = 0;
    virtual void setStatus(const std::string& uuid, CaseStatus status) = 0;
    virtual void deleteCase(const std::string& uuid) = 0;

    virtual std::string nextCaseNumber() = 0;

    // Design scene saved by OcclusaCAD (JSON document).
    virtual std::optional<std::string> loadDesignState(const std::string& uuid) = 0;
    virtual void saveDesignState(const std::string& uuid, const std::string& json, const std::string& modifiedBy) = 0;

    // Advisory lock while a case is open for design.
    virtual LockResult acquireLock(const std::string& uuid, const std::string& owner, bool force) = 0;
    virtual void releaseLock(const std::string& uuid, const std::string& owner) = 0;

    virtual ICaseFileStore& files() = 0;
};

// Create the repository configured in `config` ("sqlite" or "cloud").
std::unique_ptr<ICaseRepository> openRepository(const AppConfig& config);

// "user@host" identifier used for locks and audit fields.
std::string currentUserTag();

} // namespace occlusa::db
