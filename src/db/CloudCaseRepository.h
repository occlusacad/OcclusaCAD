#pragma once

#include "db/CaseRepository.h"

namespace occlusa::db {

// Placeholder for a hosted case database (multi-site labs, remote technicians).
//
// TODO(cloud): planned design
//  * REST/JSON API (HTTPS) mirroring ICaseRepository: GET/POST/PATCH /v1/cases, /v1/cases/{uuid}/design-state,
//    POST /v1/cases/{uuid}/lock with server-side lease expiry instead of advisory locks.
//  * Optimistic concurrency via ETag / If-Match on the case revision (maps to ConcurrencyError).
//  * Case files in object storage (S3-compatible / Azure Blob) via pre-signed URLs; CloudFileStore keeps a
//    local cache under the user's config directory and resolve() downloads on demand.
//  * Authentication with OAuth2 device-code flow; tokens in the OS keychain.
//  * HTTP client: a permissively licensed library (e.g. cpp-httplib, MIT) with the platform TLS stack.
class CloudCaseRepository final : public ICaseRepository {
public:
    CloudCaseRepository(std::string endpoint, std::string tenant);

    std::string backendName() const override { return "Cloud (preview)"; }
    std::string location() const override { return endpoint_; }

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
    ICaseFileStore& files() override;

private:
    [[noreturn]] void notImplemented(const char* what) const;
    std::string endpoint_;
    std::string tenant_;
};

} // namespace occlusa::db
