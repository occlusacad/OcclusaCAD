#include "db/CloudCaseRepository.h"

namespace occlusa::db {

namespace {

// TODO(cloud): object storage backed file store with a local cache.
class CloudFileStore final : public ICaseFileStore {
public:
    std::filesystem::path caseDirectory(const CaseRecord&) override { fail(); }
    std::string importFile(const CaseRecord&, const std::filesystem::path&, FileRole, const ProgressFn&) override { fail(); }
    std::string importDirectory(const CaseRecord&, const std::filesystem::path&, FileRole, const ProgressFn&) override { fail(); }
    std::filesystem::path resolve(const CaseRecord&, const std::string&) override { fail(); }
    void remove(const CaseRecord&, const std::string&) override { fail(); }
    std::filesystem::path designDirectory(const CaseRecord&) override { fail(); }
    std::string relativize(const CaseRecord&, const std::filesystem::path&) override { fail(); }

private:
    [[noreturn]] static void fail() { throw NotImplementedError("Cloud file storage is not implemented yet"); }
};

} // namespace

CloudCaseRepository::CloudCaseRepository(std::string endpoint, std::string tenant) : endpoint_(std::move(endpoint)), tenant_(std::move(tenant))
{
}

void CloudCaseRepository::notImplemented(const char* what) const
{
    throw NotImplementedError(std::string("Cloud case database: ") + what + " is not implemented yet (endpoint " + endpoint_ + ")");
}

std::vector<CaseSummary> CloudCaseRepository::listCases(const CaseQuery&) { notImplemented("listing cases"); }
std::optional<CaseRecord> CloudCaseRepository::loadCase(const std::string&) { notImplemented("loading cases"); }
CaseRecord CloudCaseRepository::createCase(CaseRecord) { notImplemented("creating cases"); }
void CloudCaseRepository::updateCase(CaseRecord&) { notImplemented("saving cases"); }
void CloudCaseRepository::setStatus(const std::string&, CaseStatus) { notImplemented("changing status"); }
void CloudCaseRepository::deleteCase(const std::string&) { notImplemented("deleting cases"); }
std::string CloudCaseRepository::nextCaseNumber() { notImplemented("case numbering"); }
std::optional<std::string> CloudCaseRepository::loadDesignState(const std::string&) { notImplemented("loading designs"); }
void CloudCaseRepository::saveDesignState(const std::string&, const std::string&, const std::string&) { notImplemented("saving designs"); }
LockResult CloudCaseRepository::acquireLock(const std::string&, const std::string&, bool) { notImplemented("case locking"); }
void CloudCaseRepository::releaseLock(const std::string&, const std::string&) { notImplemented("case locking"); }

ICaseFileStore& CloudCaseRepository::files()
{
    static CloudFileStore store;
    return store;
}

} // namespace occlusa::db
