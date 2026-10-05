#pragma once

#include "db/CaseRepository.h"

namespace occlusa::db {

// Case files on a local disk or network share: <casesRoot>/<record.folder>/...
class LocalFileStore final : public ICaseFileStore {
public:
    explicit LocalFileStore(std::filesystem::path casesRoot);

    std::filesystem::path caseDirectory(const CaseRecord& record) override;
    std::string importFile(const CaseRecord& record, const std::filesystem::path& source, FileRole role, const ProgressFn& progress) override;
    std::string importDirectory(const CaseRecord& record, const std::filesystem::path& source, FileRole role,
                                const ProgressFn& progress) override;
    std::filesystem::path resolve(const CaseRecord& record, const std::string& relativePath) override;
    void remove(const CaseRecord& record, const std::string& relativePath) override;
    std::filesystem::path designDirectory(const CaseRecord& record) override;
    std::string relativize(const CaseRecord& record, const std::filesystem::path& localPath) override;

    // Folder name for a new case, e.g. "OC-2026-00012_ab12cd34".
    static std::string makeFolderName(const std::string& caseNumber, const std::string& uuid);
    static std::string sanitizeFileName(const std::string& name);

private:
    std::filesystem::path uniqueTarget(const std::filesystem::path& dir, const std::string& name);
    std::filesystem::path root_;
};

} // namespace occlusa::db
