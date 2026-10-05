#include "db/LocalFileStore.h"

#include "core/Platform.h"

#include <format>

namespace fs = std::filesystem;

namespace occlusa::db {

namespace {
const char* subdirFor(FileRole role)
{
    switch (role) {
    case FileRole::Dicom: return "dicom";
    case FileRole::DesignOutput: return "design";
    default: return "scans";
    }
}
} // namespace

LocalFileStore::LocalFileStore(fs::path casesRoot) : root_(std::move(casesRoot)) {}

std::string LocalFileStore::sanitizeFileName(const std::string& name)
{
    std::string out;
    for (unsigned char c : name) {
        // Keep UTF-8 bytes; replace characters that are invalid on any of our platforms.
        if (c < 0x20 || c == '<' || c == '>' || c == ':' || c == '"' || c == '/' || c == '\\' || c == '|' || c == '?' || c == '*')
            out.push_back('_');
        else
            out.push_back(static_cast<char>(c));
    }
    while (!out.empty() && (out.back() == '.' || out.back() == ' '))
        out.pop_back(); // Windows does not allow trailing dots/spaces
    if (out.empty())
        out = "file";
    return out;
}

std::string LocalFileStore::makeFolderName(const std::string& caseNumber, const std::string& uuid)
{
    return sanitizeFileName(caseNumber) + "_" + uuid.substr(0, 8);
}

fs::path LocalFileStore::caseDirectory(const CaseRecord& record)
{
    if (record.folder.empty())
        throw DbError("Case has no folder assigned");
    fs::path dir = root_ / platform::pathFromUtf8(record.folder);
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec)
        throw DbError("Cannot create case folder " + platform::pathToUtf8(dir) + ": " + ec.message());
    return dir;
}

fs::path LocalFileStore::uniqueTarget(const fs::path& dir, const std::string& name)
{
    fs::path target = dir / platform::pathFromUtf8(name);
    if (!fs::exists(target))
        return target;
    const fs::path stem = target.stem(), ext = target.extension();
    for (int i = 2; i < 10000; ++i) {
        fs::path candidate = dir / platform::pathFromUtf8(platform::pathToUtf8(stem) + "_" + std::to_string(i) + platform::pathToUtf8(ext));
        if (!fs::exists(candidate))
            return candidate;
    }
    throw DbError("Too many files named " + name);
}

std::string LocalFileStore::importFile(const CaseRecord& record, const fs::path& source, FileRole role, const ProgressFn& progress)
{
    const fs::path dir = caseDirectory(record) / subdirFor(role);
    fs::create_directories(dir);
    const fs::path target = uniqueTarget(dir, sanitizeFileName(platform::pathToUtf8(source.filename())));
    reportProgress(progress, 0.0f, "Copying " + platform::pathToUtf8(source.filename()));
    std::error_code ec;
    fs::copy_file(source, target, fs::copy_options::none, ec);
    if (ec)
        throw DbError("Cannot copy " + platform::pathToUtf8(source) + ": " + ec.message());
    reportProgress(progress, 1.0f, "Copied");
    return relativize(record, target);
}

std::string LocalFileStore::importDirectory(const CaseRecord& record, const fs::path& source, FileRole role, const ProgressFn& progress)
{
    const fs::path dir = caseDirectory(record) / subdirFor(role);
    fs::create_directories(dir);
    std::string name = platform::pathToUtf8(source.filename());
    if (name.empty())
        name = platform::pathToUtf8(source.parent_path().filename());
    const fs::path target = uniqueTarget(dir, sanitizeFileName(name.empty() ? "series" : name));

    std::vector<fs::path> files;
    std::error_code ec;
    for (auto it = fs::recursive_directory_iterator(source, fs::directory_options::skip_permission_denied, ec);
         it != fs::recursive_directory_iterator(); it.increment(ec)) {
        if (ec)
            break;
        if (it->is_regular_file())
            files.push_back(it->path());
    }
    if (ec)
        throw DbError("Cannot read folder " + platform::pathToUtf8(source) + ": " + ec.message());
    for (std::size_t i = 0; i < files.size(); ++i) {
        reportProgress(progress, static_cast<float>(i) / static_cast<float>(std::max<std::size_t>(files.size(), 1)),
                       std::format("Copying {} / {}", i + 1, files.size()));
        const fs::path rel = fs::relative(files[i], source);
        const fs::path dst = target / rel;
        fs::create_directories(dst.parent_path());
        fs::copy_file(files[i], dst, fs::copy_options::overwrite_existing, ec);
        if (ec)
            throw DbError("Cannot copy " + platform::pathToUtf8(files[i]) + ": " + ec.message());
    }
    reportProgress(progress, 1.0f, "Copied");
    return relativize(record, target);
}

fs::path LocalFileStore::resolve(const CaseRecord& record, const std::string& relativePath)
{
    return root_ / platform::pathFromUtf8(record.folder) / platform::pathFromUtf8(relativePath);
}

void LocalFileStore::remove(const CaseRecord& record, const std::string& relativePath)
{
    if (relativePath.empty() || relativePath.find("..") != std::string::npos)
        throw DbError("Invalid case file path");
    std::error_code ec;
    fs::remove_all(resolve(record, relativePath), ec);
}

fs::path LocalFileStore::designDirectory(const CaseRecord& record)
{
    const fs::path dir = caseDirectory(record) / "design";
    fs::create_directories(dir);
    return dir;
}

std::string LocalFileStore::relativize(const CaseRecord& record, const fs::path& localPath)
{
    const fs::path base = root_ / platform::pathFromUtf8(record.folder);
    const std::u8string generic = fs::relative(localPath, base).generic_u8string();
    return std::string(generic.begin(), generic.end());
}

} // namespace occlusa::db
