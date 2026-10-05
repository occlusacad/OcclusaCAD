#include "db/CaseModel.h"

namespace occlusa::db {

namespace {
struct StatusEntry {
    CaseStatus status;
    const char* key;
    const char* label;
};
constexpr StatusEntry kStatuses[] = {
    {CaseStatus::New, "new", "New"},
    {CaseStatus::InDesign, "in_design", "In design"},
    {CaseStatus::Designed, "designed", "Designed"},
    {CaseStatus::Exported, "exported", "Exported"},
    {CaseStatus::Archived, "archived", "Archived"},
};

struct RoleEntry {
    FileRole role;
    const char* key;
    const char* label;
};
constexpr RoleEntry kRoles[] = {
    {FileRole::Dicom, "dicom", "CBCT (DICOM)"},
    {FileRole::ScanUpper, "scan_upper", "Upper jaw scan"},
    {FileRole::ScanLower, "scan_lower", "Lower jaw scan"},
    {FileRole::ScanBite, "scan_bite", "Bite registration"},
    {FileRole::ScanOther, "scan_other", "Other scan"},
    {FileRole::DesignOutput, "design_output", "Design output"},
};
} // namespace

std::string_view toString(CaseStatus s)
{
    for (const auto& e : kStatuses)
        if (e.status == s)
            return e.key;
    return "new";
}

std::string_view displayName(CaseStatus s)
{
    for (const auto& e : kStatuses)
        if (e.status == s)
            return e.label;
    return "New";
}

CaseStatus caseStatusFromString(std::string_view s)
{
    for (const auto& e : kStatuses)
        if (s == e.key)
            return e.status;
    return CaseStatus::New;
}

const std::vector<CaseStatus>& allCaseStatuses()
{
    static const std::vector<CaseStatus> v = {CaseStatus::New, CaseStatus::InDesign, CaseStatus::Designed, CaseStatus::Exported,
                                              CaseStatus::Archived};
    return v;
}

std::string_view toString(FileRole r)
{
    for (const auto& e : kRoles)
        if (e.role == r)
            return e.key;
    return "scan_other";
}

std::string_view displayName(FileRole r)
{
    for (const auto& e : kRoles)
        if (e.role == r)
            return e.label;
    return "Other";
}

FileRole fileRoleFromString(std::string_view s)
{
    for (const auto& e : kRoles)
        if (s == e.key)
            return e.role;
    return FileRole::ScanOther;
}

bool isScanRole(FileRole r)
{
    return r == FileRole::ScanUpper || r == FileRole::ScanLower || r == FileRole::ScanBite || r == FileRole::ScanOther;
}

std::string CaseRecord::patientDisplayName() const
{
    if (patientLastName.empty())
        return patientFirstName;
    if (patientFirstName.empty())
        return patientLastName;
    return patientLastName + ", " + patientFirstName;
}

bool CaseRecord::contentDiffers(const CaseRecord& o) const
{
    return caseNumber != o.caseNumber || patientFirstName != o.patientFirstName || patientLastName != o.patientLastName ||
           patientBirthDate != o.patientBirthDate || patientReference != o.patientReference || practice != o.practice ||
           dentist != o.dentist || technician != o.technician || status != o.status || dueDate != o.dueDate || notes != o.notes ||
           workflow != o.workflow || restorations != o.restorations || files != o.files;
}

const CaseFile* CaseRecord::firstFile(FileRole role) const
{
    for (const auto& f : files)
        if (f.role == role)
            return &f;
    return nullptr;
}

} // namespace occlusa::db
