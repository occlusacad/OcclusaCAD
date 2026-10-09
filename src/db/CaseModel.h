#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace occlusa::db {

enum class CaseStatus { New, InDesign, Designed, Exported, Archived };

std::string_view toString(CaseStatus s);
std::string_view displayName(CaseStatus s);
CaseStatus caseStatusFromString(std::string_view s);
const std::vector<CaseStatus>& allCaseStatuses();

// Role of a file attached to a case.
enum class FileRole { Dicom, ScanUpper, ScanLower, ScanBite, ScanOther, DesignOutput };

std::string_view toString(FileRole r);
std::string_view displayName(FileRole r);
FileRole fileRoleFromString(std::string_view s);
bool isScanRole(FileRole r);

struct Restoration {
    std::int64_t id = 0;
    int tooth = 0;              // FDI
    std::string type;           // dental::RestorationType::key
    std::string material;
    std::string shade;
    std::string notes;

    bool operator==(const Restoration&) const = default;
};

struct CaseFile {
    std::int64_t id = 0;
    FileRole role = FileRole::ScanOther;
    std::string relativePath;   // relative to the case folder, '/' separated
    std::string label;
    std::string addedUtc;

    bool operator==(const CaseFile&) const = default;
};

struct CaseRecord {
    std::int64_t id = 0;
    std::string uuid;
    std::string caseNumber;
    std::string folder;         // case folder relative to the cases root

    std::string patientFirstName;
    std::string patientLastName;
    std::string patientBirthDate; // YYYY-MM-DD
    std::string patientReference; // chart / practice patient id

    std::string practice;
    std::string dentist;
    std::string technician;

    CaseStatus status = CaseStatus::New;
    std::string dueDate;        // YYYY-MM-DD
    std::string notes;
    std::string workflow;       // workflow::WorkflowDef::key

    std::string createdUtc;
    std::string modifiedUtc;
    std::int64_t revision = 0;  // optimistic concurrency token
    std::string lockedBy;       // "user@host" while open in OcclusaCAD
    std::string lockedUtc;

    std::vector<Restoration> restorations;
    std::vector<CaseFile> files;

    std::string patientDisplayName() const;
    // True when user-editable content differs (ignores timestamps, revision and lock).
    bool contentDiffers(const CaseRecord& other) const;
    const CaseFile* firstFile(FileRole role) const;
};

struct CaseSummary {
    std::string uuid;
    std::string caseNumber;
    std::string patient;
    std::string practice;
    std::string technician;
    CaseStatus status = CaseStatus::New;
    std::string workflow;
    std::string dueDate;
    std::string modifiedUtc;
    std::string teeth;          // e.g. "36, 37"
    std::string lockedBy;
};

struct CaseQuery {
    std::string text;           // matches case number, patient, practice, dentist
    std::optional<CaseStatus> status;
    bool includeArchived = false;
    int limit = 1000;
};

} // namespace occlusa::db
