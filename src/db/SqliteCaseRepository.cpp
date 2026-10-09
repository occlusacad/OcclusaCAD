#include "db/SqliteCaseRepository.h"

#include "core/Dental.h"
#include "core/Log.h"
#include "core/Platform.h"
#include "core/Time.h"
#include "core/Uuid.h"
#include "core/Workflow.h"

#include <format>

namespace fs = std::filesystem;

namespace occlusa::db {

using sqlite::Statement;
using sqlite::Transaction;

namespace {

constexpr const char* kSchemaV1 = R"SQL(
CREATE TABLE IF NOT EXISTS cases (
    id                INTEGER PRIMARY KEY,
    uuid              TEXT NOT NULL UNIQUE,
    case_number       TEXT NOT NULL UNIQUE,
    folder            TEXT NOT NULL,
    patient_first     TEXT NOT NULL DEFAULT '',
    patient_last      TEXT NOT NULL DEFAULT '',
    patient_birth     TEXT NOT NULL DEFAULT '',
    patient_ref       TEXT NOT NULL DEFAULT '',
    practice          TEXT NOT NULL DEFAULT '',
    dentist           TEXT NOT NULL DEFAULT '',
    technician        TEXT NOT NULL DEFAULT '',
    status            TEXT NOT NULL DEFAULT 'new',
    due_date          TEXT NOT NULL DEFAULT '',
    notes             TEXT NOT NULL DEFAULT '',
    workflow          TEXT NOT NULL DEFAULT '',
    created_utc       TEXT NOT NULL,
    modified_utc      TEXT NOT NULL,
    revision          INTEGER NOT NULL DEFAULT 1,
    locked_by         TEXT NOT NULL DEFAULT '',
    locked_utc        TEXT NOT NULL DEFAULT ''
);
CREATE INDEX IF NOT EXISTS idx_cases_modified ON cases(modified_utc);
CREATE INDEX IF NOT EXISTS idx_cases_status ON cases(status);

CREATE TABLE IF NOT EXISTS restorations (
    id              INTEGER PRIMARY KEY,
    case_id         INTEGER NOT NULL REFERENCES cases(id) ON DELETE CASCADE,
    tooth           INTEGER NOT NULL,
    type            TEXT NOT NULL,
    material        TEXT NOT NULL DEFAULT '',
    shade           TEXT NOT NULL DEFAULT '',
    notes           TEXT NOT NULL DEFAULT ''
);
CREATE INDEX IF NOT EXISTS idx_restorations_case ON restorations(case_id);

CREATE TABLE IF NOT EXISTS case_files (
    id              INTEGER PRIMARY KEY,
    case_id         INTEGER NOT NULL REFERENCES cases(id) ON DELETE CASCADE,
    role            TEXT NOT NULL,
    rel_path        TEXT NOT NULL,
    label           TEXT NOT NULL DEFAULT '',
    added_utc       TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_case_files_case ON case_files(case_id);

CREATE TABLE IF NOT EXISTS design_state (
    case_id         INTEGER PRIMARY KEY REFERENCES cases(id) ON DELETE CASCADE,
    json            TEXT NOT NULL,
    modified_utc    TEXT NOT NULL,
    modified_by     TEXT NOT NULL DEFAULT ''
);

CREATE TABLE IF NOT EXISTS counters (
    name            TEXT PRIMARY KEY,
    value           INTEGER NOT NULL
);
)SQL";

CaseSummary summaryFromRow(const Statement& st)
{
    CaseSummary s;
    s.uuid = st.getText(0);
    s.caseNumber = st.getText(1);
    const std::string first = st.getText(2), last = st.getText(3);
    s.patient = last.empty() ? first : (first.empty() ? last : last + ", " + first);
    s.practice = st.getText(4);
    s.technician = st.getText(5);
    s.status = caseStatusFromString(st.getText(6));
    s.workflow = st.getText(7);
    s.dueDate = st.getText(8);
    s.modifiedUtc = st.getText(9);
    s.lockedBy = st.getText(10);
    s.teeth = st.getText(11);
    return s;
}

} // namespace

SqliteCaseRepository::SqliteCaseRepository(const fs::path& databaseFile, const fs::path& casesRoot)
    : path_(databaseFile), files_(casesRoot)
{
    std::error_code ec;
    fs::create_directories(databaseFile.parent_path(), ec);
    fs::create_directories(casesRoot, ec);
    db_.open(databaseFile);
    // Network-share friendly settings (see class comment).
    db_.exec("PRAGMA journal_mode=DELETE");
    db_.exec("PRAGMA synchronous=FULL");
    db_.exec("PRAGMA foreign_keys=ON");
    migrate();
    log::info("Case database: {}", platform::pathToUtf8(databaseFile));
}

std::string SqliteCaseRepository::location() const
{
    return platform::pathToUtf8(path_);
}

void SqliteCaseRepository::migrate()
{
    std::lock_guard lock(mutex_);
    const int version = db_.userVersion();
    if (version > kSchemaVersion)
        throw DbError(std::format("The case database was created by a newer OcclusaCAD version (schema {} > {}). Please update.", version,
                                  kSchemaVersion));
    if (version == kSchemaVersion)
        return;
    Transaction tx(db_);
    if (version < 1) {
        db_.exec(kSchemaV1);
    }
    // Future migrations: if (version < 2) { ... }
    db_.setUserVersion(kSchemaVersion);
    tx.commit();
    log::info("Case database schema migrated from v{} to v{}", version, kSchemaVersion);
}

std::vector<CaseSummary> SqliteCaseRepository::listCases(const CaseQuery& q)
{
    std::lock_guard lock(mutex_);
    std::string sql = R"SQL(
        SELECT c.uuid, c.case_number, c.patient_first, c.patient_last, c.practice, c.technician, c.status, c.workflow,
               c.due_date, c.modified_utc, c.locked_by,
               COALESCE((SELECT group_concat(tooth, ', ') FROM (SELECT DISTINCT tooth FROM restorations r
                         WHERE r.case_id = c.id ORDER BY tooth)), '')
        FROM cases c WHERE 1=1)SQL";
    if (!q.text.empty())
        sql += " AND (c.case_number LIKE ?1 OR c.patient_first LIKE ?1 OR c.patient_last LIKE ?1 OR c.practice LIKE ?1"
               " OR c.dentist LIKE ?1 OR c.patient_ref LIKE ?1 OR c.technician LIKE ?1)";
    if (q.status)
        sql += " AND c.status = ?2";
    else if (!q.includeArchived)
        sql += " AND c.status <> 'archived'";
    sql += " ORDER BY c.modified_utc DESC LIMIT ?3";

    auto st = db_.prepare(sql);
    if (!q.text.empty())
        st.bind(1, "%" + q.text + "%");
    if (q.status)
        st.bind(2, std::string(toString(*q.status)));
    st.bind(3, static_cast<std::int64_t>(q.limit));
    std::vector<CaseSummary> out;
    while (st.step())
        out.push_back(summaryFromRow(st));
    return out;
}

std::int64_t SqliteCaseRepository::caseIdForUuid(const std::string& uuid)
{
    auto st = db_.prepare("SELECT id FROM cases WHERE uuid = ?");
    st.bind(1, uuid);
    if (!st.step())
        throw DbError("Case not found: " + uuid);
    return st.getInt(0);
}

std::optional<CaseRecord> SqliteCaseRepository::loadCase(const std::string& uuid)
{
    std::lock_guard lock(mutex_);
    auto st = db_.prepare(R"SQL(
        SELECT id, uuid, case_number, folder, patient_first, patient_last, patient_birth, patient_ref, practice, dentist,
               technician, status, due_date, notes, workflow, created_utc, modified_utc, revision, locked_by, locked_utc
        FROM cases WHERE uuid = ?)SQL");
    st.bind(1, uuid);
    if (!st.step())
        return std::nullopt;
    CaseRecord r;
    r.id = st.getInt(0);
    r.uuid = st.getText(1);
    r.caseNumber = st.getText(2);
    r.folder = st.getText(3);
    r.patientFirstName = st.getText(4);
    r.patientLastName = st.getText(5);
    r.patientBirthDate = st.getText(6);
    r.patientReference = st.getText(7);
    r.practice = st.getText(8);
    r.dentist = st.getText(9);
    r.technician = st.getText(10);
    r.status = caseStatusFromString(st.getText(11));
    r.dueDate = st.getText(12);
    r.notes = st.getText(13);
    r.workflow = st.getText(14);
    r.createdUtc = st.getText(15);
    r.modifiedUtc = st.getText(16);
    r.revision = st.getInt(17);
    r.lockedBy = st.getText(18);
    r.lockedUtc = st.getText(19);

    auto rs = db_.prepare("SELECT id, tooth, type, material, shade, notes FROM restorations WHERE case_id = ? ORDER BY tooth, id");
    rs.bind(1, r.id);
    while (rs.step()) {
        Restoration x;
        x.id = rs.getInt(0);
        x.tooth = static_cast<int>(rs.getInt(1));
        x.type = rs.getText(2);
        x.material = rs.getText(3);
        x.shade = rs.getText(4);
        x.notes = rs.getText(5);
        r.restorations.push_back(std::move(x));
    }
    auto fs_ = db_.prepare("SELECT id, role, rel_path, label, added_utc FROM case_files WHERE case_id = ? ORDER BY id");
    fs_.bind(1, r.id);
    while (fs_.step()) {
        CaseFile f;
        f.id = fs_.getInt(0);
        f.role = fileRoleFromString(fs_.getText(1));
        f.relativePath = fs_.getText(2);
        f.label = fs_.getText(3);
        f.addedUtc = fs_.getText(4);
        r.files.push_back(std::move(f));
    }
    return r;
}

std::string SqliteCaseRepository::allocateCaseNumber()
{
    const std::string year = time::todayLocalDate().substr(0, 4);
    const std::string counter = "case_number_" + year;
    auto get = db_.prepare("SELECT value FROM counters WHERE name = ?");
    get.bind(1, counter);
    std::int64_t next = 1;
    if (get.step())
        next = get.getInt(0) + 1;
    // Skip numbers that already exist (e.g. manually entered).
    for (;;) {
        const std::string candidate = std::format("OC-{}-{:05}", year, next);
        auto exists = db_.prepare("SELECT 1 FROM cases WHERE case_number = ?");
        exists.bind(1, candidate);
        if (!exists.step())
            break;
        ++next;
    }
    auto put = db_.prepare("INSERT INTO counters(name, value) VALUES(?1, ?2) ON CONFLICT(name) DO UPDATE SET value = ?2");
    put.bind(1, counter).bind(2, next).run();
    return std::format("OC-{}-{:05}", year, next);
}

std::string SqliteCaseRepository::nextCaseNumber()
{
    std::lock_guard lock(mutex_);
    // Preview only: does not reserve the number.
    const std::string year = time::todayLocalDate().substr(0, 4);
    auto get = db_.prepare("SELECT value FROM counters WHERE name = ?");
    get.bind(1, "case_number_" + year);
    std::int64_t next = 1;
    if (get.step())
        next = get.getInt(0) + 1;
    return std::format("OC-{}-{:05}", year, next);
}

void SqliteCaseRepository::writeChildren(std::int64_t caseId, const CaseRecord& r)
{
    db_.prepare("DELETE FROM restorations WHERE case_id = ?").bind(1, caseId).run();
    auto ins = db_.prepare("INSERT INTO restorations(case_id, tooth, type, material, shade, notes) VALUES(?,?,?,?,?,?)");
    for (const auto& x : r.restorations) {
        if (!dental::isValidFdi(x.tooth))
            throw DbError(std::format("Invalid tooth number {}", x.tooth));
        ins.reset();
        ins.bind(1, caseId).bind(2, x.tooth).bind(3, x.type).bind(4, x.material).bind(5, x.shade).bind(6, x.notes).run();
    }
    db_.prepare("DELETE FROM case_files WHERE case_id = ?").bind(1, caseId).run();
    auto insF = db_.prepare("INSERT INTO case_files(case_id, role, rel_path, label, added_utc) VALUES(?,?,?,?,?)");
    for (const auto& f : r.files) {
        insF.reset();
        insF.bind(1, caseId)
            .bind(2, std::string(toString(f.role)))
            .bind(3, f.relativePath)
            .bind(4, f.label)
            .bind(5, f.addedUtc.empty() ? time::nowUtcIso8601() : f.addedUtc)
            .run();
    }
}

CaseRecord SqliteCaseRepository::createCase(CaseRecord r)
{
    std::lock_guard lock(mutex_);
    Transaction tx(db_);
    r.uuid = generateUuid();
    if (r.caseNumber.empty())
        r.caseNumber = allocateCaseNumber();
    r.folder = LocalFileStore::makeFolderName(r.caseNumber, r.uuid);
    if (r.workflow.empty()) {
        std::vector<std::string> keys;
        for (const auto& x : r.restorations)
            keys.push_back(x.type);
        r.workflow = workflow::workflowForRestorations(keys);
    }
    r.createdUtc = r.modifiedUtc = time::nowUtcIso8601();
    r.revision = 1;
    auto st = db_.prepare(R"SQL(
        INSERT INTO cases(uuid, case_number, folder, patient_first, patient_last, patient_birth, patient_ref, practice, dentist,
                          technician, status, due_date, notes, workflow, created_utc, modified_utc, revision)
        VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,1))SQL");
    st.bind(1, r.uuid)
        .bind(2, r.caseNumber)
        .bind(3, r.folder)
        .bind(4, r.patientFirstName)
        .bind(5, r.patientLastName)
        .bind(6, r.patientBirthDate)
        .bind(7, r.patientReference)
        .bind(8, r.practice)
        .bind(9, r.dentist)
        .bind(10, r.technician)
        .bind(11, std::string(toString(r.status)))
        .bind(12, r.dueDate)
        .bind(13, r.notes)
        .bind(14, r.workflow)
        .bind(15, r.createdUtc)
        .bind(16, r.modifiedUtc);
    try {
        st.run();
    } catch (const DbError& e) {
        if (std::string(e.what()).find("UNIQUE") != std::string::npos)
            throw DbError("Case number '" + r.caseNumber + "' already exists");
        throw;
    }
    r.id = db_.lastInsertId();
    writeChildren(r.id, r);
    tx.commit();
    log::info("Created case {} ({})", r.caseNumber, r.uuid);
    return *loadCase(r.uuid);
}

void SqliteCaseRepository::updateCase(CaseRecord& r)
{
    std::lock_guard lock(mutex_);
    Transaction tx(db_);
    const std::string now = time::nowUtcIso8601();
    auto st = db_.prepare(R"SQL(
        UPDATE cases SET case_number=?, patient_first=?, patient_last=?, patient_birth=?, patient_ref=?, practice=?, dentist=?,
                         technician=?, status=?, due_date=?, notes=?, workflow=?, modified_utc=?, revision=revision+1
        WHERE uuid=? AND revision=?)SQL");
    st.bind(1, r.caseNumber)
        .bind(2, r.patientFirstName)
        .bind(3, r.patientLastName)
        .bind(4, r.patientBirthDate)
        .bind(5, r.patientReference)
        .bind(6, r.practice)
        .bind(7, r.dentist)
        .bind(8, r.technician)
        .bind(9, std::string(toString(r.status)))
        .bind(10, r.dueDate)
        .bind(11, r.notes)
        .bind(12, r.workflow)
        .bind(13, now)
        .bind(14, r.uuid)
        .bind(15, r.revision);
    try {
        st.run();
    } catch (const DbError& e) {
        if (std::string(e.what()).find("UNIQUE") != std::string::npos)
            throw DbError("Case number '" + r.caseNumber + "' already exists");
        throw;
    }
    if (db_.changes() == 0) {
        auto chk = db_.prepare("SELECT revision FROM cases WHERE uuid = ?");
        chk.bind(1, r.uuid);
        if (!chk.step())
            throw DbError("Case was deleted by another user");
        throw ConcurrencyError("Case " + r.caseNumber + " was modified on another workstation. Reload it and apply your changes again.");
    }
    writeChildren(r.id, r);
    tx.commit();
    r.revision += 1;
    r.modifiedUtc = now;
}

void SqliteCaseRepository::setStatus(const std::string& uuid, CaseStatus status)
{
    std::lock_guard lock(mutex_);
    auto st = db_.prepare("UPDATE cases SET status=?, modified_utc=?, revision=revision+1 WHERE uuid=?");
    st.bind(1, std::string(toString(status))).bind(2, time::nowUtcIso8601()).bind(3, uuid).run();
    if (db_.changes() == 0)
        throw DbError("Case not found: " + uuid);
}

void SqliteCaseRepository::deleteCase(const std::string& uuid)
{
    std::lock_guard lock(mutex_);
    auto rec = loadCase(uuid);
    if (!rec)
        return;
    if (!rec->lockedBy.empty() && rec->lockedBy != currentUserTag())
        throw DbError("Case is open in OcclusaCAD on " + rec->lockedBy);
    {
        Transaction tx(db_);
        db_.prepare("DELETE FROM cases WHERE uuid = ?").bind(1, uuid).run();
        tx.commit();
    }
    // Remove the case folder after the database row is gone (never the other way round).
    std::error_code ec;
    if (!rec->folder.empty())
        fs::remove_all(files_.resolve(*rec, ""), ec);
    log::info("Deleted case {}", rec->caseNumber);
}

std::optional<std::string> SqliteCaseRepository::loadDesignState(const std::string& uuid)
{
    std::lock_guard lock(mutex_);
    auto st = db_.prepare("SELECT d.json FROM design_state d JOIN cases c ON c.id = d.case_id WHERE c.uuid = ?");
    st.bind(1, uuid);
    if (!st.step())
        return std::nullopt;
    return st.getText(0);
}

void SqliteCaseRepository::saveDesignState(const std::string& uuid, const std::string& json, const std::string& modifiedBy)
{
    std::lock_guard lock(mutex_);
    Transaction tx(db_);
    const std::int64_t id = caseIdForUuid(uuid);
    const std::string now = time::nowUtcIso8601();
    db_.prepare(R"SQL(INSERT INTO design_state(case_id, json, modified_utc, modified_by) VALUES(?1, ?2, ?3, ?4)
                      ON CONFLICT(case_id) DO UPDATE SET json=?2, modified_utc=?3, modified_by=?4)SQL")
        .bind(1, id)
        .bind(2, json)
        .bind(3, now)
        .bind(4, modifiedBy)
        .run();
    db_.prepare("UPDATE cases SET modified_utc=? WHERE id=?").bind(1, now).bind(2, id).run();
    tx.commit();
}

LockResult SqliteCaseRepository::acquireLock(const std::string& uuid, const std::string& owner, bool force)
{
    std::lock_guard lock(mutex_);
    Transaction tx(db_);
    auto st = db_.prepare("SELECT locked_by, locked_utc FROM cases WHERE uuid = ?");
    st.bind(1, uuid);
    if (!st.step())
        throw DbError("Case not found: " + uuid);
    LockResult res;
    const std::string holder = st.getText(0), since = st.getText(1);
    if (!holder.empty() && holder != owner && !force) {
        res.holder = holder;
        res.sinceUtc = since;
        return res;
    }
    db_.prepare("UPDATE cases SET locked_by=?, locked_utc=? WHERE uuid=?").bind(1, owner).bind(2, time::nowUtcIso8601()).bind(3, uuid).run();
    tx.commit();
    res.acquired = true;
    return res;
}

void SqliteCaseRepository::releaseLock(const std::string& uuid, const std::string& owner)
{
    std::lock_guard lock(mutex_);
    db_.prepare("UPDATE cases SET locked_by='', locked_utc='' WHERE uuid=? AND locked_by=?").bind(1, uuid).bind(2, owner).run();
}

} // namespace occlusa::db
