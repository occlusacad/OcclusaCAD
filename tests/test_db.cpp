#include "TestUtil.h"

#include "core/AppConfig.h"
#include "db/CaseRepository.h"
#include "db/SqliteCaseRepository.h"

#include <doctest.h>

#include <fstream>

using namespace occlusa;
using namespace occlusa::db;

namespace {
CaseRecord sampleCase()
{
    CaseRecord r;
    r.patientFirstName = "Zoë";
    r.patientLastName = "Müller";
    r.patientBirthDate = "1980-02-29";
    r.practice = "Smile Dental";
    r.dentist = "Dr. Ortiz";
    r.workflow = "implant_planning";
    r.restorations.push_back(Restoration{0, 36, "implant_planning", "", "", "Bone level, 4.1 x 10"});
    r.restorations.push_back(Restoration{0, 46, "anatomic_crown", "Zirconia", "A2", ""});
    return r;
}
} // namespace

TEST_CASE("SQLite repository CRUD")
{
    test::TempDir tmp;
    SqliteCaseRepository repo(tmp.path() / "occlusacad.sqlite3", tmp.path() / "cases");

    CaseRecord created = repo.createCase(sampleCase());
    CHECK(!created.uuid.empty());
    CHECK(created.caseNumber.rfind("OC-", 0) == 0);
    CHECK(created.revision == 1);
    CHECK(created.restorations.size() == 2);
    CHECK(created.patientDisplayName() == "Müller, Zoë");

    CaseRecord second = repo.createCase(sampleCase());
    CHECK(second.caseNumber != created.caseNumber);

    auto list = repo.listCases({});
    CHECK(list.size() == 2);
    CHECK(list[0].teeth.find("36") != std::string::npos);

    CaseQuery q;
    q.text = "Müller";
    CHECK(repo.listCases(q).size() == 2);
    q.text = "nobody";
    CHECK(repo.listCases(q).empty());

    // Update + optimistic concurrency.
    CaseRecord a = *repo.loadCase(created.uuid);
    CaseRecord b = *repo.loadCase(created.uuid);
    a.notes = "first edit";
    a.restorations.pop_back();
    repo.updateCase(a);
    CHECK(a.revision == 2);
    b.notes = "conflicting edit";
    CHECK_THROWS_AS(repo.updateCase(b), ConcurrencyError);
    CaseRecord reloaded = *repo.loadCase(created.uuid);
    CHECK(reloaded.notes == "first edit");
    CHECK(reloaded.restorations.size() == 1);

    // Status filter / archive.
    repo.setStatus(second.uuid, CaseStatus::Archived);
    CHECK(repo.listCases({}).size() == 1);
    CaseQuery all;
    all.includeArchived = true;
    CHECK(repo.listCases(all).size() == 2);

    // Design state.
    CHECK_FALSE(repo.loadDesignState(created.uuid).has_value());
    repo.saveDesignState(created.uuid, R"({"a":1})", "tester");
    repo.saveDesignState(created.uuid, R"({"a":2})", "tester");
    CHECK(*repo.loadDesignState(created.uuid) == R"({"a":2})");

    // Delete removes children and folder.
    const auto dir = repo.files().caseDirectory(reloaded);
    CHECK(std::filesystem::exists(dir));
    repo.deleteCase(created.uuid);
    CHECK_FALSE(repo.loadCase(created.uuid).has_value());
    CHECK_FALSE(std::filesystem::exists(dir));
}

TEST_CASE("Case locks")
{
    test::TempDir tmp;
    SqliteCaseRepository repo(tmp.path() / "db.sqlite3", tmp.path() / "cases");
    const auto c = repo.createCase(sampleCase());
    CHECK(repo.acquireLock(c.uuid, "alice@ws1", false).acquired);
    CHECK(repo.acquireLock(c.uuid, "alice@ws1", false).acquired); // re-entrant for the same owner
    const auto denied = repo.acquireLock(c.uuid, "bob@ws2", false);
    CHECK_FALSE(denied.acquired);
    CHECK(denied.holder == "alice@ws1");
    CHECK(repo.acquireLock(c.uuid, "bob@ws2", true).acquired);
    repo.releaseLock(c.uuid, "alice@ws1"); // not the owner any more: no effect
    CHECK(repo.loadCase(c.uuid)->lockedBy == "bob@ws2");
    repo.releaseLock(c.uuid, "bob@ws2");
    CHECK(repo.loadCase(c.uuid)->lockedBy.empty());
}

TEST_CASE("Two connections see each other's changes (shared file)")
{
    test::TempDir tmp;
    SqliteCaseRepository ws1(tmp.path() / "shared.sqlite3", tmp.path() / "cases");
    SqliteCaseRepository ws2(tmp.path() / "shared.sqlite3", tmp.path() / "cases");
    const auto c = ws1.createCase(sampleCase());
    auto fromWs2 = ws2.loadCase(c.uuid);
    REQUIRE(fromWs2.has_value());
    fromWs2->technician = "Sam";
    ws2.updateCase(*fromWs2);
    CHECK(ws1.loadCase(c.uuid)->technician == "Sam");
}

TEST_CASE("Case files are imported relative to the case folder")
{
    test::TempDir tmp;
    SqliteCaseRepository repo(tmp.path() / "db.sqlite3", tmp.path() / "cases");
    CaseRecord c = repo.createCase(sampleCase());

    const auto src = tmp.path() / "upper scan.stl";
    std::ofstream(src) << "solid x\nendsolid x\n";
    const std::string rel = repo.files().importFile(c, src, FileRole::ScanUpper);
    CHECK(rel == "scans/upper scan.stl");
    const std::string rel2 = repo.files().importFile(c, src, FileRole::ScanUpper);
    CHECK(rel2 == "scans/upper scan_2.stl");
    CHECK(std::filesystem::exists(repo.files().resolve(c, rel)));

    const auto dicomDir = tmp.path() / "export" / "CT";
    std::filesystem::create_directories(dicomDir / "sub");
    std::ofstream(dicomDir / "a.dcm") << "x";
    std::ofstream(dicomDir / "sub" / "b.dcm") << "y";
    const std::string relDir = repo.files().importDirectory(c, dicomDir, FileRole::Dicom);
    CHECK(relDir == "dicom/CT");
    CHECK(std::filesystem::exists(repo.files().resolve(c, relDir) / "sub" / "b.dcm"));

    c.files.push_back(CaseFile{0, FileRole::ScanUpper, rel, "Upper", ""});
    c.files.push_back(CaseFile{0, FileRole::Dicom, relDir, "CBCT", ""});
    repo.updateCase(c);
    const auto back = *repo.loadCase(c.uuid);
    REQUIRE(back.files.size() == 2);
    CHECK(back.firstFile(FileRole::Dicom)->relativePath == "dicom/CT");
}

TEST_CASE("Repository factory and config round trip")
{
    test::TempDir tmp;
    AppConfig cfg;
    cfg.dataRoot = tmp.path() / "Lab Share";
    cfg.theme = "dark";
    cfg.save(tmp.path() / "config.json");
    const AppConfig back = AppConfig::load(tmp.path() / "config.json");
    CHECK(back.theme == "dark");
    CHECK(back.dataRoot == cfg.dataRoot);
    CHECK(back.isConfigured());

    auto repo = openRepository(back);
    CHECK(repo->backendName() == "SQLite");
    CHECK(std::filesystem::exists(back.databasePath()));

    AppConfig cloud;
    cloud.databaseBackend = "cloud";
    cloud.cloudEndpoint = "https://example.invalid";
    auto c = openRepository(cloud);
    CHECK_THROWS_AS(c->listCases({}), NotImplementedError);
}
