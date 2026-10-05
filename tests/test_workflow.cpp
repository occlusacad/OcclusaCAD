#include "core/Dental.h"
#include "core/Uuid.h"
#include "core/Workflow.h"

#include <doctest.h>

#include <set>

using namespace occlusa;

TEST_CASE("Workflow selection from restorations")
{
    CHECK(workflow::workflowForRestorations({"anatomic_crown"}) == "crown_bridge");
    CHECK(workflow::workflowForRestorations({"anatomic_crown", "implant_planning"}) == "implant_planning");
    CHECK(workflow::workflowForRestorations({"implant_planning", "surgical_guide"}) == "surgical_guide");
    CHECK(workflow::workflowForRestorations({}) == workflow::defaultWorkflow().key);
}

TEST_CASE("Workflow steps are known and unique")
{
    for (const auto& wf : workflow::allWorkflows()) {
        std::set<workflow::StepId> seen;
        for (auto s : wf.steps) {
            CHECK(seen.insert(s).second);
            CHECK(workflow::stepFromKey(workflow::stepInfo(s).key) == s);
        }
        CHECK(wf.steps.front() == workflow::StepId::LoadData);
        CHECK(wf.steps.back() == workflow::StepId::Review);
    }
    std::size_t grouped = 0;
    for (const auto& g : workflow::stepGroups())
        grouped += g.second.size();
    CHECK(grouped == workflow::allSteps().size());
}

TEST_CASE("Tooth numbering")
{
    CHECK(dental::fdiToUniversal(18) == 1);
    CHECK(dental::fdiToUniversal(11) == 8);
    CHECK(dental::fdiToUniversal(21) == 9);
    CHECK(dental::fdiToUniversal(28) == 16);
    CHECK(dental::fdiToUniversal(38) == 17);
    CHECK(dental::fdiToUniversal(31) == 24);
    CHECK(dental::fdiToUniversal(41) == 25);
    CHECK(dental::fdiToUniversal(48) == 32);
    CHECK(dental::isValidFdi(36));
    CHECK_FALSE(dental::isValidFdi(19));
    CHECK(dental::toothName(36) == "Lower left first molar");
}

TEST_CASE("UUIDs")
{
    const auto a = generateUuid(), b = generateUuid();
    CHECK(a.size() == 36);
    CHECK(a != b);
    CHECK(a[14] == '4');
    const auto uid = generateDicomUid();
    CHECK(uid.rfind("2.25.", 0) == 0);
    CHECK(uid.size() <= 64);
}
