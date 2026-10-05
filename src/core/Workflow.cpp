#include "core/Workflow.h"

#include "core/Dental.h"

#include <algorithm>
#include <stdexcept>

namespace occlusa::workflow {

const std::vector<StepInfo>& allSteps()
{
    static const std::vector<StepInfo> steps = {
        {StepId::LoadData, "load_data", "Load data", "Data", "Load CBCT and surface scans",
         "Load the patient's CBCT (DICOM) and the intraoral or model scans (STL). Data attached to the case in "
         "OcclusaCAD DB is loaded automatically; you can add more files here.",
         true},
        {StepId::VolumeSetup, "volume_setup", "Volume setup", "Data", "Window, threshold and rendering",
         "Adjust the grey-value window for the slice views and the bone/teeth threshold for the 3D view so that "
         "teeth and bone are clearly visible.",
         true},
        {StepId::ScanAlignment, "scan_alignment", "Scan alignment", "Data", "Register scans to the CBCT",
         "Align each surface scan with the CBCT. Pick at least three matching points on the scan (left) and on the "
         "CBCT (right), e.g. cusp tips and incisal edges, then press Align. Refine automatically and verify the "
         "scan outline against the slices. The CBCT never moves; only the scans are transformed.",
         true},
        {StepId::PanoramicCurve, "panoramic_curve", "Panoramic curve", "Planning", "Define the dental arch curve",
         "Place points along the dental arch in the axial view to generate a panoramic reconstruction.", false},
        {StepId::NerveCanal, "nerve_canal", "Nerve canal", "Planning", "Mark the mandibular nerve",
         "Trace the inferior alveolar nerve canal from the mental foramen to the mandibular foramen.", false},
        {StepId::VirtualTeeth, "virtual_teeth", "Virtual teeth", "Planning", "Prosthetic set-up for missing teeth",
         "Place virtual teeth in the gaps so the implant can be planned from the restoration (prosthetically driven).", false},
        {StepId::ImplantPlacement, "implant_placement", "Implant placement", "Planning", "Position implants",
         "Select an implant from the library and position it relative to bone, nerve and virtual tooth.", false},
        {StepId::SleeveSetup, "sleeve_setup", "Sleeve setup", "Surgical guide", "Guide sleeves and drill offsets",
         "Choose the guided-surgery kit and sleeve offsets for each planned implant.", false},
        {StepId::GuideDesign, "guide_design", "Guide design", "Surgical guide", "Design the surgical guide",
         "Define the guide outline, thickness, inspection windows and connectors.", false},
        {StepId::ScanBodyAlignment, "scan_body_alignment", "Scan body alignment", "Restoration", "Match scan bodies",
         "Match library scan bodies to the scanned scan bodies to find the implant positions.", false},
        {StepId::MarginLine, "margin_line", "Margin line", "Restoration", "Detect the preparation margin",
         "Define the preparation margin of each crown. Click on the top of the preparation to detect the margin "
         "automatically, then correct it in Edit mode if needed, or draw it point by point.",
         true},
        {StepId::InsertionAxis, "insertion_axis", "Insertion axis", "Restoration", "Set the insertion direction",
         "Check the insertion direction of the crown. Undercuts along the axis are shown in yellow to red and are blocked "
         "out on the inside of the crown. Optimise the axis or tilt it by hand.",
         true},
        {StepId::CrownDesign, "crown_design", "Crown design", "Restoration", "Anatomy and contacts",
         "The crown is designed automatically from the tooth library: fitted between the neighbours, to the antagonist and "
         "with adapted contacts. Adjust its shape, contacts and thickness, or sculpt it with the free-form tools.",
         true},
        {StepId::AbutmentDesign, "abutment_design", "Abutment design", "Restoration", "Emergence profile and core",
         "Design the abutment emergence profile, margin and core.", false},
        {StepId::Review, "review", "Review & save", "Finish", "Verify and save the design",
         "Review the result, then save the design to the case. Registered scans are exported in CBCT coordinates.", true},
    };
    return steps;
}

const StepInfo& stepInfo(StepId id)
{
    for (const auto& s : allSteps())
        if (s.id == id)
            return s;
    throw std::logic_error("unknown step id");
}

std::optional<StepId> stepFromKey(std::string_view key)
{
    for (const auto& s : allSteps())
        if (key == s.key)
            return s.id;
    return std::nullopt;
}

const std::vector<WorkflowDef>& allWorkflows()
{
    using enum StepId;
    static const std::vector<WorkflowDef> wfs = {
        {"implant_planning", "Implant planning", "Prosthetically driven implant planning on CBCT with registered scans.",
         {LoadData, VolumeSetup, ScanAlignment, PanoramicCurve, NerveCanal, VirtualTeeth, ImplantPlacement, Review}},
        {"surgical_guide", "Implant planning + surgical guide", "Implant planning followed by tooth/mucosa supported guide design.",
         {LoadData, VolumeSetup, ScanAlignment, PanoramicCurve, NerveCanal, VirtualTeeth, ImplantPlacement, SleeveSetup, GuideDesign, Review}},
        {"custom_abutment", "Custom abutment", "Custom abutment and screw-retained restorations on implants.",
         {LoadData, ScanBodyAlignment, MarginLine, AbutmentDesign, CrownDesign, Review}},
        {"crown_bridge", "Crown & bridge", "Tooth-borne crowns, copings, bridges, inlays and veneers.",
         {LoadData, MarginLine, InsertionAxis, CrownDesign, Review}},
    };
    return wfs;
}

const WorkflowDef* findWorkflow(std::string_view key)
{
    for (const auto& w : allWorkflows())
        if (w.key == key)
            return &w;
    return nullptr;
}

const WorkflowDef& defaultWorkflow()
{
    return allWorkflows().front();
}

std::vector<std::pair<std::string, std::vector<StepId>>> stepGroups()
{
    std::vector<std::pair<std::string, std::vector<StepId>>> groups;
    for (const auto& s : allSteps()) {
        auto it = std::find_if(groups.begin(), groups.end(), [&](const auto& g) { return g.first == s.group; });
        if (it == groups.end()) {
            groups.emplace_back(s.group, std::vector<StepId>{});
            it = groups.end() - 1;
        }
        it->second.push_back(s.id);
    }
    return groups;
}

std::string workflowForRestorations(const std::vector<std::string>& keys)
{
    // Priority order: the most comprehensive planning workflow wins.
    static constexpr const char* kPriority[] = {"surgical_guide", "implant_planning", "custom_abutment", "crown_bridge"};
    for (const char* wf : kPriority) {
        for (const auto& k : keys) {
            const auto* t = dental::findRestorationType(k);
            if (t && std::string_view(t->workflow) == wf)
                return wf;
        }
    }
    return defaultWorkflow().key;
}

} // namespace occlusa::workflow
