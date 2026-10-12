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
        {StepId::ScanBodyAlignment, "scan_body_alignment", "Scan body alignment", "Restoration", "Implant system and position",
         "Choose the implant system and platform from the implant library, then click on the top of the scan body in the scan. "
         "The library scan body is matched to it, which gives the position and rotation of the implant.",
         true},
        {StepId::MarginLine, "margin_line", "Margin line", "Restoration", "Detect the preparation margin",
         "Define the preparation margin of each crown. Click on the top of the preparation to detect the margin "
         "automatically, then correct it in Edit mode: drag points, press Delete over a point to remove it, click on the "
         "line to add one. Or draw it point by point.",
         true},
        {StepId::InsertionAxis, "insertion_axis", "Insertion axis", "Restoration", "Set the insertion direction",
         "Check the insertion direction of the crown. Undercuts along the axis are shown in yellow to red and are blocked "
         "out on the inside of the crown. Optimise the axis or tilt it by hand.",
         true},
        {StepId::CrownBottom, "crown_bottom", "Crown bottom", "Restoration", "Cement gap, border, milling",
         "Design the inside of the crown. Gap: the cement gap and the zone along the margin without it, plus extra axial and "
         "radial spacing. Border: the crown edge at the margin (horizontal, angled, vertical). Advanced: undercut block-out and "
         "milling tool compensation. Yellow: cement gap, green: no gap, red: blocked-out undercuts.",
         true},
        {StepId::ToothPlacement, "tooth_placement", "Tooth placement", "Restoration", "Position the library teeth",
         "Position the library tooth: drag it to move, hold Ctrl to rotate, Shift to scale, Ctrl+Shift to scale in one direction. "
         "Auto position fits it between the neighbours and to the antagonist. When it sits right, press Adapt to margin: the "
         "tooth becomes a thin shell attached to the margin, ready for free-forming.",
         true},
        {StepId::Freeforming, "freeforming", "Free-forming", "Restoration", "Wax knife and adaptation",
         "Shape the crown with the wax knife: hold the mouse button to add material (Shift: remove), or smooth (Shift: flatten). "
         "Choose the tool (round ball, pointed knife, texture or your own). Adapt occlusion and contacts to the antagonist and "
         "the neighbours, or cut all intersections.",
         true},
        {StepId::Connectors, "connectors", "Connectors", "Restoration", "Bridge connectors",
         "Set the bridge connectors: cross-section area, height/width, embrasure, and the position and size of each one.", true},
        {StepId::Merge, "merge", "Merge", "Restoration", "Unite crowns with their bottoms",
         "Each crown is united with its crown bottom (cement gap, border, minimum thickness) and the preparation space is "
         "removed; bridges are joined with their connectors. Check the results, then save.",
         true},
        {StepId::AbutmentDesign, "abutment_design", "Abutment design", "Restoration", "Emergence profile and core",
         "Shape the abutment on the library interface: drag the margin points (orange) and the emergence points halfway down "
         "(blue), then set the core the crown sits on. The core follows the margin until you unlock it.",
         true},
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
         {LoadData, ScanBodyAlignment, AbutmentDesign, Review}},
        {"crown_bridge", "Crown & bridge", "Tooth-borne crowns, copings, bridges, inlays and veneers.",
         {LoadData, MarginLine, InsertionAxis, CrownBottom, ToothPlacement, Freeforming, Connectors, Merge, Review}},
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
