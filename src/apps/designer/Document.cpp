#include "apps/designer/Document.h"

#include "core/Time.h"

#include <json.hpp>

#include <algorithm>
#include <cmath>

using nlohmann::json;

namespace occlusa::designer {

ScanObject* Document::findScan(int id)
{
    for (auto& s : scans)
        if (s->id == id)
            return s.get();
    return nullptr;
}

ScanObject& Document::addScan(std::shared_ptr<const Mesh> mesh, std::string source, std::string label, db::FileRole role)
{
    auto s = std::make_unique<ScanObject>();
    s->id = nextScanId_++;
    s->mesh = std::move(mesh);
    s->source = std::move(source);
    s->label = std::move(label);
    s->role = role;
    scans.push_back(std::move(s));
    touch();
    return *scans.back();
}

void Document::removeScan(int id)
{
    prepScans.erase(id);
    scans.erase(std::remove_if(scans.begin(), scans.end(), [&](const auto& s) { return s->id == id; }), scans.end());
    touch();
}

Aabb Document::sceneBounds() const
{
    Aabb box;
    if (volume)
        box.expand(volume->volume->geometry.worldBounds());
    for (const auto& s : scans)
        if (s->visible)
            box.expand(s->worldBounds());
    return box;
}

// ---------------------------------------------------------------------------
// JSON
// ---------------------------------------------------------------------------

namespace {

json matToJson(const glm::dmat4& m)
{
    json a = json::array();
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
            a.push_back(m[c][r]);
    return a;
}

glm::dmat4 matFromJson(const json& a)
{
    glm::dmat4 m(1.0);
    if (!a.is_array() || a.size() != 16)
        return m;
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
            m[c][r] = a[static_cast<std::size_t>(c * 4 + r)].get<double>();
    return m;
}

json vec3ToJson(const glm::vec3& v)
{
    return json::array({v.x, v.y, v.z});
}

glm::vec3 vec3FromJson(const json& a, glm::vec3 fallback)
{
    if (!a.is_array() || a.size() != 3)
        return fallback;
    return {a[0].get<float>(), a[1].get<float>(), a[2].get<float>()};
}

const char* modeKey(gfx::VolumeMode m)
{
    switch (m) {
    case gfx::VolumeMode::MaximumIntensity: return "mip";
    case gfx::VolumeMode::DirectVolume: return "dvr";
    default: return "isosurface";
    }
}

gfx::VolumeMode modeFromKey(const std::string& k)
{
    if (k == "mip")
        return gfx::VolumeMode::MaximumIntensity;
    if (k == "dvr")
        return gfx::VolumeMode::DirectVolume;
    return gfx::VolumeMode::Isosurface;
}

json dvec3ToJson(const glm::dvec3& v)
{
    return json::array({v.x, v.y, v.z});
}

std::optional<glm::dvec3> dvec3FromJson(const json& j, const char* key)
{
    if (!j.contains(key) || !j[key].is_array() || j[key].size() != 3)
        return std::nullopt;
    const json& a = j[key];
    return glm::dvec3(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
}

json paramsToJson(const crown::CrownParameters& p)
{
    return {{"library", p.library},
            {"kind", static_cast<int>(p.kind)},
            {"upper", p.upper},
            {"coping", p.coping},
            {"copingThickness", p.copingThickness},
            {"cementGap", p.cementGap},
            {"extraGap", p.extraGap},
            {"distanceToMargin", p.distanceToMargin},
            {"blockOutUndercuts", p.blockOutUndercuts},
            {"minThickness", p.minThickness},
            {"marginThickness", p.marginThickness},
            {"crownHeight", p.crownHeight},
            {"halfMesial", p.halfMesial},
            {"halfDistal", p.halfDistal},
            {"halfBuccal", p.halfBuccal},
            {"halfLingual", p.halfLingual},
            {"rotationDeg", p.rotationDeg},
            {"shiftMesial", p.shiftMesial},
            {"shiftBuccal", p.shiftBuccal},
            {"cuspScale", p.cuspScale},
            {"flipMesioDistal", p.flipMesioDistal},
            {"flipBuccoLingual", p.flipBuccoLingual},
            {"ridgeOffset", p.ridgeOffset},
            {"proximalTarget", p.proximalTarget},
            {"occlusalTarget", p.occlusalTarget}};
}

crown::CrownParameters paramsFromJson(const json& j)
{
    crown::CrownParameters p;
    p.library = j.value("library", p.library);
    p.kind = static_cast<crown::ToothKind>(std::clamp(j.value("kind", static_cast<int>(p.kind)), 0, 6));
    p.upper = j.value("upper", p.upper);
    p.coping = j.value("coping", p.coping);
    p.copingThickness = j.value("copingThickness", p.copingThickness);
    p.cementGap = j.value("cementGap", p.cementGap);
    p.extraGap = j.value("extraGap", p.extraGap);
    p.distanceToMargin = j.value("distanceToMargin", p.distanceToMargin);
    p.blockOutUndercuts = j.value("blockOutUndercuts", p.blockOutUndercuts);
    p.minThickness = j.value("minThickness", p.minThickness);
    p.marginThickness = j.value("marginThickness", p.marginThickness);
    p.crownHeight = j.value("crownHeight", p.crownHeight);
    p.halfMesial = j.value("halfMesial", p.halfMesial);
    p.halfDistal = j.value("halfDistal", p.halfDistal);
    p.halfBuccal = j.value("halfBuccal", p.halfBuccal);
    p.halfLingual = j.value("halfLingual", p.halfLingual);
    p.rotationDeg = j.value("rotationDeg", p.rotationDeg);
    p.shiftMesial = j.value("shiftMesial", p.shiftMesial);
    p.shiftBuccal = j.value("shiftBuccal", p.shiftBuccal);
    p.cuspScale = j.value("cuspScale", p.cuspScale);
    p.flipMesioDistal = j.value("flipMesioDistal", p.flipMesioDistal);
    p.flipBuccoLingual = j.value("flipBuccoLingual", p.flipBuccoLingual);
    p.ridgeOffset = j.value("ridgeOffset", p.ridgeOffset);
    p.proximalTarget = j.value("proximalTarget", p.proximalTarget);
    p.occlusalTarget = j.value("occlusalTarget", p.occlusalTarget);
    return p;
}

// Teeth are written in the file's numbering and read back to FDI.
int toFile(int fdi, dental::Numbering n)
{
    return n == dental::Numbering::Universal ? dental::fdiToUniversal(fdi) : fdi;
}

int fromFile(int number, dental::Numbering n)
{
    return n == dental::Numbering::Universal ? dental::universalToFdi(number) : number;
}

json restorationToJson(const SavedRestoration& r, dental::Numbering n)
{
    json o;
    o["tooth"] = toFile(r.tooth, n);
    o["type"] = r.type;
    o["prepScan"] = r.prepScanSource;
    o["antagonist"] = r.antagonistSource;
    o["controls"] = r.controls;
    o["margin"] = r.margin;
    o["marginClosed"] = r.marginClosed;
    o["prepVertexCount"] = r.prepVertexCount;
    if (r.prepPoint)
        o["prepPoint"] = dvec3ToJson(*r.prepPoint);
    if (r.insertionAxis)
        o["insertionAxis"] = dvec3ToJson(*r.insertionAxis);
    o["params"] = paramsToJson(r.params);
    if (r.orientation)
        o["orientation"] = {{"origin", dvec3ToJson(r.orientation->origin)},
                            {"axis", dvec3ToJson(r.orientation->axis)},
                            {"mesial", dvec3ToJson(r.orientation->mesial)},
                            {"buccal", dvec3ToJson(r.orientation->buccal)}};
    // Displacement in units of 0.1 micrometre keeps the JSON compact.
    std::vector<std::int32_t> disp;
    disp.reserve(r.displacement.size());
    for (float d : r.displacement)
        disp.push_back(static_cast<std::int32_t>(std::lround(d * 1e4f)));
    o["displacement"] = disp;
    o["crownGenerated"] = r.crownGenerated;
    if (!r.crownFile.empty())
        o["crownFile"] = r.crownFile;
    return o;
}

SavedRestoration restorationFromJson(const json& o, dental::Numbering n)
{
    SavedRestoration r;
    r.tooth = fromFile(o.value("tooth", 0), n);
    r.type = o.value("type", "");
    r.prepScanSource = o.value("prepScan", "");
    r.antagonistSource = o.value("antagonist", "");
    if (o.contains("controls"))
        r.controls = o["controls"].get<std::vector<std::uint32_t>>();
    if (o.contains("margin"))
        r.margin = o["margin"].get<std::vector<std::uint32_t>>();
    r.marginClosed = o.value("marginClosed", false);
    r.prepVertexCount = o.value("prepVertexCount", std::size_t{0});
    r.prepPoint = dvec3FromJson(o, "prepPoint");
    r.insertionAxis = dvec3FromJson(o, "insertionAxis");
    if (o.contains("params"))
        r.params = paramsFromJson(o["params"]);
    if (o.contains("orientation")) {
        const json& f = o["orientation"];
        crown::CrownFrame fr;
        fr.origin = dvec3FromJson(f, "origin").value_or(fr.origin);
        fr.axis = dvec3FromJson(f, "axis").value_or(fr.axis);
        fr.mesial = dvec3FromJson(f, "mesial").value_or(fr.mesial);
        fr.buccal = dvec3FromJson(f, "buccal").value_or(fr.buccal);
        r.orientation = fr;
    }
    if (o.contains("displacement"))
        for (std::int32_t d : o["displacement"].get<std::vector<std::int32_t>>())
            r.displacement.push_back(static_cast<float>(d) * 1e-4f);
    r.crownGenerated = o.value("crownGenerated", false);
    r.crownFile = o.value("crownFile", "");
    return r;
}

} // namespace

std::string DesignState::toJson() const
{
    json j;
    j["format"] = "occlusacad.design";
    j["version"] = 1;
    j["savedUtc"] = time::nowUtcIso8601();
    j["workflow"] = workflow;
    j["toothNumbering"] = numbering == dental::Numbering::Universal ? "universal" : "fdi";
    j["currentStep"] = currentStep;
    j["completedSteps"] = completedSteps;
    j["cursor"] = json::array({cursor.x, cursor.y, cursor.z});
    if (volumeSource) {
        json v;
        v["source"] = *volumeSource;
        if (volumeDisplay) {
            const auto& d = *volumeDisplay;
            v["window"] = json::array({d.windowCenter, d.windowWidth});
            v["iso"] = d.isoValue;
            v["mode"] = modeKey(d.mode);
            v["color"] = vec3ToJson(d.color);
            v["opacity"] = d.opacity;
            v["cropMin"] = vec3ToJson(d.cropMin);
            v["cropMax"] = vec3ToJson(d.cropMax);
            v["visible"] = d.visible;
        }
        j["volume"] = v;
    }
    json scansJson = json::array();
    for (const auto& s : scans) {
        json o;
        o["source"] = s.source;
        o["label"] = s.label;
        o["role"] = std::string(db::toString(s.role));
        o["transform"] = matToJson(s.transform);
        o["color"] = vec3ToJson(s.color);
        o["opacity"] = s.opacity;
        o["visible"] = s.visible;
        o["registration"] = {{"registered", s.registration.registered},
                             {"method", s.registration.method},
                             {"landmarkRms", s.registration.landmarkRms},
                             {"surfaceRms", s.registration.surfaceRms},
                             {"fractionWithinTolerance", s.registration.fractionWithinTolerance}};
        scansJson.push_back(o);
    }
    j["scans"] = scansJson;
    if (!restorations.empty()) {
        json rj = json::array();
        for (const auto& r : restorations)
            rj.push_back(restorationToJson(r, numbering));
        j["restorations"] = rj;
    }
    if (!bridges.empty()) {
        json bj = json::array();
        for (const auto& b : bridges) {
            std::vector<int> teeth;
            for (int t : b.teeth)
                teeth.push_back(toFile(t, numbering));
            json o = {{"teeth", teeth}, {"connectorArea", b.connectorArea}, {"connectorHeightRatio", b.connectorHeightRatio}, {"embrasure", b.embrasure}};
            if (b.axis)
                o["axis"] = dvec3ToJson(*b.axis);
            json edits = json::array();
            for (const auto& e : b.edits)
                edits.push_back({{"offset", dvec3ToJson(e.offset)}, {"area", e.area}, {"heightRatio", e.heightRatio}, {"length", e.length}});
            o["connectorEdits"] = edits;
            bj.push_back(o);
        }
        j["bridges"] = bj;
    }
    return j.dump(1);
}

DesignState DesignState::fromJson(const std::string& text)
{
    DesignState st;
    const json j = json::parse(text);
    st.workflow = j.value("workflow", "");
    st.numbering = dental::numberingFromString(j.value("toothNumbering", "fdi")); // older files: FDI
    st.currentStep = j.value("currentStep", "");
    if (j.contains("completedSteps"))
        st.completedSteps = j["completedSteps"].get<std::vector<std::string>>();
    if (j.contains("cursor") && j["cursor"].size() == 3)
        st.cursor = glm::dvec3(j["cursor"][0].get<double>(), j["cursor"][1].get<double>(), j["cursor"][2].get<double>());
    if (j.contains("volume")) {
        const json& v = j["volume"];
        st.volumeSource = v.value("source", "");
        VolumeDisplay d;
        if (v.contains("window") && v["window"].size() == 2) {
            d.windowCenter = v["window"][0].get<double>();
            d.windowWidth = v["window"][1].get<double>();
        }
        d.isoValue = v.value("iso", d.isoValue);
        d.mode = modeFromKey(v.value("mode", "isosurface"));
        d.color = vec3FromJson(v.value("color", json()), d.color);
        d.opacity = v.value("opacity", d.opacity);
        d.cropMin = vec3FromJson(v.value("cropMin", json()), d.cropMin);
        d.cropMax = vec3FromJson(v.value("cropMax", json()), d.cropMax);
        d.visible = v.value("visible", true);
        st.volumeDisplay = d;
    }
    if (j.contains("scans")) {
        for (const auto& o : j["scans"]) {
            SavedScan s;
            s.source = o.value("source", "");
            s.label = o.value("label", "");
            s.role = db::fileRoleFromString(o.value("role", "scan_other"));
            s.transform = matFromJson(o.value("transform", json()));
            s.color = vec3FromJson(o.value("color", json()), s.color);
            s.opacity = o.value("opacity", 1.0f);
            s.visible = o.value("visible", true);
            if (o.contains("registration")) {
                const json& r = o["registration"];
                s.registration.registered = r.value("registered", false);
                s.registration.method = r.value("method", "");
                s.registration.landmarkRms = r.value("landmarkRms", 0.0);
                s.registration.surfaceRms = r.value("surfaceRms", 0.0);
                s.registration.fractionWithinTolerance = r.value("fractionWithinTolerance", 0.0);
            }
            st.scans.push_back(s);
        }
    }
    if (j.contains("restorations"))
        for (const auto& o : j["restorations"])
            st.restorations.push_back(restorationFromJson(o, st.numbering));
    if (j.contains("bridges"))
        for (const auto& o : j["bridges"]) {
            SavedBridge b;
            for (int t : o.value("teeth", std::vector<int>{}))
                b.teeth.push_back(fromFile(t, st.numbering));
            b.connectorArea = o.value("connectorArea", b.connectorArea);
            b.connectorHeightRatio = o.value("connectorHeightRatio", b.connectorHeightRatio);
            b.embrasure = o.value("embrasure", b.embrasure);
            b.axis = dvec3FromJson(o, "axis");
            if (o.contains("connectorEdits"))
                for (const auto& e : o["connectorEdits"]) {
                    crown::ConnectorEdit ce;
                    ce.offset = dvec3FromJson(e, "offset").value_or(glm::dvec3(0.0));
                    ce.area = e.value("area", 0.0);
                    ce.heightRatio = e.value("heightRatio", 0.0);
                    ce.length = e.value("length", 0.0);
                    b.edits.push_back(ce);
                }
            st.bridges.push_back(b);
        }
    return st;
}

} // namespace occlusa::designer
