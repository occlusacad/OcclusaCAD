// Implant restorations: scan body alignment, the implant library picker and abutment design.
#include "apps/designer/AbutmentSteps.h"

#include "apps/designer/DesignerApp.h"
#include "core/Dental.h"
#include "core/Geometry.h"
#include "core/Log.h"
#include "core/MeshBvh.h"
#include "core/Platform.h"
#include "core/StlIO.h"
#include "ui/FileDialog.h"
#include "ui/Fonts.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"

#include <imgui.h>
#include <imgui_stdlib.h>
#include <json.hpp>

#include <cctype>
#include <cmath>
#include <cstdint>
#include <format>
#include <fstream>
#include <map>
#include <mutex>
#include <set>

namespace occlusa::designer {

using workflow::StepId;

namespace {

const glm::vec3 kScanBodyColor(0.35f, 0.75f, 0.95f);
const glm::vec3 kInterfaceColor(0.60f, 0.64f, 0.70f);
const glm::vec3 kAbutmentColor(0.88f, 0.82f, 0.64f);
constexpr ImU32 kMarginHandle = IM_COL32(255, 150, 40, 255);
constexpr ImU32 kMidHandle = IM_COL32(70, 160, 255, 255);
constexpr ImU32 kCoreHandle = IM_COL32(90, 200, 110, 255);
constexpr ImU32 kHintBorder = IM_COL32(255, 150, 40, 255);

enum Ring { MarginRing = 0, MidRing = 1, CoreRing = 2 };

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

bool isImplantType(const std::string& type)
{
    const auto* t = dental::findRestorationType(type);
    return t && std::string_view(t->workflow) == "custom_abutment";
}

ScanObject* scanOf(DesignerApp& app, const ImplantRestoration& r)
{
    return r.scanId ? app.doc().findScan(r.scanId) : nullptr;
}

ImplantRestoration* findImplant(DesignerApp& app, int tooth)
{
    for (auto& r : app.doc().implants)
        if (r.tooth == tooth)
            return &r;
    return nullptr;
}

std::string key(const char* kind, int tooth)
{
    return std::format("{}:{}", kind, tooth);
}

void assignScan(DesignerApp& app, ImplantRestoration& r)
{
    auto& scans = app.doc().scans;
    if (scans.empty())
        return;
    const db::FileRole own = r.tooth && dental::isUpper(r.tooth) ? db::FileRole::ScanUpper : db::FileRole::ScanLower;
    r.scanId = scans.front()->id;
    for (const auto& s : scans)
        if (s->role == own) {
            r.scanId = s->id;
            break;
        }
}

std::string restorationTypeLabel(const std::string& type)
{
    const auto* t = dental::findRestorationType(type);
    return t ? t->label : type;
}

// Spatial index per scan mesh (gingiva ray casts).
std::shared_ptr<const MeshBvh> scanBvh(const std::shared_ptr<const Mesh>& mesh)
{
    static std::mutex mutex;
    static std::map<const Mesh*, std::pair<std::weak_ptr<const Mesh>, std::shared_ptr<const MeshBvh>>> cache;
    std::lock_guard lock(mutex);
    auto it = cache.find(mesh.get());
    if (it != cache.end() && !it->second.first.expired())
        return it->second.second;
    auto bvh = std::make_shared<const MeshBvh>(mesh);
    cache[mesh.get()] = {mesh, bvh};
    return bvh;
}

// The connection for the restoration's library choice (loaded once per choice).
bool ensureConnection(ImplantRestoration& r)
{
    if (!r.hasConnection()) {
        r.connection.reset();
        r.connectionError.clear();
        r.loadedKey.clear();
        return false;
    }
    const std::string k = r.libraryId + "/" + r.connectionId;
    if (r.loadedKey == k)
        return r.connection != nullptr;
    r.loadedKey = k;
    r.invalidate();
    try {
        r.connection = implant::ImplantLibraryRegistry::instance().connection(r.libraryId, r.connectionId);
        r.connectionError.clear();
        return true;
    } catch (const std::exception& e) {
        r.connection.reset();
        r.connectionError = e.what();
        log::warn("Implant {}: {}", r.tooth, e.what());
        return false;
    }
}

std::string connectionLabel(const ImplantRestoration& r)
{
    if (!r.hasConnection())
        return "No implant chosen";
    if (auto lib = implant::ImplantLibraryRegistry::instance().find(r.libraryId)) {
        if (const auto* c = lib->connection(r.connectionId))
            return std::format("{} {} {}", lib->manufacturer.empty() ? lib->name : lib->manufacturer, c->system, c->name);
        return lib->name + " / " + r.connectionId + " (missing)";
    }
    return r.libraryId + " (not installed)";
}

// Implant frame -> scan coordinates.
Mesh toScan(const Mesh& m, const glm::dmat4& t)
{
    Mesh out;
    out.positions.reserve(m.positions.size());
    for (const auto& p : m.positions)
        out.positions.emplace_back(transformPoint(t, glm::dvec3(p)));
    out.indices = m.indices;
    out.colors = m.colors;
    out.computeVertexNormals();
    return out;
}

double scanBodyRadius(const implant::Connection& c)
{
    double r = 0.0;
    for (const auto& p : c.scanBody->positions)
        if (p.z > c.topHeight + 0.5)
            r = std::max(r, std::hypot(static_cast<double>(p.x), static_cast<double>(p.y)));
    return r;
}

// The scan without the scanned scan bodies (they would hide the abutment): triangles within
// 0.12 mm of a matched library scan body are left out.
Mesh scanWithoutScanBodies(const Mesh& scan, const std::vector<std::pair<glm::dmat4, std::shared_ptr<const implant::Connection>>>& bodies)
{
    std::vector<char> keep(scan.positions.size(), 1);
    for (const auto& [implantToScan, c] : bodies) {
        const MeshBvh bvh(c->scanBody);
        Aabb box = c->scanBody->bounds();
        box.expand(box.min - glm::dvec3(0.3));
        box.expand(box.max + glm::dvec3(0.3));
        const glm::dmat4 toImplant = glm::inverse(implantToScan);
        for (std::size_t v = 0; v < scan.positions.size(); ++v) {
            const glm::dvec3 p = transformPoint(toImplant, glm::dvec3(scan.positions[v]));
            if (glm::any(glm::lessThan(p, box.min)) || glm::any(glm::greaterThan(p, box.max)))
                continue;
            if (auto cp = bvh.closestPoint(glm::vec3(p), 0.12f))
                keep[v] = 0;
        }
    }
    Mesh out;
    std::vector<std::int64_t> remap(scan.positions.size(), -1);
    for (std::size_t t = 0; t + 2 < scan.indices.size(); t += 3) {
        if (!keep[scan.indices[t]] || !keep[scan.indices[t + 1]] || !keep[scan.indices[t + 2]])
            continue;
        for (int k = 0; k < 3; ++k) {
            const std::uint32_t i = scan.indices[t + static_cast<std::size_t>(k)];
            if (remap[i] < 0) {
                remap[i] = static_cast<std::int64_t>(out.positions.size());
                out.positions.push_back(scan.positions[i]);
                if (i < scan.normals.size())
                    out.normals.push_back(scan.normals[i]);
            }
            out.indices.push_back(static_cast<std::uint32_t>(remap[i]));
        }
    }
    if (out.normals.size() != out.positions.size())
        out.computeVertexNormals();
    return out;
}

// Margin heights from the gingiva around the scan body: `depth` mm below the gingiva at each
// control point (ray cast down along the implant axis just outside the scan body).
int fitMarginToGingiva(DesignerApp& app, const ImplantRestoration& r, implant::AbutmentShape& s, double depth)
{
    ScanObject* scan = scanOf(app, r);
    if (!scan || !r.connection || !r.implantToScan)
        return 0;
    const auto bvh = scanBvh(scan->mesh);
    const implant::Connection& c = *r.connection;
    const double sbR = scanBodyRadius(c);
    const glm::dmat4 toScan = *r.implantToScan, toImplant = glm::inverse(toScan);
    const glm::dvec3 down = glm::normalize(transformVector(toScan, glm::dvec3(0, 0, -1)));
    int found = 0;
    for (int k = 0; k < implant::kControlPoints; ++k) {
        const double a = s.azimuth(k);
        const double rs = std::max(s.marginRadius[static_cast<std::size_t>(k)], sbR + 0.35);
        const glm::dvec3 o = transformPoint(toScan, glm::dvec3(rs * std::cos(a), rs * std::sin(a), c.topHeight + 15.0));
        if (auto hit = bvh->raycast(glm::vec3(o), glm::vec3(down), 30.0f)) {
            const double gum = transformPoint(toImplant, glm::dvec3(hit->point)).z;
            s.marginHeight[static_cast<std::size_t>(k)] = std::clamp(gum - depth - c.topHeight, 0.3, 5.0);
            ++found;
        }
    }
    return found;
}

implant::AbutmentShape makeDefaultShape(DesignerApp& app, const ImplantRestoration& r)
{
    implant::AbutmentShape s = implant::defaultAbutmentShape(*r.connection, 1.5);
    fitMarginToGingiva(app, r, s, 0.5);
    return s;
}

bool ensureGeometry(DesignerApp& app, ImplantRestoration& r)
{
    if (!r.designsAbutment() || !ensureConnection(r) || !r.implantToScan)
        return false;
    if (!r.shape)
        r.shape = makeDefaultShape(app, r);
    if (!r.geometry)
        r.geometry = std::make_shared<const implant::AbutmentGeometry>(implant::buildAbutment(*r.shape, *r.connection));
    return true;
}

glm::vec3 wallColor(float wall, float minWall)
{
    if (wall < minWall - 1e-3f)
        return {0.95f, 0.30f, 0.20f};
    if (wall < minWall + 0.25f)
        return {0.98f, 0.80f, 0.20f};
    return kAbutmentColor;
}

// Overlays of one implant for the current step: the scan body while it is matched, the interface
// and the designed abutment afterwards.
void updateImplantOverlays(DesignerApp& app, ImplantRestoration& r, bool wallMap)
{
    Document& doc = app.doc();
    const StepId step = app.currentStep();
    const bool aligning = step == StepId::ScanBodyAlignment;
    ensureConnection(r);
    if (!r.connection || !r.implantToScan || !scanOf(app, r)) {
        doc.removeOverlay(key("scanbody", r.tooth));
        doc.removeOverlay(key("interface", r.tooth));
        doc.removeOverlay(key("abutment", r.tooth));
        return;
    }
    const glm::dmat4 t = *r.implantToScan;
    if (aligning) {
        DisplayMesh dm;
        dm.gpu = std::make_unique<gfx::GpuMesh>(toScan(*r.connection->scanBody, t));
        dm.scanId = r.scanId;
        dm.color = kScanBodyColor;
        dm.opacity = 0.75f;
        doc.setOverlay(key("scanbody", r.tooth), std::move(dm));
    } else {
        doc.removeOverlay(key("scanbody", r.tooth));
    }
    if (aligning || !r.designsAbutment()) {
        doc.removeOverlay(key("interface", r.tooth));
        doc.removeOverlay(key("abutment", r.tooth));
        return;
    }
    {
        DisplayMesh dm;
        dm.gpu = std::make_unique<gfx::GpuMesh>(toScan(*r.connection->interfaceMesh, t));
        dm.scanId = r.scanId;
        dm.color = kInterfaceColor;
        doc.setOverlay(key("interface", r.tooth), std::move(dm));
    }
    if (ensureGeometry(app, r)) {
        Mesh m = r.geometry->designed;
        if (wallMap) {
            const float minWall = static_cast<float>(r.connection->info.minWall);
            m.colors.assign(m.positions.size(), kAbutmentColor);
            for (std::size_t v = 0; v < r.geometry->outerRowEnd; ++v)
                if (r.geometry->wall[v] < 100.0f)
                    m.colors[v] = wallColor(r.geometry->wall[v], minWall);
        }
        DisplayMesh dm;
        dm.gpu = std::make_unique<gfx::GpuMesh>(toScan(m, t));
        dm.scanId = r.scanId;
        dm.color = kAbutmentColor;
        dm.vertexColors = wallMap;
        doc.setOverlay(key("abutment", r.tooth), std::move(dm));
    }
}

bool gWallMap = true;

void refreshImplantOverlays(DesignerApp& app)
{
    for (auto& r : app.doc().implants)
        updateImplantOverlays(app, r, gWallMap);
}

void drawHint(ImDrawList* dl, const Projector& proj, const char* text)
{
    const float s = ImGui::GetStyle().FontScaleDpi;
    const ImVec2 ts = ImGui::CalcTextSize(text);
    const ImVec2 pos((proj.min.x + proj.max.x - ts.x) * 0.5f, proj.max.y - ts.y - 16 * s);
    dl->AddRectFilled(ImVec2(pos.x - 10 * s, pos.y - 5 * s), ImVec2(pos.x + ts.x + 10 * s, pos.y + ts.y + 5 * s), IM_COL32(15, 18, 22, 200), 6 * s);
    dl->AddRect(ImVec2(pos.x - 10 * s, pos.y - 5 * s), ImVec2(pos.x + ts.x + 10 * s, pos.y + ts.y + 5 * s), kHintBorder, 6 * s, 1.5f * s);
    dl->AddText(pos, IM_COL32(240, 242, 245, 255), text);
}

void focusOnImplant(DesignerApp& app, const ImplantRestoration& r)
{
    ScanObject* scan = scanOf(app, r);
    if (!scan || !r.implantToScan)
        return;
    const glm::dmat4 w = scan->transform * *r.implantToScan;
    Aabb box;
    box.expand(transformPoint(w, glm::dvec3(-6, -6, -2)));
    box.expand(transformPoint(w, glm::dvec3(6, 6, 11)));
    app.view3D(ViewId::Main3D).focus(box);
}

// View of the implant from `elevation` (0 = from the side, 1 = along the axis).
void viewImplant(DesignerApp& app, const ImplantRestoration& r, double elevation, double size)
{
    ScanObject* scan = scanOf(app, r);
    if (!scan || !r.implantToScan)
        return;
    const glm::dmat4 w = scan->transform * *r.implantToScan;
    gfx::Camera& cam = app.view3D(ViewId::Main3D).camera;
    const glm::dvec3 axis = glm::normalize(transformVector(w, glm::dvec3(0, 0, 1)));
    const glm::dvec3 side = glm::normalize(transformVector(w, glm::dvec3(0.3, -1.0, 0.0)));
    cam.setView(glm::normalize(-side * (1.0 - elevation) - axis * elevation), elevation > 0.7 ? side : axis);
    Aabb box;
    box.expand(transformPoint(w, glm::dvec3(-size, -size, -2)));
    box.expand(transformPoint(w, glm::dvec3(size, size, 11)));
    app.view3D(ViewId::Main3D).focus(box);
}

// Side view of the implant (the emergence profile and core are seen best from the side).
void viewFromSide(DesignerApp& app, const ImplantRestoration& r)
{
    ScanObject* scan = scanOf(app, r);
    if (!scan || !r.implantToScan)
        return;
    const glm::dmat4 w = scan->transform * *r.implantToScan;
    gfx::Camera& cam = app.view3D(ViewId::Main3D).camera;
    const glm::dvec3 axis = glm::normalize(transformVector(w, glm::dvec3(0, 0, 1)));
    const glm::dvec3 side = glm::normalize(transformVector(w, glm::dvec3(0.3, -1.0, 0.0)));
    cam.setView(glm::normalize(side * -1.0 - axis * 0.45), axis);
    focusOnImplant(app, r);
}

// Copy an implant library folder into the lab's implant libraries (or the user's own without a data folder).
std::optional<std::string> importImplantLibrary(DesignerApp& app, const std::filesystem::path& source)
{
    namespace fs = std::filesystem;
    try {
        const implant::LibraryInfo lib = implant::readImplantLibrary(source);
        // Check the geometry before installing it.
        for (const auto& c : lib.connections) {
            for (const std::string* f : {&c.interfaceFile, &c.scanBodyFile})
                if (readStl(source / platform::pathFromUtf8(*f)).empty())
                    throw std::runtime_error(*f + " is empty.");
        }
        const fs::path root = app.config().dataRoot.empty() ? platform::configDir() / "implant-libraries" : app.config().implantLibrariesRoot();
        const fs::path dest = root / platform::pathFromUtf8(lib.id);
        fs::create_directories(dest);
        fs::copy(source, dest, fs::copy_options::recursive | fs::copy_options::overwrite_existing);
        const std::string id = implant::ImplantLibraryRegistry::instance().addFolder(dest);
        ui::toast(ui::ToastKind::Success, std::format("Implant library \"{}\" imported ({} connections)", lib.name, lib.connections.size()));
        log::info("Implant library {} imported to {}", lib.name, platform::pathToUtf8(dest));
        return id;
    } catch (const std::exception& e) {
        ui::showError("Import implant library", e.what());
        return std::nullopt;
    }
}

// ---------------------------------------------------------------------------
// Implant library picker (exocad style: manufacturer > system > connection, with details)
// ---------------------------------------------------------------------------

class LibraryPicker {
public:
    void open(const ImplantRestoration& r)
    {
        tooth_ = r.tooth;
        library_ = r.libraryId;
        connection_ = r.connectionId;
        search_.clear();
        openRequested_ = true;
    }

    // Returns true when the active implant's connection changed.
    bool draw(DesignerApp& app)
    {
        const char* title = "Choose implant";
        if (openRequested_) {
            ImGui::OpenPopup(title);
            openRequested_ = false;
        }
        const float fs = ImGui::GetFontSize();
        ImGui::SetNextWindowSize(ImVec2(fs * 48, fs * 31), ImGuiCond_Appearing);
        ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        bool changed = false;
        if (!ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_NoSavedSettings))
            return false;
        const ui::Palette& pal = ui::palette();
        ImplantRestoration* r = findImplant(app, tooth_);
        ui::mutedText("Implant for tooth %s", r ? app.toothText(r->tooth).c_str() : "-");
        ImGui::Spacing();
        const float footer = ImGui::GetFrameHeightWithSpacing() + ImGui::GetStyle().ItemSpacing.y;
        const float listW = ImGui::GetContentRegionAvail().x * 0.48f;
        auto libs = implant::ImplantLibraryRegistry::instance().libraries();

        // Left: search + tree.
        ImGui::BeginChild("##tree", ImVec2(listW, -footer), ImGuiChildFlags_Borders);
        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::InputTextWithHint("##search", "Search manufacturer, system, platform", &search_);
        ImGui::Separator();
        auto matches = [&](const implant::LibraryInfo& l, const implant::ConnectionInfo& c) {
            if (search_.empty())
                return true;
            auto lower = [](std::string s) {
                for (char& ch : s)
                    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
                return s;
            };
            const std::string hay = lower(l.manufacturer + " " + l.name + " " + c.system + " " + c.name + " " + std::format("{:.1f}", c.platformDiameter));
            return hay.find(lower(search_)) != std::string::npos;
        };
        std::map<std::string, std::vector<const implant::LibraryInfo*>> byMaker;
        for (const auto& l : libs)
            byMaker[l.manufacturer.empty() ? l.name : l.manufacturer].push_back(&l);
        for (const auto& [maker, list] : byMaker) {
            bool any = false;
            for (const auto* l : list)
                for (const auto& c : l->connections)
                    any |= matches(*l, c);
            if (!any)
                continue;
            ImGui::SetNextItemOpen(true, search_.empty() ? ImGuiCond_Once : ImGuiCond_Always);
            if (!ImGui::TreeNodeEx(maker.c_str(), ImGuiTreeNodeFlags_SpanAvailWidth))
                continue;
            for (const auto* l : list) {
                std::map<std::string, std::vector<const implant::ConnectionInfo*>> bySystem;
                for (const auto& c : l->connections)
                    if (matches(*l, c))
                        bySystem[c.system.empty() ? l->name : c.system].push_back(&c);
                for (const auto& [system, conns] : bySystem) {
                    ImGui::SetNextItemOpen(true, search_.empty() ? ImGuiCond_Once : ImGuiCond_Always);
                    const std::string node = list.size() > 1 ? std::format("{}  ({})", system, l->name) : system;
                    if (!ImGui::TreeNodeEx(std::format("{}##{}", node, l->id).c_str(), ImGuiTreeNodeFlags_SpanAvailWidth))
                        continue;
                    for (const auto* c : conns) {
                        const bool sel = library_ == l->id && connection_ == c->id;
                        const std::string label = std::format("{}   \xC3\x98{:.1f} mm##{}/{}", c->name, c->platformDiameter, l->id, c->id);
                        if (ImGui::Selectable(label.c_str(), sel, ImGuiSelectableFlags_AllowDoubleClick)) {
                            library_ = l->id;
                            connection_ = c->id;
                            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                                changed = apply(app);
                        }
                    }
                    ImGui::TreePop();
                }
            }
            ImGui::TreePop();
        }
        ImGui::EndChild();
        ImGui::SameLine();

        // Right: details of the selection.
        ImGui::BeginChild("##details", ImVec2(0, -footer), ImGuiChildFlags_Borders);
        std::optional<implant::LibraryInfo> lib = implant::ImplantLibraryRegistry::instance().find(library_);
        const implant::ConnectionInfo* ci = lib ? lib->connection(connection_) : nullptr;
        if (!ci) {
            ui::wrappedMutedText("Choose the implant system and platform on the left. The library provides the interface to the "
                                 "implant (milled unchanged) and the scan body used to find the implant in the scan.");
        } else {
            ImGui::PushFont(ui::fonts::semibold(), 0.0f);
            ImGui::TextWrapped("%s %s", ci->system.c_str(), ci->name.c_str());
            ImGui::PopFont();
            ui::mutedText("%s", lib->manufacturer.empty() ? lib->name.c_str() : lib->manufacturer.c_str());
            ImGui::Spacing();
            auto row = [&](const char* label, const std::string& value) {
                ui::mutedText("%s", label);
                ImGui::SameLine(fs * 9.0f);
                ImGui::TextUnformatted(value.c_str());
            };
            row("Library", lib->name);
            row("Platform", std::format("\xC3\x98 {:.2f} mm", ci->platformDiameter));
            std::shared_ptr<const implant::Connection> conn;
            std::string error;
            try {
                conn = implant::ImplantLibraryRegistry::instance().connection(library_, connection_);
            } catch (const std::exception& e) {
                error = e.what();
            }
            if (conn)
                row("Interface top", std::format("\xC3\x98 {:.2f} mm at {:.2f} mm", conn->topRadius * 2.0, conn->topHeight));
            row("Screw channel", std::format("\xC3\x98 {:.2f} mm", ci->screwChannelDiameter));
            row("Minimum wall", std::format("{:.2f} mm", ci->minWall));
            ImGui::Spacing();
            ui::subheading("Geometry");
            auto have = [&](const char* label, bool present, const char* missing) {
                ImGui::TextColored(present ? pal.success : pal.textMuted, present ? "  \xE2\x9C\x93  %s" : "  -  %s", label);
                if (!present && missing) {
                    ImGui::SameLine();
                    ui::mutedText("(%s)", missing);
                }
            };
            have("Interface", true, nullptr);
            have("Scan body", true, nullptr);
            have("Screw channel", !ci->screwChannelFile.empty(), "straight cylinder");
            have("Minimum thickness", !ci->minThicknessFile.empty(), "not checked");
            have("Blank", !ci->blankFile.empty(), "none");
            if (conn && !conn->interfaceClosed)
                ImGui::TextColored(pal.warning, "The interface is not a closed solid.");
            if (!error.empty())
                ImGui::TextColored(pal.danger, "%s", error.c_str());
            ImGui::Spacing();
            if (!lib->author.empty())
                ui::wrappedMutedText(("Author: " + lib->author).c_str());
            if (!lib->license.empty())
                ui::wrappedMutedText(("License: " + lib->license).c_str());
        }
        ImGui::EndChild();

        if (ImGui::Button("Import library...")) {
            if (auto folder = ui::dialogs::pickFolder())
                if (auto id = importImplantLibrary(app, *folder)) {
                    library_ = *id;
                    connection_.clear();
                }
        }
        const float bw = fs * 7;
        ImGui::SameLine();
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x - bw * 2 - ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::Button("Cancel", ImVec2(bw, 0)) || ui::accessKeyPressed(ImGuiKey_Escape))
            ImGui::CloseCurrentPopup();
        ImGui::SameLine();
        if (ui::primaryButton("Select", ImVec2(bw, 0), ci != nullptr && r != nullptr))
            changed = apply(app);
        ImGui::EndPopup();
        return changed;
    }

private:
    bool apply(DesignerApp& app)
    {
        ImGui::CloseCurrentPopup();
        ImplantRestoration* r = findImplant(app, tooth_);
        if (!r || (r->libraryId == library_ && r->connectionId == connection_))
            return false;
        r->libraryId = library_;
        r->connectionId = connection_;
        // The abutment is built on the interface: start again on the new one. The implant position
        // stays if a scan body was matched (the new scan body is checked in the alignment step).
        r->shape.reset();
        r->invalidate();
        ensureConnection(*r);
        app.markModified();
        return true;
    }

    int tooth_ = 0;
    std::string library_, connection_, search_;
    bool openRequested_ = false;
};

LibraryPicker gPicker;

// Implant restoration chooser plus the implant system and scan. Returns true when the active
// restoration (or its connection) changed.
bool drawImplantCard(DesignerApp& app, bool showConnection)
{
    Document& doc = app.doc();
    const ui::Palette& pal = ui::palette();
    bool changed = false;
    ui::beginCard("##implant");
    ui::subheading("Implant restoration");
    if (doc.implants.empty()) {
        ui::wrappedMutedText(app.caseMode() ? "The case has no implant restorations. Add a custom abutment to the case in OcclusaCAD DB."
                                            : "No case is open. Choose the implant position to design.");
        if (!app.caseMode()) {
            static int number = 0;
            const bool universal = app.numbering() == dental::Numbering::Universal;
            if (number == 0)
                number = universal ? 19 : 36;
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4.5f);
            ImGui::InputInt(universal ? "Tooth (Universal)" : "Tooth (FDI)", &number, 0, 0);
            const int fdi = dental::parseTooth(number, app.numbering());
            if (ui::primaryButton("Add custom abutment", ImVec2(-FLT_MIN, 0), dental::isValidFdi(fdi) && !findImplant(app, fdi))) {
                ImplantRestoration r;
                r.tooth = fdi;
                r.type = "custom_abutment";
                assignScan(app, r);
                r.scanAssigned = !doc.scans.empty();
                doc.implants.push_back(std::move(r));
                doc.activeImplant = static_cast<int>(doc.implants.size()) - 1;
                changed = true;
            }
        }
        ui::endCard();
        return changed;
    }
    ImplantRestoration* act = doc.activeImplantRestoration();
    auto label = [&](const ImplantRestoration& r) {
        std::string s = std::format("{}  {}", app.toothText(r.tooth), restorationTypeLabel(r.type));
        if (r.geometry || r.shape)
            s += "  - designed";
        else if (r.implantToScan)
            s += "  - implant found";
        return s;
    };
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginCombo("##implantsel", act ? label(*act).c_str() : "")) {
        for (std::size_t i = 0; i < doc.implants.size(); ++i)
            if (ImGui::Selectable(label(doc.implants[i]).c_str(), static_cast<int>(i) == doc.activeImplant)) {
                doc.activeImplant = static_cast<int>(i);
                changed = true;
            }
        ImGui::EndCombo();
    }
    act = doc.activeImplantRestoration();
    if (act) {
        ui::mutedText("%s", dental::toothName(act->tooth).c_str());
        if (!act->designsAbutment()) {
            ImGui::Spacing();
            ImGui::TextColored(pal.warning, "Only the implant position is found for this type; its crown is not designed in this version.");
        }
        if (showConnection) {
            ImGui::Spacing();
            ensureConnection(*act);
            ImGui::TextWrapped("%s", connectionLabel(*act).c_str());
            if (!act->connectionError.empty())
                ImGui::TextColored(pal.danger, "%s", act->connectionError.c_str());
            const bool none = !act->hasConnection();
            if (none ? ui::primaryButton("Choose implant...", ImVec2(-FLT_MIN, 0)) : ui::button("Change implant...", ImVec2(-FLT_MIN, 0)))
                gPicker.open(*act);
        }
        if (!doc.scans.empty()) {
            ImGui::Spacing();
            ScanObject* cur = scanOf(app, *act);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted("Scan");
            ImGui::SameLine(ImGui::GetFontSize() * 4.0f);
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::BeginCombo("##implantscan", cur ? cur->label.c_str() : "None")) {
                for (const auto& s : doc.scans)
                    if (ImGui::Selectable(s->label.c_str(), s->id == act->scanId) && s->id != act->scanId) {
                        act->scanId = s->id;
                        act->implantToScan.reset(); // the position belongs to the scan
                        act->invalidate();
                        app.markModified();
                        changed = true;
                    }
                ImGui::EndCombo();
            }
        }
    }
    ui::endCard();
    if (gPicker.draw(app))
        changed = true;
    if (changed)
        refreshImplantOverlays(app);
    return changed;
}

void startScanBodyFit(DesignerApp& app, int tooth, const glm::dvec3& click)
{
    ImplantRestoration* r = findImplant(app, tooth);
    ScanObject* scan = r ? scanOf(app, *r) : nullptr;
    if (!r || !scan || !ensureConnection(*r))
        return;
    std::shared_ptr<const Mesh> mesh = scan->mesh;
    std::shared_ptr<const Mesh> body = r->connection->scanBody;
    app.tasks().start("Matching the scan body", [&app, tooth, mesh, body, click](const ProgressFn& progress) -> ui::TaskRunner::Continuation {
        auto fit = implant::fitScanBody(*mesh, *body, click, progress);
        return [&app, tooth, fit] {
            ImplantRestoration* r = findImplant(app, tooth);
            if (!r)
                return;
            if (!fit || !fit->ok || fit->inlierFraction < 0.3) {
                ui::toast(ui::ToastKind::Warning, "No scan body found there. Click on the top of the scan body.");
                return;
            }
            r->implantToScan = fit->implantToScan;
            r->fit = *fit;
            r->invalidate();
            log::info("Scan body {} matched: {:.3f} mm RMS, {:.0f}% within 0.05 mm", app.toothText(tooth), fit->rms, fit->fractionWithin * 100.0);
            if (fit->fractionWithin < 0.8)
                ui::toast(ui::ToastKind::Warning, "The scan body fits poorly. Check the implant system and the scan.");
            refreshImplantOverlays(app);
            app.markModified();
        };
    });
}

// Ray through a screen point (inverse of the projector).
std::optional<Ray> rayAt(const Projector& proj, const ImVec2& mouse)
{
    const double w = proj.max.x - proj.min.x, h = proj.max.y - proj.min.y;
    if (w <= 0.0 || h <= 0.0)
        return std::nullopt;
    const double x = (mouse.x - proj.min.x) / w * 2.0 - 1.0, y = 1.0 - (mouse.y - proj.min.y) / h * 2.0;
    const glm::dmat4 inv = glm::inverse(proj.viewProj);
    glm::dvec4 a = inv * glm::dvec4(x, y, -1.0, 1.0), b = inv * glm::dvec4(x, y, 1.0, 1.0);
    a /= a.w;
    b /= b.w;
    Ray r;
    r.origin = glm::dvec3(a);
    r.direction = glm::normalize(glm::dvec3(b - a));
    return r;
}

// ---------------------------------------------------------------------------
// Scan body alignment
// ---------------------------------------------------------------------------

class ScanBodyAlignmentStep final : public Step {
public:
    ScanBodyAlignmentStep() : Step(StepId::ScanBodyAlignment) {}

    ViewLayout preferredLayout() const override { return ViewLayout::Single3D; }

    std::optional<std::string> blocker(const DesignerApp& appC) const override
    {
        auto& app = const_cast<DesignerApp&>(appC);
        if (app.doc().implants.empty())
            return std::string("There is no implant restoration to design.");
        for (auto& r : app.doc().implants) {
            if (!r.hasConnection())
                return std::format("Choose the implant for {}.", app.toothText(r.tooth));
            if (!r.implantToScan)
                return std::format("Match the scan body of {}.", app.toothText(r.tooth));
        }
        return std::nullopt;
    }

    void onEnter(DesignerApp& app) override
    {
        syncImplants(app);
        app.view3D(ViewId::Main3D).pickCursor = true;
        refreshImplantOverlays(app);
        viewPending_ = true;
    }

    void onLeave(DesignerApp& app) override { app.view3D(ViewId::Main3D).pickCursor = false; }

    void update(DesignerApp& app) override
    {
        syncImplants(app);
        if (viewPending_ && !app.loading() && app.doc().pendingImplants.empty()) {
            viewPending_ = false;
            refreshImplantOverlays(app);
            if (ImplantRestoration* r = app.doc().activeImplantRestoration(); r && r->implantToScan)
                viewImplant(app, *r, 0.6, 10.0);
        }
    }

    void drawPanel(DesignerApp& app) override
    {
        const ui::Palette& pal = ui::palette();
        if (drawImplantCard(app, true))
            if (ImplantRestoration* r = app.doc().activeImplantRestoration(); r && r->implantToScan)
                viewImplant(app, *r, 0.6, 10.0);
        ImplantRestoration* r = app.doc().activeImplantRestoration();
        if (!r)
            return;
        ImGui::Spacing();
        ui::beginCard("##scanbody");
        ui::subheading("Scan body");
        if (!r->connection) {
            ui::wrappedMutedText("Choose the implant system first: its library provides the scan body.");
        } else if (app.tasks().busy()) {
            ui::mutedText("Matching...");
        } else if (r->implantToScan) {
            ImGui::TextColored(r->fit.fractionWithin >= 0.8 ? pal.success : pal.warning, r->fit.fractionWithin >= 0.8 ? "Matched" : "Matched (poor fit)");
            if (r->fit.inlierFraction > 0.0)
                ui::mutedText("%.3f mm RMS, %.0f%% within 0.05 mm", r->fit.rms, r->fit.fractionWithin * 100.0);
            ImGui::Spacing();
            ui::wrappedMutedText("Click on the scan body again to match it from there.");
            ImGui::Spacing();
            const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
            if (ui::button("Refine", ImVec2(half, 0))) {
                if (ScanObject* scan = scanOf(app, *r)) {
                    const implant::ScanBodyFitResult fit = implant::refineScanBody(*scan->mesh, *r->connection->scanBody, *r->implantToScan);
                    if (fit.ok) {
                        r->implantToScan = fit.implantToScan;
                        r->fit = fit;
                        r->invalidate();
                        refreshImplantOverlays(app);
                        app.markModified();
                    }
                }
            }
            ImGui::SameLine();
            if (ui::button("Clear", ImVec2(half, 0))) {
                r->implantToScan.reset();
                r->fit = {};
                r->invalidate();
                refreshImplantOverlays(app);
                app.markModified();
            }
        } else {
            ui::wrappedMutedText("Click on the top of the scan body in the scan. The library scan body is matched to it, which "
                                 "gives the position and rotation of the implant.");
        }
        ui::endCard();
        if (r->implantToScan) {
            ImGui::Spacing();
            bool vis = true;
            if (auto it = app.doc().overlays.find(key("scanbody", r->tooth)); it != app.doc().overlays.end()) {
                vis = it->second.visible;
                if (ImGui::Checkbox("Show library scan body", &vis)) {
                    it->second.visible = vis;
                    app.doc().redraw();
                }
            }
        }
    }

    OverlayFn overlay(DesignerApp& app, ViewId view) override
    {
        if (view != ViewId::Main3D)
            return {};
        return [&app](ImDrawList* dl, const Projector& proj) {
            ImplantRestoration* r = app.doc().activeImplantRestoration();
            if (!r || app.tasks().busy())
                return;
            if (!r->hasConnection())
                drawHint(dl, proj, "Choose the implant system first");
            else if (!r->implantToScan)
                drawHint(dl, proj, "Click on the top of the scan body");
        };
    }

    void onViewEvent(DesignerApp& app, ViewId view, const ViewEvents& ev) override
    {
        if (view != ViewId::Main3D || !ev.click || app.tasks().busy())
            return;
        ImplantRestoration* r = app.doc().activeImplantRestoration();
        if (!r || !ensureConnection(*r))
            return;
        ScanObject* scan = scanOf(app, *r);
        if (!scan)
            return;
        auto hit = raycastMesh(*scan->mesh, scan->transform, *ev.click);
        if (!hit)
            return;
        startScanBodyFit(app, r->tooth, transformPoint(glm::inverse(scan->transform), hit->point));
    }

private:
    bool viewPending_ = false;
};

// ---------------------------------------------------------------------------
// Abutment design
// ---------------------------------------------------------------------------

class AbutmentDesignStep final : public Step {
public:
    AbutmentDesignStep() : Step(StepId::AbutmentDesign) {}

    ViewLayout preferredLayout() const override { return ViewLayout::Single3D; }

    std::optional<std::string> blocker(const DesignerApp& appC) const override
    {
        auto& app = const_cast<DesignerApp&>(appC);
        for (auto& r : app.doc().implants)
            if (r.designsAbutment() && (!r.hasConnection() || !r.implantToScan))
                return std::format("Find the implant position of {} first (scan body alignment).", app.toothText(r.tooth));
        return std::nullopt;
    }

    void onEnter(DesignerApp& app) override
    {
        syncImplants(app);
        for (auto& r : app.doc().implants)
            ensureGeometry(app, r);
        refreshImplantOverlays(app);
        applyScanDisplay(app);
        viewPending_ = true;
    }

    void onLeave(DesignerApp& app) override
    {
        endDrag(app);
        restoreScans(app);
    }

    void update(DesignerApp& app) override
    {
        syncImplants(app);
        // Follow the scan's visibility in the objects panel.
        for (auto& [k, o] : app.doc().overlays)
            if (k.rfind("scanview:", 0) == 0) {
                const ScanObject* sc = app.doc().findScan(o.scanId);
                const bool want = showScan_ && sc && sc->visible;
                if (o.visible != want) {
                    o.visible = want;
                    app.doc().redraw();
                }
            }
        if (viewPending_ && !app.loading() && app.doc().pendingImplants.empty()) {
            viewPending_ = false;
            for (auto& r : app.doc().implants)
                ensureGeometry(app, r);
            refreshImplantOverlays(app);
            applyScanDisplay(app);
            if (ImplantRestoration* r = app.doc().activeImplantRestoration())
                viewFromSide(app, *r);
        }
    }

    void drawPanel(DesignerApp& app) override
    {
        const ui::Palette& pal = ui::palette();
        if (drawImplantCard(app, true)) {
            if (ImplantRestoration* r = app.doc().activeImplantRestoration()) {
                ensureGeometry(app, *r);
                refreshImplantOverlays(app);
                focusOnImplant(app, *r);
            }
        }
        ImplantRestoration* r = app.doc().activeImplantRestoration();
        if (!r || !r->designsAbutment())
            return;
        if (!ensureGeometry(app, *r)) {
            ImGui::Spacing();
            ui::wrappedMutedText(r->hasConnection() ? "Match the scan body first (previous step)." : "Choose the implant first.");
            return;
        }
        implant::AbutmentShape& s = *r->shape;
        const implant::Connection& c = *r->connection;
        const float full = -FLT_MIN;
        bool changed = false;
        auto pushUndoOnActivate = [&] {
            if (ImGui::IsItemActivated())
                pushUndo(*r);
        };

        // Emergence profile.
        ImGui::Spacing();
        ui::beginCard("##emergence");
        ui::subheading("Emergence profile");
        ui::wrappedMutedText("Drag the orange margin points and the blue points halfway down (convex or concave). Hold Shift to move a "
                             "whole ring.");
        ImGui::Spacing();
        {
            double meanH = 0.0, meanR = 0.0, meanD = 0.0;
            for (int k = 0; k < implant::kControlPoints; ++k) {
                meanH += s.marginHeight[static_cast<std::size_t>(k)] / implant::kControlPoints;
                meanR += s.marginRadius[static_cast<std::size_t>(k)] / implant::kControlPoints;
                meanD += s.midOffset[static_cast<std::size_t>(k)] / implant::kControlPoints;
            }
            float h = static_cast<float>(meanH), d = static_cast<float>(meanR * 2.0), m = static_cast<float>(meanD);
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat("##mh", &h, 0.3f, 6.0f, "Margin height  %.2f mm")) {
                for (auto& v : s.marginHeight)
                    v = std::clamp(v + (h - meanH), 0.2, 8.0);
                changed = true;
            }
            pushUndoOnActivate();
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat("##md", &d, static_cast<float>(c.topRadius * 2.0 - 0.5), 9.0f, "Margin diameter  %.2f mm")) {
                for (auto& v : s.marginRadius)
                    v = std::clamp(v + (d * 0.5 - meanR), implant::minCoreRadius(s, c) + 0.2, 6.0);
                changed = true;
            }
            pushUndoOnActivate();
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat("##mo", &m, -1.0f, 1.0f, m >= 0.0f ? "Emergence  %+.2f mm (convex)" : "Emergence  %+.2f mm (concave)")) {
                for (auto& v : s.midOffset)
                    v = std::clamp(v + (m - meanD), -1.5, 1.5);
                changed = true;
            }
            pushUndoOnActivate();
        }
        ImGui::Spacing();
        float depth = static_cast<float>(gingivaDepth_);
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 7.5f);
        if (ImGui::SliderFloat("##gd", &depth, 0.0f, 2.0f, "%.1f mm below"))
            gingivaDepth_ = depth;
        ImGui::SameLine();
        if (ui::button("Fit to gingiva", ImVec2(full, 0))) {
            pushUndo(*r);
            if (fitMarginToGingiva(app, *r, s, gingivaDepth_) == 0)
                ui::toast(ui::ToastKind::Warning, "No gingiva found around the scan body.");
            changed = true;
        }
        ui::endCard();

        // Core.
        ImGui::Spacing();
        ui::beginCard("##core");
        ui::subheading("Core");
        bool follows = s.coreLocked;
        if (ImGui::Checkbox("Follows the margin", &follows)) {
            pushUndo(*r);
            if (follows)
                s.coreLocked = true;
            else
                s.unlockCore(implant::minCoreRadius(s, c));
            changed = true;
        }
        ui::helpMarker("While locked, the core outline follows the margin points: its height above the margin and its taper are "
                       "set here. Unlock it to drag each of the 9 green points of the core on its own.");
        float sw = static_cast<float>(s.shoulderWidth);
        ImGui::SetNextItemWidth(full);
        if (ImGui::SliderFloat("##sw", &sw, 0.0f, 1.5f, "Shoulder width  %.2f mm")) {
            s.shoulderWidth = sw;
            changed = true;
        }
        pushUndoOnActivate();
        if (s.coreLocked) {
            float ch = static_cast<float>(s.coreHeight), tp = static_cast<float>(s.taperDeg);
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat("##ch", &ch, 2.0f, 10.0f, "Core height  %.2f mm")) {
                s.coreHeight = ch;
                changed = true;
            }
            pushUndoOnActivate();
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderFloat("##tp", &tp, 0.0f, 15.0f, "Taper  %.1f\xC2\xB0")) {
                s.taperDeg = tp;
                changed = true;
            }
            pushUndoOnActivate();
        } else {
            ui::wrappedMutedText("Drag the green points to shape the core.");
        }
        ui::endCard();

        // Screw channel and checks.
        ImGui::Spacing();
        ui::beginCard("##checks");
        ui::subheading("Screw channel and checks");
        float sc = static_cast<float>(s.screwChannelDiameter);
        ImGui::SetNextItemWidth(full);
        if (ImGui::SliderFloat("##sc", &sc, 1.5f, 4.0f, "Screw channel  \xC3\x98%.2f mm")) {
            s.screwChannelDiameter = sc;
            changed = true;
        }
        pushUndoOnActivate();
        if (c.screwChannel)
            ui::mutedText("The library's screw channel is also removed.");
        const double minWall = c.info.minWall;
        const double wall = r->geometry->minWall;
        ImGui::TextColored(wall < minWall - 1e-3 ? pal.danger : pal.success, "Thinnest wall %.2f mm", wall);
        ImGui::SameLine();
        ui::mutedText("(minimum %.2f)", minWall);
        for (const auto& w : r->geometry->warnings)
            ImGui::TextColored(pal.warning, "%s", w.c_str());
        if (ImGui::Checkbox("Show wall thickness", &gWallMap))
            refreshImplantOverlays(app);
        ui::endCard();

        // Display.
        ImGui::Spacing();
        if (ImGui::Checkbox("Show scan", &showScan_))
            updateScanOpacity(app);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(full);
        if (ImGui::SliderFloat("##op", &scanOpacity_, 0.1f, 1.0f, "Opacity %.1f"))
            updateScanOpacity(app);
        ImGui::Spacing();
        const float third = (ImGui::GetContentRegionAvail().x - 2 * ImGui::GetStyle().ItemSpacing.x) / 3.0f;
        auto& stack = undo_[r->tooth];
        if (ui::button("Undo", ImVec2(third, 0), !stack.empty())) {
            s = stack.back();
            stack.pop_back();
            changed = true;
        }
        ImGui::SameLine();
        if (ui::button("Reset", ImVec2(third, 0))) {
            pushUndo(*r);
            s = makeDefaultShape(app, *r);
            changed = true;
        }
        ImGui::SameLine();
        if (ui::button("Side view", ImVec2(third, 0)))
            viewFromSide(app, *r);
        if (changed)
            shapeChanged(app, *r);
    }

    OverlayFn overlay(DesignerApp& app, ViewId view) override
    {
        if (view != ViewId::Main3D)
            return {};
        return [this, &app](ImDrawList* dl, const Projector& proj) { drawHandles(app, dl, proj); };
    }

private:
    struct Handle {
        Ring ring;
        int index;
    };

    void pushUndo(const ImplantRestoration& r)
    {
        if (!r.shape)
            return;
        auto& stack = undo_[r.tooth];
        stack.push_back(*r.shape);
        if (stack.size() > 50)
            stack.erase(stack.begin());
    }

    void shapeChanged(DesignerApp& app, ImplantRestoration& r)
    {
        r.invalidate();
        ensureGeometry(app, r);
        updateImplantOverlays(app, r, gWallMap);
        app.markModified();
    }

    // The scans holding scan bodies are shown without them (as an overlay; the scan itself is hidden
    // while this step is open).
    void applyScanDisplay(DesignerApp& app)
    {
        Document& doc = app.doc();
        std::map<int, std::vector<std::pair<glm::dmat4, std::shared_ptr<const implant::Connection>>>> bodies;
        for (auto& r : doc.implants)
            if (scanOf(app, r) && r.implantToScan && ensureConnection(r))
                bodies[r.scanId].emplace_back(*r.implantToScan, r.connection);
        for (const auto& [id, list] : bodies) {
            ScanObject* s = doc.findScan(id);
            s->stepHidden = true;
            hidden_.insert(id);
            std::string sig = std::to_string(reinterpret_cast<std::uintptr_t>(s->mesh.get()));
            for (const auto& [t, c] : list)
                for (int i = 0; i < 4; ++i)
                    sig += std::format("|{:.4f},{:.4f},{:.4f},{:.4f}", t[i][0], t[i][1], t[i][2], t[i][3]);
            auto& cache = masked_[id];
            if (cache.first != sig) {
                cache.first = sig;
                cache.second = std::make_shared<const Mesh>(scanWithoutScanBodies(*s->mesh, list));
            }
            DisplayMesh dm;
            dm.gpu = std::make_unique<gfx::GpuMesh>(*cache.second);
            dm.scanId = id;
            dm.color = s->color;
            dm.opacity = scanOpacity_;
            dm.visible = showScan_ && s->visible;
            doc.setOverlay(std::format("scanview:{}", id), std::move(dm));
        }
        doc.redraw();
    }

    void updateScanOpacity(DesignerApp& app)
    {
        for (auto& [k, o] : app.doc().overlays)
            if (k.rfind("scanview:", 0) == 0) {
                const ScanObject* s = app.doc().findScan(o.scanId);
                o.opacity = scanOpacity_;
                o.visible = showScan_ && s && s->visible;
            }
        app.doc().redraw();
    }

    void restoreScans(DesignerApp& app)
    {
        for (int id : hidden_)
            if (ScanObject* s = app.doc().findScan(id))
                s->stepHidden = false;
        hidden_.clear();
        app.doc().removeOverlaysWithPrefix("scanview:");
    }

    void endDrag(DesignerApp& app)
    {
        dragging_.reset();
        app.view3D(ViewId::Main3D).blockOrbit = false;
    }

    glm::dvec3 handlePoint(const implant::AbutmentGeometry& g, const Handle& h) const
    {
        const auto i = static_cast<std::size_t>(h.index);
        return h.ring == MarginRing ? g.marginPoints[i] : h.ring == MidRing ? g.midPoints[i] : g.corePoints[i];
    }

    // Moves handle `h` so that it lies under the mouse ray (in the plane through the implant axis
    // at the handle's azimuth).
    void dragTo(ImplantRestoration& r, const Ray& worldRay, const glm::dmat4& implantToWorld, bool wholeRing)
    {
        implant::AbutmentShape& s = *r.shape;
        const implant::Connection& c = *r.connection;
        const Handle h = *dragging_;
        const double a = s.azimuth(h.index);
        const glm::dmat4 inv = glm::inverse(implantToWorld);
        const glm::dvec3 o = transformPoint(inv, worldRay.origin);
        const glm::dvec3 d = glm::normalize(transformVector(inv, worldRay.direction));
        const glm::dvec3 radial(std::cos(a), std::sin(a), 0.0), normal(-std::sin(a), std::cos(a), 0.0);
        glm::dvec3 q;
        const double dn = glm::dot(d, normal);
        if (std::abs(dn) > 0.08) {
            q = o + d * (-glm::dot(o, normal) / dn);
        } else {
            // Looking along the plane: use the plane facing the camera through the handle instead.
            const glm::dvec3 p0 = handlePoint(*r.geometry, h);
            const double dd = glm::dot(d, d);
            q = o + d * (glm::dot(p0 - o, d) / dd);
        }
        const glm::dvec2 rz(glm::dot(q, radial), q.z - c.topHeight);
        const implant::AbutmentShape& s0 = dragStart_;
        const auto k = static_cast<std::size_t>(h.index);
        const double minR = implant::minCoreRadius(s, c);
        auto apply = [&](auto&& setOne) {
            if (!wholeRing) {
                setOne(k, 1.0);
                return;
            }
            for (std::size_t i = 0; i < implant::kControlPoints; ++i)
                setOne(i, 0.0);
        };
        if (h.ring == MarginRing) {
            const glm::dvec2 delta = rz - glm::dvec2(s0.marginRadius[k], s0.marginHeight[k]);
            apply([&](std::size_t i, double) {
                s.marginRadius[i] = std::clamp(s0.marginRadius[i] + delta.x, minR + 0.2, 6.0);
                s.marginHeight[i] = std::clamp(s0.marginHeight[i] + delta.y, 0.2, 8.0);
            });
        } else if (h.ring == MidRing) {
            auto offsetAt = [&](std::size_t i, const glm::dvec2& p) {
                const glm::dvec2 B(c.topRadius, 0.0), M(s0.marginRadius[i], std::max(s0.marginHeight[i], 0.2));
                const glm::dvec2 dir = glm::normalize(M - B), n(dir.y, -dir.x);
                return glm::dot(p - (B + M) * 0.5, n);
            };
            const double delta = offsetAt(k, rz) - s0.midOffset[k];
            apply([&](std::size_t i, double) { s.midOffset[i] = std::clamp(s0.midOffset[i] + delta, -1.5, 1.5); });
        } else if (!s.coreLocked) {
            const glm::dvec2 delta = rz - glm::dvec2(s0.coreRadius[k], s0.coreTop[k]);
            apply([&](std::size_t i, double) {
                s.coreRadius[i] = std::clamp(s0.coreRadius[i] + delta.x, minR, 6.0);
                s.coreTop[i] = std::clamp(s0.coreTop[i] + delta.y, s.marginHeight[i] + 0.8, 14.0);
            });
        }
    }

    void drawHandles(DesignerApp& app, ImDrawList* dl, const Projector& proj)
    {
        ImplantRestoration* r = app.doc().activeImplantRestoration();
        View3D& view = app.view3D(ViewId::Main3D);
        if (!r || !r->designsAbutment() || !r->geometry || !r->shape || !r->implantToScan) {
            if (r && r->designsAbutment() && !r->implantToScan)
                drawHint(dl, proj, "Find the implant position first (scan body alignment)");
            endDragIfAny(app);
            return;
        }
        ScanObject* scan = scanOf(app, *r);
        if (!scan) {
            endDragIfAny(app);
            return;
        }
        const glm::dmat4 toWorld = scan->transform * *r->implantToScan;
        const float sc = ImGui::GetStyle().FontScaleDpi;
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        const bool inView = proj.contains(mouse) && ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
        const bool shift = ImGui::GetIO().KeyShift;

        // Dragging.
        if (dragging_) {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                if (auto ray = rayAt(proj, mouse)) {
                    dragTo(*r, *ray, toWorld, shift);
                    r->invalidate();
                    ensureGeometry(app, *r);
                    updateImplantOverlays(app, *r, gWallMap);
                    app.markModified();
                }
            } else {
                endDrag(app);
            }
        }

        // Rings and handles.
        const implant::AbutmentGeometry& g = *r->geometry;
        auto screen = [&](const glm::dvec3& p) { return proj(transformPoint(toWorld, p)); };
        auto ring = [&](const std::array<glm::dvec3, implant::kControlPoints>& pts, ImU32 col) {
            for (int k = 0; k < implant::kControlPoints; ++k) {
                auto a = screen(pts[static_cast<std::size_t>(k)]), b = screen(pts[static_cast<std::size_t>((k + 1) % implant::kControlPoints)]);
                if (a && b)
                    dl->AddLine(*a, *b, (col & 0x00FFFFFF) | 0x70000000, 1.2f * sc);
            }
        };
        ring(g.marginPoints, kMarginHandle);
        ring(g.midPoints, kMidHandle);
        ring(g.corePoints, kCoreHandle);

        std::optional<Handle> hovered;
        float best = 10.0f * sc;
        const Ring rings[] = {CoreRing, MidRing, MarginRing}; // margin wins when handles overlap
        for (Ring rg : rings)
            for (int k = 0; k < implant::kControlPoints; ++k) {
                auto p = screen(handlePoint(g, {rg, k}));
                if (!p)
                    continue;
                const float dist = std::hypot(p->x - mouse.x, p->y - mouse.y);
                if (dist <= best) {
                    best = dist;
                    hovered = Handle{rg, k};
                }
            }
        if (!dragging_ && hovered && inView) {
            const bool movable = hovered->ring != CoreRing || !r->shape->coreLocked;
            ImGui::SetMouseCursor(movable ? ImGuiMouseCursor_ResizeAll : ImGuiMouseCursor_NotAllowed);
            if (movable && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                pushUndo(*r);
                dragStart_ = *r->shape;
                dragging_ = hovered;
                view.blockOrbit = true;
            }
        }
        for (Ring rg : {CoreRing, MidRing, MarginRing}) {
            const ImU32 col = rg == MarginRing ? kMarginHandle : rg == MidRing ? kMidHandle : kCoreHandle;
            const bool locked = rg == CoreRing && r->shape->coreLocked;
            for (int k = 0; k < implant::kControlPoints; ++k) {
                auto p = screen(handlePoint(g, {rg, k}));
                if (!p)
                    continue;
                const bool active = (dragging_ && dragging_->ring == rg && (dragging_->index == k || shift)) ||
                                    (!dragging_ && hovered && hovered->ring == rg && hovered->index == k && inView);
                const float rad = (active ? 6.5f : 5.0f) * sc;
                dl->AddCircleFilled(*p, rad + 1.2f * sc, IM_COL32(0, 0, 0, 160));
                if (locked)
                    dl->AddCircle(*p, rad - 0.5f * sc, col, 0, 2.0f * sc);
                else
                    dl->AddCircleFilled(*p, rad, active ? IM_COL32(255, 255, 255, 255) : col);
            }
        }

        // Value read-out while dragging.
        if (dragging_) {
            const auto k = static_cast<std::size_t>(dragging_->index);
            const implant::AbutmentShape& s = *r->shape;
            std::string text;
            if (dragging_->ring == MarginRing)
                text = std::format("Margin  \xC3\x98 {:.2f} mm   height {:.2f} mm", s.marginRadius[k] * 2.0, s.marginHeight[k]);
            else if (dragging_->ring == MidRing)
                text = std::format("Emergence  {:+.2f} mm {}", s.midOffset[k], s.midOffset[k] >= 0.0 ? "(convex)" : "(concave)");
            else
                text = std::format("Core  r {:.2f} mm   height {:.2f} mm", s.coreRadius[k], s.coreTop[k] - s.marginHeight[k]);
            if (shift)
                text += "   (whole ring)";
            const ImVec2 at(mouse.x + 16 * sc, mouse.y + 10 * sc);
            const ImVec2 ts = ImGui::CalcTextSize(text.c_str());
            dl->AddRectFilled(ImVec2(at.x - 6 * sc, at.y - 3 * sc), ImVec2(at.x + ts.x + 6 * sc, at.y + ts.y + 3 * sc), IM_COL32(15, 18, 22, 210), 4 * sc);
            dl->AddText(at, IM_COL32(240, 242, 245, 255), text.c_str());
        } else if (hovered && inView && hovered->ring == CoreRing && r->shape->coreLocked) {
            ImGui::SetTooltip("The core follows the margin. Untick \"Follows the margin\" to move its points.");
        }
    }

    void endDragIfAny(DesignerApp& app)
    {
        if (dragging_)
            endDrag(app);
    }

    bool viewPending_ = false;
    bool showScan_ = true;
    float scanOpacity_ = 0.6f;
    double gingivaDepth_ = 0.5;
    std::set<int> hidden_; // scans shown as "scanview:" overlays while this step is open
    std::map<int, std::pair<std::string, std::shared_ptr<const Mesh>>> masked_; // scan id -> (signature, scan without scan bodies)
    std::optional<Handle> dragging_;
    implant::AbutmentShape dragStart_;
    std::map<int, std::vector<implant::AbutmentShape>> undo_;
};

} // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

std::unique_ptr<Step> makeScanBodyAlignmentStep()
{
    return std::make_unique<ScanBodyAlignmentStep>();
}

std::unique_ptr<Step> makeAbutmentDesignStep()
{
    return std::make_unique<AbutmentDesignStep>();
}

void syncImplants(DesignerApp& app)
{
    Document& doc = app.doc();
    if (!doc.pendingImplants.empty()) {
        if (app.loading())
            return;
        auto bysource = [&](const std::string& src) -> ScanObject* {
            for (const auto& s : doc.scans)
                if (s->source == src)
                    return s.get();
            return nullptr;
        };
        for (const auto& saved : doc.pendingImplants)
            if (!saved.scanSource.empty() && !bysource(saved.scanSource))
                return; // still loading
        for (const auto& saved : doc.pendingImplants) {
            ImplantRestoration r;
            r.tooth = saved.tooth;
            r.type = saved.type;
            ScanObject* scan = bysource(saved.scanSource);
            r.scanId = scan ? scan->id : 0;
            r.scanAssigned = scan != nullptr;
            r.libraryId = saved.libraryId;
            r.connectionId = saved.connectionId;
            if (scan) {
                r.implantToScan = saved.implantToScan;
                r.fit.rms = saved.fitRms;
                r.fit.fractionWithin = saved.fitWithin;
                r.fit.ok = saved.implantToScan.has_value();
            }
            r.shape = saved.shape;
            doc.implants.push_back(std::move(r));
        }
        doc.pendingImplants.clear();
        refreshImplantOverlays(app);
    }
    if (auto* rec = app.caseRecord())
        for (const auto& cr : rec->restorations) {
            if (!isImplantType(cr.type) || findImplant(app, cr.tooth))
                continue;
            ImplantRestoration r;
            r.tooth = cr.tooth;
            r.type = cr.type;
            doc.implants.push_back(std::move(r));
        }
    for (auto& r : doc.implants)
        if ((!r.scanAssigned || !scanOf(app, r)) && !app.loading() && !doc.scans.empty()) {
            assignScan(app, r);
            r.scanAssigned = true;
        }
}

std::vector<SavedImplant> captureImplants(const DesignerApp& appC)
{
    auto& app = const_cast<DesignerApp&>(appC);
    std::vector<SavedImplant> out;
    for (const auto& r : app.doc().implants) {
        SavedImplant s;
        s.tooth = r.tooth;
        s.type = r.type;
        if (ScanObject* scan = scanOf(app, r))
            s.scanSource = scan->source;
        s.libraryId = r.libraryId;
        s.connectionId = r.connectionId;
        s.implantToScan = r.implantToScan;
        s.fitRms = r.fit.rms;
        s.fitWithin = r.fit.fractionWithin;
        s.shape = r.shape;
        out.push_back(s);
    }
    for (const auto& p : app.doc().pendingImplants)
        out.push_back(p); // not restored yet (scans still loading)
    return out;
}

std::vector<CrownExport> abutmentExports(DesignerApp& app)
{
    std::vector<CrownExport> out;
    for (auto& r : app.doc().implants) {
        if (!r.designsAbutment() || !r.shape || !ensureGeometry(app, r))
            continue;
        if (!r.solid)
            r.solid = std::make_shared<const implant::AbutmentSolid>(implant::finishAbutment(*r.geometry, *r.shape, *r.connection));
        if (!r.solid->error.empty()) {
            log::warn("Abutment {} is not exported: {}", app.toothText(r.tooth), r.solid->error);
            continue;
        }
        for (const auto& w : r.solid->warnings)
            log::warn("Abutment {}: {}", app.toothText(r.tooth), w);
        auto mesh = std::make_shared<Mesh>(toScan(r.solid->mesh, *r.implantToScan));
        mesh->colors.clear();
        const std::string tooth = app.toothText(r.tooth);
        const std::string system = connectionLabel(r);
        out.push_back({r.tooth, std::format("Abutment {}", tooth), std::format("abutment_{}", tooth), mesh,
                       app.numbering() == dental::Numbering::Universal
                           ? std::format("OcclusaCAD Abutment {} (Universal; FDI {}), {}, scan coordinates, mm", tooth, r.tooth, system)
                           : std::format("OcclusaCAD Abutment {} (FDI), {}, scan coordinates, mm", tooth, system)});
    }
    return out;
}

bool allAbutmentsExported(const DesignerApp& appC)
{
    auto& app = const_cast<DesignerApp&>(appC);
    for (auto& r : app.doc().implants)
        if (r.designsAbutment() && r.shape && (!ensureConnection(r) || !r.implantToScan))
            return false;
    return true;
}

// ---------------------------------------------------------------------------
// Demo
// ---------------------------------------------------------------------------

bool runAbutmentDemo(DesignerApp& app, AbutmentDemo& demo)
{
    if (demo.state >= 100)
        return false;
    if (app.loading() || demo.waitFrames-- > 0)
        return true;
    auto finish = [&](int code) {
        if (code)
            app.setExitCode(code);
        demo.state = 100;
        return false;
    };
    nlohmann::json truth;
    try {
        std::ifstream in(demo.truthFile);
        truth = nlohmann::json::parse(in);
    } catch (const std::exception& e) {
        log::error("DEMO cannot read {}: {}", demo.truthFile.string(), e.what());
        return finish(2);
    }
    syncImplants(app);
    const int tooth = truth.value("tooth", 0);
    ImplantRestoration* r = findImplant(app, tooth);
    auto vec = [&](const char* k) { return glm::dvec3(truth[k][0].get<double>(), truth[k][1].get<double>(), truth[k][2].get<double>()); };
    switch (demo.state) {
    case 0: // wait for the data, choose the implant
        if (app.doc().scans.empty() || !app.doc().pendingImplants.empty() || !r || !r->scanId)
            return true;
        app.goToStep(StepId::ScanBodyAlignment);
        r->libraryId = truth.value("library", "");
        r->connectionId = truth.value("connection", "");
        r->shape.reset();
        r->invalidate();
        if (!ensureConnection(*r)) {
            log::error("DEMO implant connection: {}", r->connectionError);
            return finish(2);
        }
        startScanBodyFit(app, tooth, vec("click"));
        demo.state = 1;
        demo.waitFrames = 2;
        return true;
    case 1: { // compare with the ground truth
        if (app.tasks().busy())
            return true;
        if (!r->implantToScan) {
            log::error("DEMO scan body not found");
            return finish(3);
        }
        const glm::dvec3 platform = transformPoint(*r->implantToScan, glm::dvec3(0.0));
        const glm::dvec3 axis = glm::normalize(transformVector(*r->implantToScan, glm::dvec3(0, 0, 1)));
        const double posErr = glm::length(platform - vec("platform"));
        const double angErr = glm::degrees(std::acos(std::clamp(glm::dot(axis, glm::normalize(vec("axis"))), -1.0, 1.0)));
        log::info("DEMO scan body {}: platform error {:.3f} mm, axis error {:.2f} deg, fit {:.3f} mm RMS, {:.0f}% within 0.05 mm", app.toothText(tooth),
                  posErr, angErr, r->fit.rms, r->fit.fractionWithin * 100.0);
        if (demo.maxError > 0.0 && (posErr > demo.maxError || angErr > 1.0))
            return finish(3);
        app.goToStep(StepId::AbutmentDesign);
        demo.state = 2;
        demo.waitFrames = 3;
        return true;
    }
    case 2: { // default abutment
        if (!ensureGeometry(app, *r)) {
            log::error("DEMO abutment could not be built");
            return finish(3);
        }
        const implant::AbutmentShape& s = *r->shape;
        double hMin = 1e9, hMax = -1e9;
        for (double h : s.marginHeight) {
            hMin = std::min(hMin, h);
            hMax = std::max(hMax, h);
        }
        const auto exports = abutmentExports(app);
        const bool ok = !exports.empty() && r->solid && r->solid->merged;
        log::info("DEMO abutment {}: margin \xC3\x98{:.2f} mm (interface \xC3\x98{:.2f}), height {:.2f}-{:.2f} mm, thinnest wall {:.2f} mm, {} ({} triangles)",
                  app.toothText(tooth), s.marginRadius[0] * 2.0, r->connection->topRadius * 2.0, hMin, hMax, r->geometry->minWall,
                  ok ? "united with the interface" : "NOT united", ok ? exports.front().mesh->triangleCount() : 0);
        if (!ok)
            return finish(3);
        if (demo.save && app.caseMode())
            app.saveDesign(true);
        demo.state = 3;
        demo.waitFrames = 2;
        return true;
    }
    default:
        return finish(0);
    }
}

} // namespace occlusa::designer
