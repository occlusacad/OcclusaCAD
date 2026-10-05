// occlusa_phantom: synthetic test data generator.
//
// Produces a CBCT-like DICOM series of a lower jaw (bone, teeth with enamel caps, a
// missing first molar as implant site), an intraoral-style surface scan (teeth + gingiva)
// in its own scanner coordinate system, and the ground-truth scan->CBCT transform.
// A second data set (crown/) has a shoulder preparation on 46 with the opposing upper
// jaw and the analytic margin line, for the crown & bridge workflow.
// Optionally creates demo cases in a OcclusaCAD data folder.
//
//   occlusa_phantom --out <dir> [--voxel 0.4] [--seed 7]
//   occlusa_phantom --out <dir> --create-case <dataRoot> [--extra-cases 6]

#include "core/AppConfig.h"
#include "core/CommandLine.h"
#include "core/IsoSurface.h"
#include "core/Log.h"
#include "core/Parallel.h"
#include "core/Platform.h"
#include "core/StlIO.h"
#include "core/Time.h"
#include "core/dicom/DicomWriter.h"
#include "db/CaseRepository.h"

#include <json.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <stdexcept>
#include <algorithm>
#include <vector>

namespace fs = std::filesystem;
using namespace occlusa;

namespace {

constexpr double kGumZ = 0.0; // gingival margin height (mm, patient superior axis)

struct Arch {
    std::vector<glm::dvec2> pts;   // dense polyline
    std::vector<double> arcLength; // cumulative

    Arch()
    {
        const int n = 400;
        for (int i = 0; i <= n; ++i) {
            const double t = -1.45 + 2.9 * i / n;
            pts.emplace_back(27.0 * std::sin(t), -24.0 + 22.0 * (1.0 - std::cos(t)));
        }
        // Straight distal extensions so the molars sit on the arch (not clamped to its end).
        const glm::dvec2 dirEnd = glm::normalize(pts.back() - pts[pts.size() - 2]);
        const glm::dvec2 dirStart = glm::normalize(pts.front() - pts[1]);
        const glm::dvec2 end = pts.back(), start = pts.front();
        for (int i = 1; i <= 60; ++i)
            pts.push_back(end + dirEnd * (0.25 * i));
        for (int i = 1; i <= 60; ++i)
            pts.insert(pts.begin(), start + dirStart * (0.25 * i));
        arcLength.push_back(0.0);
        for (std::size_t i = 1; i < pts.size(); ++i)
            arcLength.push_back(arcLength.back() + glm::length(pts[i] - pts[i - 1]));
    }

    double length() const { return arcLength.back(); }

    // Point and unit tangent at arc length s.
    std::pair<glm::dvec2, glm::dvec2> at(double s) const
    {
        s = std::clamp(s, 0.0, length());
        auto it = std::lower_bound(arcLength.begin(), arcLength.end(), s);
        std::size_t i = std::max<std::size_t>(1, static_cast<std::size_t>(it - arcLength.begin()));
        i = std::min(i, pts.size() - 1);
        const double f = (s - arcLength[i - 1]) / std::max(arcLength[i] - arcLength[i - 1], 1e-9);
        return {glm::mix(pts[i - 1], pts[i], f), glm::normalize(pts[i] - pts[i - 1])};
    }

    double distance(const glm::dvec2& p) const
    {
        double best = 1e9;
        for (std::size_t i = 1; i < pts.size(); i += 1) {
            const glm::dvec2 a = pts[i - 1], b = pts[i];
            const glm::dvec2 ab = b - a;
            const double t = std::clamp(glm::dot(p - a, ab) / glm::dot(ab, ab), 0.0, 1.0);
            best = std::min(best, glm::length(p - (a + ab * t)));
        }
        return best;
    }
};

struct Tooth {
    int fdi;
    glm::dvec2 center;
    glm::dvec2 tangent;
    double mesioDistal; // crown width along the arch
    double buccoLingual;
    double crownHeight;
};

double sdEllipsoid(const glm::dvec3& p, const glm::dvec3& r)
{
    // Approximate signed distance (Inigo Quilez).
    const double k0 = glm::length(p / r);
    const double k1 = glm::length(p / (r * r));
    return k0 * (k0 - 1.0) / std::max(k1, 1e-9);
}

double sdCapsule(const glm::dvec3& p, const glm::dvec3& a, const glm::dvec3& b, double r)
{
    const glm::dvec3 pa = p - a, ba = b - a;
    const double h = std::clamp(glm::dot(pa, ba) / glm::dot(ba, ba), 0.0, 1.0);
    return glm::length(pa - ba * h) - r;
}

double smin(double a, double b, double k)
{
    const double h = std::clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
    return glm::mix(b, a, h) - k * h * (1.0 - h);
}

double ellipse2d(const glm::dvec2& p, double rx, double ry)
{
    return (glm::length(p / glm::dvec2(rx, ry)) - 1.0) * std::min(rx, ry);
}

// Shoulder preparation in the tooth frame (x mesio-distal, y bucco-lingual, z up from the gum line).
struct Prep {
    double marginZ = kGumZ + 1.4;
    double rx = 0.0, ry = 0.0;   // margin ellipse (outer rim of the shoulder)
    double shoulder = 1.0;       // shoulder width
    double height = 4.5;         // axial wall height above the margin
    double taper = 0.6;          // radius reduction over the wall height (about 6 degrees per side)
    double rounding = 0.6;

    double sd(const glm::dvec3& q) const
    {
        const double top = marginZ + height;
        const double f = std::clamp((q.z - marginZ) / height, 0.0, 1.0);
        const double e = ellipse2d(glm::dvec2(q), rx - shoulder - taper * f, ry - shoulder - taper * f);
        const double qx = std::max(e + rounding, 0.0), qz = std::max(q.z - top + rounding, 0.0);
        double wall = std::sqrt(qx * qx + qz * qz) - rounding + std::min(std::max(e + rounding, q.z - top + rounding), 0.0);
        wall = std::max(wall, (kGumZ - 8.0) - q.z);
        const double stump = std::max({ellipse2d(glm::dvec2(q), rx, ry), q.z - marginZ, (kGumZ - 8.0) - q.z});
        return std::min(wall, stump);
    }
};

class Phantom {
public:
    // crownCase: complete dentition with equal crown heights and a prepared 46 (crown & bridge data).
    explicit Phantom(bool crownCase = false) : crownCase_(crownCase)
    {
        // Lower jaw teeth: crown widths (mm) from central incisor to second molar.
        const double widths[7] = {5.4, 5.9, 6.9, 7.0, 7.1, 11.0, 10.5};
        const double bl[7] = {6.0, 6.2, 7.6, 7.6, 8.0, 10.2, 9.8};
        const double mid = arch_.length() * 0.5;
        for (int side = 0; side < 2; ++side) {
            double s = 0.0;
            for (int k = 0; k < 7; ++k) {
                const double gap = 0.25;
                const double centerS = s + widths[k] * 0.5;
                s += widths[k] + gap;
                const int quadrant = side == 0 ? 3 : 4; // 3 = patient left (+X)
                const int fdi = quadrant * 10 + (k + 1);
                if (fdi == 36 && !crownCase)
                    continue; // missing first molar: implant site
                const double arcPos = side == 0 ? mid + centerS : mid - centerS;
                auto [c, t] = arch_.at(arcPos);
                teeth_.push_back(Tooth{fdi, c, t, widths[k], bl[k], (k < 3 && !crownCase) ? 9.0 : 7.0});
            }
        }
    }

    double sdTooth(const glm::dvec3& p, bool enamelOnly = false, const std::vector<int>& skip = {}) const
    {
        double d = 1e9;
        for (const auto& t : teeth_) {
            if (std::find(skip.begin(), skip.end(), t.fdi) != skip.end())
                continue;
            const glm::dvec2 rel2 = glm::dvec2(p) - t.center;
            if (glm::dot(rel2, rel2) > 200.0)
                continue;
            const glm::dvec2 n(-t.tangent.y, t.tangent.x);
            const glm::dvec3 local(glm::dot(rel2, t.tangent), glm::dot(rel2, n), p.z - (kGumZ + t.crownHeight * 0.45));
            double crown = sdEllipsoid(local, glm::dvec3(t.mesioDistal * 0.5, t.buccoLingual * 0.5, t.crownHeight * 0.62));
            // Occlusal grooves / cusps: modulate the top for registration landmarks.
            if (t.mesioDistal > 9.0)
                crown += 0.35 * std::sin(local.x * 1.2) * std::sin(local.y * 1.4) * std::clamp(local.z / 3.0, 0.0, 1.0);
            if (enamelOnly) {
                // Enamel: outer 1.1 mm of the crown above the gum line.
                const double inner = -(crown + 1.1);
                d = std::min(d, std::max(std::max(crown, inner), kGumZ + 0.5 - p.z));
                continue;
            }
            const glm::dvec3 rootTop(t.center, kGumZ + 1.0), rootBottom(t.center, kGumZ - (t.mesioDistal > 9.0 ? 13.0 : 15.0));
            const double root = sdCapsule(p, rootTop, rootBottom, std::min(t.mesioDistal, t.buccoLingual) * 0.28);
            d = std::min(d, smin(crown, root, 1.5));
        }
        return d;
    }

    double sdBone(const glm::dvec3& p, double archDist) const
    {
        const double body = archDist - 6.5;
        const double vertical = std::max((kGumZ - 24.0) - p.z, p.z - (kGumZ - 1.8));
        return std::max(body, vertical);
    }

    double sdGum(const glm::dvec3& p, double archDist) const
    {
        const double body = archDist - 7.8;
        const double vertical = std::max((kGumZ - 26.0) - p.z, p.z - (kGumZ + 0.8 - 0.06 * archDist * archDist));
        return std::max(body, vertical);
    }

    double sdHead(const glm::dvec3& p) const { return sdEllipsoid(p - glm::dvec3(0, 4, -8), glm::dvec3(46, 44, 34)); }

    const Arch& arch() const { return arch_; }

    const Tooth& tooth(int fdi) const
    {
        for (const auto& t : teeth_)
            if (t.fdi == fdi)
                return t;
        throw std::runtime_error("no such tooth");
    }

    // Tooth frame -> world (x = tangent along the arch, y = arch normal, z = up).
    glm::dmat4 toothFrame(int fdi) const
    {
        const Tooth& t = tooth(fdi);
        const glm::dvec2 n(-t.tangent.y, t.tangent.x);
        glm::dmat4 m(1.0);
        m[0] = glm::dvec4(t.tangent, 0.0, 0.0);
        m[1] = glm::dvec4(n, 0.0, 0.0);
        m[2] = glm::dvec4(0.0, 0.0, 1.0, 0.0);
        m[3] = glm::dvec4(t.center, 0.0, 1.0);
        return m;
    }

    // The preparation of `fdi`, sized from the natural crown at the margin height.
    Prep prepFor(int fdi) const
    {
        const Tooth& t = tooth(fdi);
        Prep pr;
        const double cz = kGumZ + t.crownHeight * 0.45, rz = t.crownHeight * 0.62;
        const double f = std::sqrt(std::max(0.0, 1.0 - std::pow((pr.marginZ - cz) / rz, 2.0)));
        pr.rx = t.mesioDistal * 0.5 * f;
        pr.ry = t.buccoLingual * 0.5 * f;
        return pr;
    }

    bool crownCase() const { return crownCase_; }

private:
    Arch arch_;
    std::vector<Tooth> teeth_;
    bool crownCase_ = false;
};

double ramp(double sd, double width)
{
    return std::clamp(0.5 - sd / width, 0.0, 1.0);
}

Volume makeCbct(const Phantom& ph, double voxel, unsigned seed)
{
    Volume v;
    const glm::dvec3 lo(-36.0, -32.0, -30.0), hi(36.0, 28.0, 16.0);
    v.geometry.spacing = glm::dvec3(voxel);
    v.geometry.dims = glm::ivec3(glm::ceil((hi - lo) / voxel));
    v.geometry.origin = lo;
    v.rescaleSlope = 1.0;
    v.rescaleIntercept = -1000.0;
    v.info.windowCenter = 600.0;
    v.info.windowWidth = 3500.0;
    v.voxels.resize(v.geometry.voxelCount());
    const auto d = v.geometry.dims;
    log::info("Generating CBCT {}x{}x{} at {:.2f} mm", d.x, d.y, d.z, voxel);

    parallelFor(static_cast<std::size_t>(d.z), [&](std::size_t zi) {
        std::mt19937 rng(seed + static_cast<unsigned>(zi) * 7919u);
        std::normal_distribution<double> noise(0.0, 35.0);
        const int z = static_cast<int>(zi);
        for (int y = 0; y < d.y; ++y) {
            for (int x = 0; x < d.x; ++x) {
                const glm::dvec3 p = lo + glm::dvec3(x, y, z) * voxel;
                double hu = -1000.0;
                hu = glm::mix(hu, 40.0, ramp(ph.sdHead(p), voxel));
                if (p.z > -30.0 && p.z < 14.0) {
                    const double archDist = ph.arch().distance(glm::dvec2(p));
                    if (archDist < 12.0) {
                        const double bone = ph.sdBone(p, archDist);
                        // Cortical shell is denser than the cancellous core.
                        const double boneHu = bone > -1.2 ? 1500.0 : 750.0;
                        hu = glm::mix(hu, boneHu, ramp(bone, voxel));
                        hu = glm::mix(hu, 1900.0, ramp(ph.sdTooth(p), voxel));
                        hu = glm::mix(hu, 2700.0, ramp(ph.sdTooth(p, true), voxel));
                    }
                }
                hu += noise(rng);
                v.voxels[v.index(x, y, z)] = static_cast<std::int16_t>(std::clamp(std::lround(hu + 1000.0), -32768L, 32767L));
            }
        }
    });
    return v;
}

template <class SdfFn>
Mesh surfaceFromSdf(const glm::dvec3& lo, const glm::dvec3& hi, double res, SdfFn&& sdfAt, bool openBottom)
{
    Volume sdf;
    sdf.geometry.spacing = glm::dvec3(res);
    sdf.geometry.dims = glm::ivec3(glm::ceil((hi - lo) / res));
    sdf.geometry.origin = lo;
    sdf.voxels.resize(sdf.geometry.voxelCount());
    const auto d = sdf.geometry.dims;
    parallelFor(static_cast<std::size_t>(d.z), [&](std::size_t zi) {
        const int z = static_cast<int>(zi);
        for (int y = 0; y < d.y; ++y)
            for (int x = 0; x < d.x; ++x) {
                const double sd = sdfAt(lo + glm::dvec3(x, y, z) * res);
                sdf.voxels[sdf.index(x, y, z)] = static_cast<std::int16_t>(std::clamp(-sd * 1000.0, -32000.0, 32000.0));
            }
    });
    IsoSurfaceOptions opt;
    opt.isoValue = 0.0;
    Mesh m = extractIsoSurface(sdf, opt);
    // Drop the flat cap at the bottom of the sampled block so the scan is open like a real one.
    Mesh out;
    std::vector<std::int64_t> remap(m.positions.size(), -1);
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        bool keep = true;
        for (int k = 0; k < 3; ++k) {
            const float z = m.positions[m.indices[t + k]].z;
            if (openBottom ? z < static_cast<float>(lo.z + 1.0) : z > static_cast<float>(hi.z - 1.0))
                keep = false;
        }
        if (!keep)
            continue;
        for (int k = 0; k < 3; ++k) {
            const auto i = m.indices[t + k];
            if (remap[i] < 0) {
                remap[i] = static_cast<std::int64_t>(out.positions.size());
                out.positions.push_back(m.positions[i]);
            }
            out.indices.push_back(static_cast<std::uint32_t>(remap[i]));
        }
    }
    out.computeVertexNormals();
    return out;
}

Mesh makeScan(const Phantom& ph)
{
    // Scanned surface = teeth crowns + gingiva, only the occlusal part a scanner sees.
    return surfaceFromSdf(glm::dvec3(-36.0, -32.0, kGumZ - 6.0), glm::dvec3(36.0, 24.0, kGumZ + 11.0), 0.2,
                          [&](const glm::dvec3& p) {
                              const double archDist = ph.arch().distance(glm::dvec2(p));
                              return archDist < 14.0 ? smin(ph.sdTooth(p), ph.sdGum(p, archDist), 0.6) : 50.0;
                          },
                          true);
}

// Crown & bridge data: lower scan with the prepared teeth, upper scan in occlusion (mirrored
// natural dentition), in world (phantom) coordinates. 46 is prepared for a single crown; 35 and
// 37 are bridge abutments with 36 missing (edentulous ridge).
constexpr int kPrepTooth = 46;
const std::vector<int> kBridgeAbutments = {37, 35};
constexpr int kBridgePontic = 36;
constexpr double kOcclusalPlane = kGumZ + 7.0 * 1.07 + 0.15; // cusp tips of both jaws 0.3 mm apart

Mesh makePrepScan(const Phantom& ph)
{
    struct Prepared {
        glm::dmat4 toTooth;
        Prep prep;
    };
    std::vector<int> prepared = {kPrepTooth};
    prepared.insert(prepared.end(), kBridgeAbutments.begin(), kBridgeAbutments.end());
    std::vector<Prepared> preps;
    for (int fdi : prepared)
        preps.push_back({glm::inverse(ph.toothFrame(fdi)), ph.prepFor(fdi)});
    std::vector<int> removed = prepared;
    removed.push_back(kBridgePontic);
    return surfaceFromSdf(glm::dvec3(-36.0, -32.0, kGumZ - 6.0), glm::dvec3(36.0, 24.0, kGumZ + 11.0), 0.2,
                          [&](const glm::dvec3& p) {
                              const double archDist = ph.arch().distance(glm::dvec2(p));
                              if (archDist >= 14.0)
                                  return 50.0;
                              double d = smin(ph.sdTooth(p, false, removed), ph.sdGum(p, archDist), 0.6);
                              for (const auto& pr : preps)
                                  d = smin(d, pr.prep.sd(transformPoint(pr.toTooth, p)), 0.12);
                              return d;
                          },
                          true);
}

Mesh makeAntagonistScan(const Phantom& ph)
{
    auto mirror = [](const glm::dvec3& p) { return glm::dvec3(p.x, p.y, 2.0 * kOcclusalPlane - p.z); };
    const double zTop = 2.0 * kOcclusalPlane - (kGumZ - 6.0), zBottom = 2.0 * kOcclusalPlane - (kGumZ + 11.0);
    Mesh m = surfaceFromSdf(glm::dvec3(-36.0, -32.0, zBottom), glm::dvec3(36.0, 24.0, zTop), 0.2,
                            [&](const glm::dvec3& p) {
                                const glm::dvec3 q = mirror(p);
                                const double archDist = ph.arch().distance(glm::dvec2(q));
                                return archDist < 14.0 ? smin(ph.sdTooth(q), ph.sdGum(q, archDist), 0.6) : 50.0;
                            },
                            false);
    return m;
}

int createCases(const fs::path& dataRoot, const fs::path& dicomDir, const fs::path& scanPath, const fs::path& crownDir, int extra)
{
    AppConfig cfg;
    cfg.dataRoot = dataRoot;
    auto repo = db::openRepository(cfg);

    db::CaseRecord c;
    c.patientFirstName = "Demo";
    c.patientLastName = "Phantom";
    c.patientBirthDate = "1975-06-14";
    c.patientReference = "PH-0001";
    c.practice = "Riverside Dental Group";
    c.dentist = "Dr. A. Moreno";
    c.technician = platform::userName();
    c.dueDate = time::todayLocalDate();
    c.workflow = "implant_planning";
    c.notes = "Synthetic phantom: missing 36, plan one implant. Ground truth registration in phantom output folder.";
    c.restorations.push_back(db::Restoration{0, 36, "implant_planning", "", "", "Generic 4.1 x 10 mm", ""});
    c = repo->createCase(c);
    const std::string dicomRel = repo->files().importDirectory(c, dicomDir, db::FileRole::Dicom);
    const std::string scanRel = repo->files().importFile(c, scanPath, db::FileRole::ScanLower);
    c.files.push_back(db::CaseFile{0, db::FileRole::Dicom, dicomRel, "Synthetic CBCT", ""});
    c.files.push_back(db::CaseFile{0, db::FileRole::ScanLower, scanRel, "Lower jaw scan", ""});
    repo->updateCase(c);
    std::printf("Created demo case %s (%s)\n", c.caseNumber.c_str(), c.uuid.c_str());

    // Crown & bridge case with a prepared 46.
    db::CaseRecord cc;
    cc.patientFirstName = "Crown";
    cc.patientLastName = "Phantom";
    cc.patientBirthDate = "1981-02-03";
    cc.patientReference = "PH-0002";
    cc.practice = "Riverside Dental Group";
    cc.dentist = "Dr. A. Moreno";
    cc.technician = platform::userName();
    cc.dueDate = time::todayLocalDate();
    cc.notes = "Synthetic phantom: shoulder preparation on 46, upper jaw as antagonist. Ground-truth margin in crown/crown_truth.json.";
    cc.restorations.push_back(db::Restoration{0, kPrepTooth, "anatomic_crown", "Zirconia", "A2", "", ""});
    cc = repo->createCase(cc);
    const std::string lowerRel = repo->files().importFile(cc, crownDir / "lower_prep.stl", db::FileRole::ScanLower);
    const std::string upperRel = repo->files().importFile(cc, crownDir / "upper.stl", db::FileRole::ScanUpper);
    cc.files.push_back(db::CaseFile{0, db::FileRole::ScanLower, lowerRel, "Lower jaw (preparation)", ""});
    cc.files.push_back(db::CaseFile{0, db::FileRole::ScanUpper, upperRel, "Upper jaw", ""});
    repo->updateCase(cc);
    std::printf("Created crown demo case %s (%s)\n", cc.caseNumber.c_str(), cc.uuid.c_str());

    // Three-unit bridge 35-36-37 on the same scans.
    db::CaseRecord bc;
    bc.patientFirstName = "Bridge";
    bc.patientLastName = "Phantom";
    bc.patientBirthDate = "1968-09-21";
    bc.patientReference = "PH-0003";
    bc.practice = "Riverside Dental Group";
    bc.dentist = "Dr. A. Moreno";
    bc.technician = platform::userName();
    bc.dueDate = time::todayLocalDate();
    bc.notes = "Synthetic phantom: three-unit bridge 35-37 with 36 as pontic. Ground-truth margins in crown/bridge_truth.json.";
    bc.restorations.push_back(db::Restoration{0, 35, "anatomic_crown", "Zirconia", "A3", "", ""});
    bc.restorations.push_back(db::Restoration{0, kBridgePontic, "pontic", "Zirconia", "A3", "", ""});
    bc.restorations.push_back(db::Restoration{0, 37, "anatomic_crown", "Zirconia", "A3", "", ""});
    bc = repo->createCase(bc);
    const std::string bLower = repo->files().importFile(bc, crownDir / "lower_prep.stl", db::FileRole::ScanLower);
    const std::string bUpper = repo->files().importFile(bc, crownDir / "upper.stl", db::FileRole::ScanUpper);
    bc.files.push_back(db::CaseFile{0, db::FileRole::ScanLower, bLower, "Lower jaw (preparations)", ""});
    bc.files.push_back(db::CaseFile{0, db::FileRole::ScanUpper, bUpper, "Upper jaw", ""});
    repo->updateCase(bc);
    std::printf("Created bridge demo case %s (%s)\n", bc.caseNumber.c_str(), bc.uuid.c_str());

    // Additional cases without data for a realistic case list.
    struct Demo {
        const char* first;
        const char* last;
        const char* practice;
        std::vector<std::pair<int, const char*>> teeth;
        db::CaseStatus status;
    };
    const std::vector<Demo> demos = {
        {"Lena", "Fischer", "Lakeside Orthodontics", {{14, "surgical_guide"}, {15, "surgical_guide"}}, db::CaseStatus::InDesign},
        {"Marco", "Rossi", "Riverside Dental Group", {{11, "anatomic_crown"}, {21, "anatomic_crown"}}, db::CaseStatus::Designed},
        {"Aiko", "Tanaka", "Northgate Smiles", {{46, "custom_abutment"}}, db::CaseStatus::New},
        {"Samuel", "Okafor", "City Dental Studio", {{24, "implant_planning"}, {25, "pontic"}, {26, "implant_planning"}}, db::CaseStatus::Exported},
        {"Elena", "Novak", "Lakeside Orthodontics", {{36, "inlay_onlay"}}, db::CaseStatus::New},
        {"James", "O'Neill", "Northgate Smiles", {{12, "veneer"}, {11, "veneer"}, {21, "veneer"}, {22, "veneer"}}, db::CaseStatus::Designed},
    };
    for (int i = 0; i < extra && i < static_cast<int>(demos.size()); ++i) {
        db::CaseRecord r;
        r.patientFirstName = demos[i].first;
        r.patientLastName = demos[i].last;
        r.practice = demos[i].practice;
        r.status = demos[i].status;
        r.technician = platform::userName();
        std::vector<std::string> keys;
        for (const auto& [tooth, type] : demos[i].teeth) {
            r.restorations.push_back(db::Restoration{0, tooth, type, "", "", "", ""});
            keys.emplace_back(type);
        }
        r.workflow = "";
        r = repo->createCase(r);
    }
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    const CommandLine cl = CommandLine::fromMain(argc, argv);
    if (!cl.has("out")) {
        std::fprintf(stderr, "usage: occlusa_phantom --out <dir> [--voxel 0.4] [--seed 7] [--create-case <dataRoot>] [--extra-cases N]\n");
        return 2;
    }
    try {
        const fs::path out = platform::pathFromUtf8(*cl.get("out"));
        const double voxel = std::stod(cl.get("voxel").value_or("0.4"));
        const unsigned seed = static_cast<unsigned>(std::stoul(cl.get("seed").value_or("7")));
        fs::create_directories(out);

        Phantom ph;
        const Volume cbct = makeCbct(ph, voxel, seed);
        const fs::path dicomDir = out / "dicom";
        fs::remove_all(dicomDir);
        dicom::WriteOptions wopt;
        wopt.patientName = "PHANTOM^DEMO";
        wopt.seriesDescription = "Synthetic CBCT lower jaw";
        dicom::writeSeries(cbct, dicomDir, wopt);
        std::printf("Wrote %d DICOM slices to %s\n", cbct.geometry.dims.z, dicomDir.string().c_str());

        // Scan in scanner coordinates: scanLocal = inverse(truth) * world.
        Mesh scan = makeScan(ph);
        std::mt19937 rng(seed * 31u + 5u);
        std::uniform_real_distribution<double> u(-1.0, 1.0);
        const glm::dvec3 axis = glm::normalize(glm::dvec3(u(rng), u(rng), u(rng)));
        glm::dmat4 truth = glm::rotate(glm::dmat4(1.0), glm::radians(25.0 + 20.0 * u(rng)), axis);
        truth[3] = glm::dvec4(18.0 * u(rng), 18.0 * u(rng), 12.0 * u(rng), 1.0);
        const glm::dmat4 inv = glm::inverse(truth);
        for (auto& p : scan.positions)
            p = glm::vec3(transformPoint(inv, glm::dvec3(p)));
        scan.computeVertexNormals();
        const fs::path scanPath = out / "lower_scan.stl";
        writeStlBinary(scanPath, scan, glm::dmat4(1.0), "OcclusaCAD phantom lower scan");
        std::printf("Wrote scan with %zu triangles to %s\n", scan.triangleCount(), scanPath.string().c_str());

        nlohmann::json gt;
        std::vector<double> m;
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                m.push_back(truth[c][r]);
        gt["scanToCbct_columnMajor"] = m;
        gt["note"] = "Maps lower_scan.stl coordinates into the CBCT patient coordinate system (LPS, mm).";
        std::ofstream(out / "ground_truth.json") << gt.dump(2);

        // Crown & bridge data set, in its own scanner coordinates (both jaws share the pose).
        const fs::path crownDir = out / "crown";
        fs::create_directories(crownDir);
        {
            const Phantom cp(true);
            glm::dmat4 pose = glm::rotate(glm::dmat4(1.0), glm::radians(30.0 + 25.0 * u(rng)), glm::normalize(glm::dvec3(u(rng), u(rng), u(rng))));
            pose[3] = glm::dvec4(10.0 * u(rng), 10.0 * u(rng), 10.0 * u(rng), 1.0);
            const glm::dmat4 toScanner = glm::inverse(pose);
            auto save = [&](Mesh mesh, const char* name, const char* header) {
                for (auto& p : mesh.positions)
                    p = glm::vec3(transformPoint(toScanner, glm::dvec3(p)));
                mesh.computeVertexNormals();
                writeStlBinary(crownDir / name, mesh, glm::dmat4(1.0), header);
                std::printf("Wrote %s (%zu triangles)\n", (crownDir / name).string().c_str(), mesh.triangleCount());
            };
            save(makePrepScan(cp), "lower_prep.stl", "OcclusaCAD phantom lower jaw, 46 prepared");
            save(makeAntagonistScan(cp), "upper.stl", "OcclusaCAD phantom upper jaw");
            // Ground truth of a preparation: top point, axis and the analytic margin (scanner coordinates).
            auto prepTruth = [&](int fdi) {
                const glm::dmat4 toothToScanner = toScanner * cp.toothFrame(fdi);
                const Prep prep = cp.prepFor(fdi);
                nlohmann::json ct;
                ct["tooth"] = fdi;
                const glm::dvec3 top = transformPoint(toothToScanner, glm::dvec3(0.0, 0.0, prep.marginZ + prep.height));
                ct["prepPoint"] = {top.x, top.y, top.z};
                const glm::dvec3 ax = glm::normalize(transformVector(toothToScanner, glm::dvec3(0, 0, 1)));
                ct["axis"] = {ax.x, ax.y, ax.z};
                nlohmann::json margin = nlohmann::json::array();
                for (int i = 0; i < 360; ++i) {
                    const double a = i * 2.0 * 3.14159265358979 / 360.0;
                    const glm::dvec3 q = transformPoint(toothToScanner, glm::dvec3(prep.rx * std::cos(a), prep.ry * std::sin(a), prep.marginZ));
                    margin.push_back({q.x, q.y, q.z});
                }
                ct["margin"] = margin;
                return ct;
            };
            nlohmann::json ct = prepTruth(kPrepTooth);
            ct["note"] = "Coordinates of lower_prep.stl / upper.stl (mm). margin: outer rim of the shoulder; prepPoint: top of the preparation.";
            std::ofstream(crownDir / "crown_truth.json") << ct.dump(1);
            nlohmann::json bt;
            bt["note"] = "Bridge 35-36-37 (36 pontic). Coordinates of lower_prep.stl / upper.stl (mm).";
            bt["bridge"] = {37, 36, 35};
            bt["preps"] = nlohmann::json::array();
            for (int fdi : kBridgeAbutments)
                bt["preps"].push_back(prepTruth(fdi));
            std::ofstream(crownDir / "bridge_truth.json") << bt.dump(1);
        }

        if (auto root = cl.get("create-case"))
            return createCases(platform::pathFromUtf8(*root), dicomDir, scanPath, crownDir, std::stoi(cl.get("extra-cases").value_or("6")));
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
