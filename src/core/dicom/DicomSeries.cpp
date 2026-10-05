#include "core/dicom/DicomSeries.h"

#include "core/Log.h"
#include "core/Parallel.h"
#include "core/Platform.h"
#include "core/dicom/DicomCodecs.h"
#include "core/dicom/DicomDataset.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <format>
#include <map>
#include <mutex>
#include <optional>

namespace fs = std::filesystem;

namespace occlusa::dicom {
namespace {

constexpr std::size_t kHeaderProbeBytes = 256 * 1024;

bool skipByExtension(const fs::path& p)
{
    std::string ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    static constexpr const char* kSkip[] = {".jpg", ".jpeg", ".png", ".bmp", ".gif", ".txt", ".pdf", ".xml", ".html", ".htm", ".ini",
                                            ".exe", ".dll", ".so", ".dylib", ".stl", ".ply", ".obj", ".zip", ".json", ".csv", ".log",
                                            ".js", ".css", ".inf", ".bat", ".sh", ".md", ".db", ".sqlite3", ".ds_store"};
    for (const char* s : kSkip)
        if (ext == s)
            return true;
    const std::string name = p.filename().string();
    return name == "DICOMDIR" || name == ".DS_Store";
}

// Parse header-only; re-read the whole file if the probe was too short.
ParsedFile parseHeader(const fs::path& path)
{
    ParseOptions opts;
    opts.stopBeforePixelData = true;
    opts.maxBytes = kHeaderProbeBytes;
    ParsedFile pf = parseFile(path, opts);
    if (pf.truncated) {
        opts.maxBytes = 0;
        pf = parseFile(path, opts);
    }
    return pf;
}

std::string formatPersonName(const std::string& pn)
{
    // DICOM PN: Family^Given^Middle^Prefix^Suffix (only the alphabetic group is used).
    const std::string alpha = pn.substr(0, pn.find('='));
    std::vector<std::string> parts;
    std::size_t start = 0;
    for (;;) {
        const std::size_t p = alpha.find('^', start);
        parts.push_back(alpha.substr(start, p == std::string::npos ? std::string::npos : p - start));
        if (p == std::string::npos)
            break;
        start = p + 1;
    }
    if (parts.size() >= 2 && !parts[1].empty())
        return parts[0] + ", " + parts[1];
    return parts.empty() ? pn : parts[0];
}

std::optional<std::array<double, 6>> orientationOf(const Dataset& ds)
{
    auto v = ds.getNumbers(tags::ImageOrientationPatient);
    if (v.size() < 6)
        return std::nullopt;
    return std::array<double, 6>{v[0], v[1], v[2], v[3], v[4], v[5]};
}

// Look up a functional group attribute: per-frame first, then shared, then top level.
const Dataset* functionalGroup(const Dataset& ds, std::size_t frame, Tag sequence)
{
    if (const Dataset* perFrame = ds.item(tags::PerFrameFunctionalGroupsSequence, frame))
        if (const Dataset* g = perFrame->item(sequence))
            return g;
    if (const Dataset* shared = ds.item(tags::SharedFunctionalGroupsSequence))
        if (const Dataset* g = shared->item(sequence))
            return g;
    return nullptr;
}

struct ImageHeader {
    fs::path path;
    std::string seriesUid;
    std::string stackKey;
    std::string description;
    std::string patientName;
    std::string patientId;
    std::string studyDate;
    std::string modality;
    std::string transferSyntax;
    int rows = 0;
    int cols = 0;
    int frames = 1;
};

std::optional<ImageHeader> readImageHeader(const fs::path& path)
{
    ParsedFile pf = parseHeader(path);
    const Dataset& ds = pf.data;
    if (!ds.has(tags::PixelData))
        return std::nullopt;
    const auto rows = ds.getInt(tags::Rows), cols = ds.getInt(tags::Columns);
    if (!rows || !cols || *rows <= 0 || *cols <= 0)
        return std::nullopt;

    ImageHeader h;
    h.path = path;
    h.rows = static_cast<int>(*rows);
    h.cols = static_cast<int>(*cols);
    h.frames = static_cast<int>(std::max<long long>(1, ds.getInt(tags::NumberOfFrames).value_or(1)));
    h.seriesUid = ds.getString(tags::SeriesInstanceUID, "unknown-series");
    h.description = ds.getString(tags::SeriesDescription, "");
    h.patientName = formatPersonName(ds.getString(tags::PatientName, ""));
    h.patientId = ds.getString(tags::PatientID, "");
    h.studyDate = ds.getString(tags::StudyDate, "");
    h.modality = ds.getString(tags::Modality, "");
    h.transferSyntax = pf.transferSyntax;

    std::string orient = "none";
    std::optional<std::array<double, 6>> iop = orientationOf(ds);
    if (!iop) {
        if (const Dataset* g = functionalGroup(ds, 0, tags::PlaneOrientationSequence))
            iop = orientationOf(*g);
    }
    if (iop)
        orient = std::format("{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}", (*iop)[0], (*iop)[1], (*iop)[2], (*iop)[3], (*iop)[4], (*iop)[5]);
    h.stackKey = std::format("{}|{}|{}x{}", h.seriesUid, orient, h.rows, h.cols);
    if (h.frames > 1)
        h.stackKey += "|" + ds.getString(tags::SOPInstanceUID, path.string()); // each multi-frame object is its own stack
    return h;
}

// ---------------------------------------------------------------------------

struct FrameRef {
    std::size_t file = 0;
    int frame = 0;
    std::optional<glm::dvec3> position;
    double instance = 0.0;
    double slope = 1.0;
    double intercept = 0.0;
    double sortKey = 0.0;
};

struct PixelFormat {
    int rows = 0, cols = 0;
    int bitsAllocated = 16, bitsStored = 16, highBit = 15;
    int pixelRepresentation = 0;
    int samplesPerPixel = 1;
    bool monochrome1 = false;
};

std::vector<std::span<const std::uint8_t>> splitFrames(const Element& pixel, int frames)
{
    std::vector<std::span<const std::uint8_t>> out;
    if (pixel.fragments.empty())
        return out;
    const auto bot = pixel.fragments[0];
    std::vector<std::span<const std::uint8_t>> frags(pixel.fragments.begin() + 1, pixel.fragments.end());
    if (frags.empty())
        return out;
    if (frames == 1) {
        if (frags.size() == 1) {
            out.push_back(frags[0]);
        } else {
            // Fragments of one frame are contiguous in the buffer apart from the 8 byte item headers;
            // concatenation happens in the caller via a copy.
            out.push_back({}); // marker: concatenate all
        }
        return out;
    }
    if (static_cast<int>(frags.size()) == frames) {
        return frags;
    }
    if (bot.size() >= 4u * static_cast<std::size_t>(frames)) {
        // Basic offset table present: offsets are relative to the first fragment item tag.
        // Each fragment item has an 8 byte header.
        std::vector<std::uint32_t> offsets(static_cast<std::size_t>(frames));
        for (int f = 0; f < frames; ++f) {
            const std::uint8_t* p = bot.data() + 4 * f;
            offsets[f] = p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
        }
        // Map fragments to their byte offsets.
        std::vector<std::uint32_t> fragOffsets;
        std::uint32_t acc = 0;
        for (const auto& fr : frags) {
            fragOffsets.push_back(acc);
            acc += 8 + static_cast<std::uint32_t>(fr.size());
        }
        for (int f = 0; f < frames; ++f) {
            auto it = std::find(fragOffsets.begin(), fragOffsets.end(), offsets[f]);
            if (it == fragOffsets.end())
                throw DicomError("Basic offset table does not match fragments");
            const std::size_t first = static_cast<std::size_t>(it - fragOffsets.begin());
            const std::uint32_t endOff = (f + 1 < frames) ? offsets[f + 1] : acc;
            std::size_t last = first;
            while (last + 1 < frags.size() && fragOffsets[last + 1] < endOff)
                ++last;
            if (first == last)
                out.push_back(frags[first]);
            else
                throw DicomError("Multi-fragment frames in multi-frame objects are not supported yet"); // TODO
        }
        return out;
    }
    // No offset table: split on JPEG SOI markers.
    for (const auto& fr : frags) {
        if (fr.size() >= 2 && fr[0] == 0xFF && fr[1] == 0xD8)
            out.push_back(fr);
        else if (!out.empty())
            throw DicomError("Multi-fragment frames without offset table are not supported yet"); // TODO
    }
    if (static_cast<int>(out.size()) != frames)
        throw DicomError("Could not determine frame boundaries in encapsulated pixel data");
    return out;
}

} // namespace

std::string SeriesInfo::displayName() const
{
    std::string name = seriesDescription.empty() ? std::string("(no description)") : seriesDescription;
    return std::format("{} - {} [{}x{}x{}]", modality.empty() ? "?" : modality, name, columns, rows, sliceCount);
}

ScanResult scanForSeries(const fs::path& root, const ProgressFn& progress)
{
    ScanResult result;
    std::vector<fs::path> files;
    std::error_code ec;
    if (fs::is_regular_file(root, ec)) {
        files.push_back(root);
    } else {
        reportProgress(progress, 0.0f, "Listing files...");
        for (auto it = fs::recursive_directory_iterator(root, fs::directory_options::skip_permission_denied, ec);
             it != fs::recursive_directory_iterator(); it.increment(ec)) {
            if (ec)
                break;
            std::error_code fec;
            if (it->is_regular_file(fec) && !skipByExtension(it->path()))
                files.push_back(it->path());
        }
    }
    std::sort(files.begin(), files.end());

    std::vector<std::optional<ImageHeader>> headers(files.size());
    std::atomic<std::size_t> done{0};
    std::atomic<bool> cancelled{false};
    std::mutex progressMutex;
    parallelFor(files.size(), [&](std::size_t i) {
        if (cancelled)
            return;
        try {
            headers[i] = readImageHeader(files[i]);
        } catch (const std::exception&) {
            headers[i].reset();
        }
        const std::size_t d = ++done;
        if (progress && (d % 16 == 0 || d == files.size())) {
            std::lock_guard lock(progressMutex);
            if (!progress(static_cast<float>(d) / static_cast<float>(files.size()), std::format("Scanning {} / {} files", d, files.size())))
                cancelled = true;
        }
    });
    if (cancelled)
        throw CancelledError();

    std::map<std::string, SeriesInfo> stacks;
    for (auto& h : headers) {
        ++result.filesScanned;
        if (!h) {
            ++result.filesSkipped;
            continue;
        }
        SeriesInfo& s = stacks[h->stackKey];
        if (s.files.empty()) {
            s.seriesInstanceUid = h->seriesUid;
            s.stackKey = h->stackKey;
            s.seriesDescription = h->description;
            s.patientName = h->patientName;
            s.patientId = h->patientId;
            s.studyDate = h->studyDate;
            s.modality = h->modality;
            s.transferSyntax = h->transferSyntax;
            s.rows = h->rows;
            s.columns = h->cols;
            s.multiFrame = h->frames > 1;
            s.supported = isTransferSyntaxSupported(h->transferSyntax);
        }
        s.files.push_back(h->path);
        s.sliceCount += h->frames;
    }
    for (auto& [key, s] : stacks)
        result.series.push_back(std::move(s));
    // Largest stacks first: the volume of interest is almost always the biggest.
    std::sort(result.series.begin(), result.series.end(), [](const SeriesInfo& a, const SeriesInfo& b) {
        return static_cast<long long>(a.rows) * a.columns * a.sliceCount > static_cast<long long>(b.rows) * b.columns * b.sliceCount;
    });
    return result;
}

Volume loadSeries(const SeriesInfo& series, const ProgressFn& progress, LoadReport* report)
{
    auto warn = [&](const std::string& msg) {
        log::warn("DICOM: {}", msg);
        if (report)
            report->warnings.push_back(msg);
    };
    if (series.files.empty())
        throw DicomError("Series contains no files");
    if (!series.supported)
        throw DicomError("Unsupported transfer syntax: " + transferSyntaxName(series.transferSyntax));

    // ---- Pass 1: headers --------------------------------------------------
    std::vector<ParsedFile> headers(series.files.size());
    std::atomic<std::size_t> done{0};
    std::atomic<bool> cancelled{false};
    std::mutex progressMutex;
    auto tick = [&](float base, float span, std::size_t total, const char* what) {
        const std::size_t d = ++done;
        if (progress && (d % 8 == 0 || d == total)) {
            std::lock_guard lock(progressMutex);
            if (!progress(base + span * static_cast<float>(d) / static_cast<float>(total), std::format("{} {} / {}", what, d, total)))
                cancelled = true;
        }
    };
    parallelFor(series.files.size(), [&](std::size_t i) {
        if (cancelled)
            return;
        headers[i] = parseHeader(series.files[i]);
        tick(0.0f, 0.2f, series.files.size(), "Reading headers");
    });
    if (cancelled)
        throw CancelledError();

    const Dataset& first = headers[0].data;
    PixelFormat fmt;
    fmt.rows = static_cast<int>(first.getInt(tags::Rows).value_or(0));
    fmt.cols = static_cast<int>(first.getInt(tags::Columns).value_or(0));
    fmt.bitsAllocated = static_cast<int>(first.getInt(tags::BitsAllocated).value_or(16));
    fmt.bitsStored = static_cast<int>(first.getInt(tags::BitsStored).value_or(fmt.bitsAllocated));
    fmt.highBit = static_cast<int>(first.getInt(tags::HighBit).value_or(fmt.bitsStored - 1));
    fmt.pixelRepresentation = static_cast<int>(first.getInt(tags::PixelRepresentation).value_or(0));
    fmt.samplesPerPixel = static_cast<int>(first.getInt(tags::SamplesPerPixel).value_or(1));
    fmt.monochrome1 = first.getString(tags::PhotometricInterpretation, "") == "MONOCHROME1";
    if (fmt.samplesPerPixel != 1)
        throw DicomError("Colour DICOM images are not supported as volumes");
    if (fmt.bitsAllocated != 8 && fmt.bitsAllocated != 16 && fmt.bitsAllocated != 32)
        throw DicomError(std::format("Unsupported BitsAllocated: {}", fmt.bitsAllocated));
    if (fmt.rows <= 0 || fmt.cols <= 0)
        throw DicomError("Invalid image dimensions");
    if (fmt.bitsStored <= 0 || fmt.bitsStored > fmt.bitsAllocated)
        fmt.bitsStored = fmt.bitsAllocated;
    if (fmt.monochrome1)
        warn("MONOCHROME1 data: values inverted");

    // Orientation, spacing.
    std::optional<std::array<double, 6>> iop = orientationOf(first);
    if (!iop)
        if (const Dataset* g = functionalGroup(first, 0, tags::PlaneOrientationSequence))
            iop = orientationOf(*g);
    std::vector<double> pixelSpacing = first.getNumbers(tags::PixelSpacing);
    if (pixelSpacing.size() < 2)
        if (const Dataset* g = functionalGroup(first, 0, tags::PixelMeasuresSequence))
            pixelSpacing = g->getNumbers(tags::PixelSpacing);
    std::optional<double> sliceThickness = first.getNumber(tags::SliceThickness);
    std::optional<double> spacingBetween = first.getNumber(tags::SpacingBetweenSlices);
    if (const Dataset* g = functionalGroup(first, 0, tags::PixelMeasuresSequence)) {
        if (!sliceThickness)
            sliceThickness = g->getNumber(tags::SliceThickness);
        if (!spacingBetween)
            spacingBetween = g->getNumber(tags::SpacingBetweenSlices);
    }

    glm::dvec3 rowDir(1, 0, 0), colDir(0, 1, 0);
    if (iop) {
        rowDir = glm::normalize(glm::dvec3((*iop)[0], (*iop)[1], (*iop)[2]));
        colDir = glm::normalize(glm::dvec3((*iop)[3], (*iop)[4], (*iop)[5]));
    } else {
        warn("ImageOrientationPatient missing; assuming axial slices");
    }
    const glm::dvec3 normal = glm::normalize(glm::cross(rowDir, colDir));
    if (pixelSpacing.size() < 2) {
        warn("PixelSpacing missing; assuming 1 mm");
        pixelSpacing = {1.0, 1.0};
    }

    // Collect frames.
    std::vector<FrameRef> frames;
    for (std::size_t f = 0; f < headers.size(); ++f) {
        const Dataset& ds = headers[f].data;
        const int rows = static_cast<int>(ds.getInt(tags::Rows).value_or(0));
        const int cols = static_cast<int>(ds.getInt(tags::Columns).value_or(0));
        if (rows != fmt.rows || cols != fmt.cols || ds.getInt(tags::BitsAllocated).value_or(16) != fmt.bitsAllocated) {
            warn("Skipping image with different dimensions: " + platform::pathToUtf8(series.files[f].filename()));
            continue;
        }
        const int nFrames = static_cast<int>(std::max<long long>(1, ds.getInt(tags::NumberOfFrames).value_or(1)));
        const double baseSlope = ds.getNumber(tags::RescaleSlope).value_or(1.0);
        const double baseIntercept = ds.getNumber(tags::RescaleIntercept).value_or(0.0);
        const auto topIpp = ds.getNumbers(tags::ImagePositionPatient);
        for (int fr = 0; fr < nFrames; ++fr) {
            FrameRef ref;
            ref.file = f;
            ref.frame = fr;
            ref.instance = static_cast<double>(ds.getInt(tags::InstanceNumber).value_or(static_cast<long long>(f))) * 10000.0 + fr;
            ref.slope = baseSlope;
            ref.intercept = baseIntercept;
            if (nFrames > 1) {
                if (const Dataset* g = functionalGroup(ds, static_cast<std::size_t>(fr), tags::PlanePositionSequence)) {
                    auto p = g->getNumbers(tags::ImagePositionPatient);
                    if (p.size() >= 3)
                        ref.position = glm::dvec3(p[0], p[1], p[2]);
                }
                if (const Dataset* g = functionalGroup(ds, static_cast<std::size_t>(fr), tags::PixelValueTransformationSequence)) {
                    ref.slope = g->getNumber(tags::RescaleSlope).value_or(ref.slope);
                    ref.intercept = g->getNumber(tags::RescaleIntercept).value_or(ref.intercept);
                }
                if (!ref.position && topIpp.size() >= 3) {
                    // Legacy multi-frame: frames stacked along the normal from the top-level position.
                    const double dz = spacingBetween.value_or(sliceThickness.value_or(pixelSpacing[0]));
                    ref.position = glm::dvec3(topIpp[0], topIpp[1], topIpp[2]) + normal * (dz * fr);
                }
            } else if (topIpp.size() >= 3) {
                ref.position = glm::dvec3(topIpp[0], topIpp[1], topIpp[2]);
            }
            if (ref.slope == 0.0)
                ref.slope = 1.0;
            frames.push_back(ref);
        }
    }
    if (frames.empty())
        throw DicomError("No usable images in series");

    const bool allPositioned = std::all_of(frames.begin(), frames.end(), [](const FrameRef& r) { return r.position.has_value(); });
    if (!allPositioned)
        warn("ImagePositionPatient missing on some images; ordering by instance number");
    for (auto& r : frames)
        r.sortKey = allPositioned ? glm::dot(*r.position, normal) : r.instance;
    std::stable_sort(frames.begin(), frames.end(), [](const FrameRef& a, const FrameRef& b) { return a.sortKey < b.sortKey; });

    if (allPositioned) {
        std::vector<FrameRef> unique;
        for (const auto& r : frames) {
            if (!unique.empty() && std::abs(r.sortKey - unique.back().sortKey) < 1e-4) {
                continue;
            }
            unique.push_back(r);
        }
        if (unique.size() != frames.size())
            warn(std::format("Ignored {} images with duplicate positions", frames.size() - unique.size()));
        frames = std::move(unique);
    }

    // Geometry.
    VolumeGeometry geo;
    geo.dims = glm::ivec3(fmt.cols, fmt.rows, static_cast<int>(frames.size()));
    geo.spacing.x = pixelSpacing[1]; // column spacing: distance between adjacent columns (along the row direction)
    geo.spacing.y = pixelSpacing[0]; // row spacing
    glm::dvec3 sliceDir = normal;
    double dz = spacingBetween.value_or(sliceThickness.value_or(1.0));
    if (allPositioned && frames.size() >= 2) {
        const glm::dvec3 span = *frames.back().position - *frames.front().position;
        const double len = glm::length(span);
        if (len > 1e-6) {
            sliceDir = span / len;
            dz = len / static_cast<double>(frames.size() - 1);
        }
        // Spacing uniformity check.
        double maxDev = 0.0;
        for (std::size_t i = 1; i < frames.size(); ++i) {
            const double gap = frames[i].sortKey - frames[i - 1].sortKey;
            maxDev = std::max(maxDev, std::abs(gap - dz * glm::dot(sliceDir, normal)));
        }
        if (maxDev > 0.1 * dz)
            warn(std::format("Non-uniform slice spacing (max deviation {:.3f} mm); using mean spacing {:.3f} mm", maxDev, dz));
        if (std::abs(glm::dot(sliceDir, normal)) < 0.9999)
            log::info("DICOM: sheared stack (gantry tilt) detected; geometry is preserved exactly");
    }
    if (dz <= 0.0)
        dz = 1.0;
    geo.spacing.z = dz;
    geo.direction = glm::dmat3(rowDir, colDir, sliceDir);
    geo.origin = allPositioned ? *frames.front().position : glm::dvec3(0.0);

    // Value mapping: stored = (raw*slope_i + intercept_i - intercept) / slope with global slope/intercept.
    const double slope = frames.front().slope;
    double intercept = frames.front().intercept;
    const bool unsignedFull = fmt.pixelRepresentation == 0 && fmt.bitsStored >= 16 && fmt.bitsAllocated == 16;
    const double shift = unsignedFull ? 32768.0 : 0.0;
    intercept += shift * slope;
    const bool uniformRescale = std::all_of(frames.begin(), frames.end(), [&](const FrameRef& r) {
        return std::abs(r.slope - slope) < 1e-9 && std::abs(r.intercept - frames.front().intercept) < 1e-6;
    });
    if (fmt.bitsAllocated == 32)
        warn("32-bit pixel data converted to 16-bit (values may be clamped)");

    Volume vol;
    vol.geometry = geo;
    vol.rescaleSlope = slope;
    vol.rescaleIntercept = intercept;
    vol.voxels.assign(geo.voxelCount(), 0);
    vol.info.patientName = formatPersonName(first.getString(tags::PatientName, ""));
    vol.info.patientId = first.getString(tags::PatientID, "");
    vol.info.studyDate = first.getString(tags::StudyDate, "");
    vol.info.studyDescription = first.getString(tags::StudyDescription, "");
    vol.info.seriesDescription = first.getString(tags::SeriesDescription, "");
    vol.info.seriesInstanceUid = first.getString(tags::SeriesInstanceUID, "");
    vol.info.modality = first.getString(tags::Modality, "");
    vol.info.manufacturer = first.getString(tags::Manufacturer, "");
    vol.info.windowCenter = first.getNumber(tags::WindowCenter);
    vol.info.windowWidth = first.getNumber(tags::WindowWidth);

    // ---- Pass 2: pixel data -------------------------------------------------
    // Group frames by file so each file is read once.
    std::vector<std::vector<std::pair<int, std::size_t>>> perFile(headers.size()); // (frame, z)
    for (std::size_t z = 0; z < frames.size(); ++z)
        perFile[frames[z].file].emplace_back(frames[z].frame, z);
    headers.clear(); // release header buffers
    std::vector<std::size_t> fileOrder;
    for (std::size_t f = 0; f < perFile.size(); ++f)
        if (!perFile[f].empty())
            fileOrder.push_back(f);

    const std::size_t pixelsPerFrame = static_cast<std::size_t>(fmt.rows) * fmt.cols;
    const std::size_t bytesPerSample = static_cast<std::size_t>(fmt.bitsAllocated / 8);
    const std::size_t frameBytes = pixelsPerFrame * bytesPerSample;
    const std::uint32_t storedMask = fmt.bitsStored >= 32 ? 0xFFFFFFFFu : ((1u << fmt.bitsStored) - 1u);
    const int lowBit = std::max(0, fmt.highBit + 1 - fmt.bitsStored);
    std::atomic<std::size_t> clamped{0};

    done = 0;
    parallelFor(fileOrder.size(), [&](std::size_t idx) {
        if (cancelled)
            return;
        const std::size_t f = fileOrder[idx];
        ParsedFile pf = parseFile(series.files[f]);
        const Element* pixel = pf.data.find(tags::PixelData);
        if (!pixel)
            throw DicomError("Pixel data missing in " + platform::pathToUtf8(series.files[f]));
        const bool bigEndian = pf.transferSyntax == uids::ExplicitVRBigEndian;
        const int nFrames = static_cast<int>(std::max<long long>(1, pf.data.getInt(tags::NumberOfFrames).value_or(1)));

        std::vector<std::span<const std::uint8_t>> encFrames;
        std::vector<std::uint8_t> concatenated;
        if (pixel->encapsulated) {
            encFrames = splitFrames(*pixel, nFrames);
            if (encFrames.size() == 1 && encFrames[0].empty()) {
                for (std::size_t i = 1; i < pixel->fragments.size(); ++i)
                    concatenated.insert(concatenated.end(), pixel->fragments[i].begin(), pixel->fragments[i].end());
                encFrames[0] = concatenated;
            }
        }

        for (const auto& [frameIndex, z] : perFile[f]) {
            std::vector<std::uint8_t> decoded;
            const std::uint8_t* src = nullptr;
            if (pixel->encapsulated) {
                if (frameIndex >= static_cast<int>(encFrames.size()))
                    throw DicomError("Missing encapsulated frame");
                decoded = decodeFrame(pf.transferSyntax, encFrames[static_cast<std::size_t>(frameIndex)], fmt.rows, fmt.cols, fmt.bitsAllocated);
                if (decoded.size() < frameBytes)
                    throw DicomError("Decoded frame too small");
                src = decoded.data();
            } else {
                const std::size_t offset = static_cast<std::size_t>(frameIndex) * frameBytes;
                if (pixel->value.size() < offset + frameBytes)
                    throw DicomError("Pixel data shorter than expected in " + platform::pathToUtf8(series.files[f]));
                src = pixel->value.data() + offset;
            }
            const bool swap = !pixel->encapsulated && bigEndian;
            const FrameRef& ref = frames[z];
            const double frameSlope = ref.slope, frameIntercept = ref.intercept;
            std::int16_t* dst = vol.voxels.data() + z * pixelsPerFrame;
            std::size_t localClamped = 0;

            for (std::size_t p = 0; p < pixelsPerFrame; ++p) {
                std::uint32_t raw;
                const std::uint8_t* s = src + p * bytesPerSample;
                switch (bytesPerSample) {
                case 1: raw = s[0]; break;
                case 2: raw = swap ? static_cast<std::uint32_t>((s[0] << 8) | s[1]) : static_cast<std::uint32_t>(s[0] | (s[1] << 8)); break;
                default:
                    raw = swap ? (static_cast<std::uint32_t>(s[0]) << 24) | (s[1] << 16) | (s[2] << 8) | s[3]
                               : s[0] | (s[1] << 8) | (s[2] << 16) | (static_cast<std::uint32_t>(s[3]) << 24);
                    break;
                }
                raw = (raw >> lowBit) & storedMask;
                std::int64_t v;
                if (fmt.pixelRepresentation == 1 && fmt.bitsStored < 32 && (raw & (1u << (fmt.bitsStored - 1))))
                    v = static_cast<std::int64_t>(raw) - (static_cast<std::int64_t>(1) << fmt.bitsStored);
                else if (fmt.pixelRepresentation == 1 && fmt.bitsStored == 32)
                    v = static_cast<std::int32_t>(raw);
                else
                    v = raw;
                if (fmt.monochrome1)
                    v = static_cast<std::int64_t>(storedMask) - v;

                double stored;
                if (uniformRescale)
                    stored = static_cast<double>(v) - shift;
                else
                    stored = (static_cast<double>(v) * frameSlope + frameIntercept - intercept) / slope;
                long long q = std::llround(stored);
                if (q < -32768 || q > 32767) {
                    q = std::clamp<long long>(q, -32768, 32767);
                    ++localClamped;
                }
                dst[p] = static_cast<std::int16_t>(q);
            }
            clamped += localClamped;
        }
        tick(0.2f, 0.8f, fileOrder.size(), "Loading images");
    });
    if (cancelled)
        throw CancelledError();
    if (clamped > 0)
        warn(std::format("{} voxel values were outside the 16-bit range and clamped", clamped.load()));

    log::info("DICOM: loaded {}x{}x{} volume, spacing {:.3f}x{:.3f}x{:.3f} mm", geo.dims.x, geo.dims.y, geo.dims.z, geo.spacing.x,
              geo.spacing.y, geo.spacing.z);
    return vol;
}

Volume loadLargestSeries(const fs::path& folder, const ProgressFn& progress, LoadReport* report)
{
    ScanResult scan = scanForSeries(folder, [&](float f, const std::string& m) { return !progress || progress(f * 0.2f, m); });
    for (const auto& s : scan.series)
        if (s.supported)
            return loadSeries(s, [&](float f, const std::string& m) { return !progress || progress(0.2f + f * 0.8f, m); }, report);
    if (!scan.series.empty())
        throw DicomError("Found DICOM images but the transfer syntax is not supported: " + transferSyntaxName(scan.series.front().transferSyntax));
    throw DicomError("No DICOM image series found");
}

} // namespace occlusa::dicom
