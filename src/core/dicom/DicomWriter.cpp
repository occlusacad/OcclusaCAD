#include "core/dicom/DicomWriter.h"

#include "core/Uuid.h"
#include "core/dicom/DicomDataset.h"
#include "core/dicom/DicomTags.h"

#include <cstring>
#include <format>
#include <fstream>
#include <vector>

namespace occlusa::dicom {
namespace {

class Encoder {
public:
    explicit Encoder(bool explicitVR) : explicit_(explicitVR) {}

    std::vector<std::uint8_t> bytes;

    void tag(Tag t)
    {
        u16(tagGroup(t));
        u16(tagElement(t));
    }
    void u16(std::uint16_t v)
    {
        bytes.push_back(static_cast<std::uint8_t>(v));
        bytes.push_back(static_cast<std::uint8_t>(v >> 8));
    }
    void u32(std::uint32_t v)
    {
        for (int i = 0; i < 4; ++i)
            bytes.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    }

    void element(Tag t, const char* vr, std::span<const std::uint8_t> value)
    {
        const bool longLen = std::strcmp(vr, "OB") == 0 || std::strcmp(vr, "OW") == 0 || std::strcmp(vr, "UN") == 0 ||
                             std::strcmp(vr, "SQ") == 0 || std::strcmp(vr, "UT") == 0;
        tag(t);
        if (explicit_ || tagGroup(t) == 0x0002) {
            bytes.push_back(static_cast<std::uint8_t>(vr[0]));
            bytes.push_back(static_cast<std::uint8_t>(vr[1]));
            if (longLen) {
                u16(0);
                u32(static_cast<std::uint32_t>(value.size()));
            } else {
                u16(static_cast<std::uint16_t>(value.size()));
            }
        } else {
            u32(static_cast<std::uint32_t>(value.size()));
        }
        bytes.insert(bytes.end(), value.begin(), value.end());
    }

    void text(Tag t, const char* vr, std::string s)
    {
        if (s.size() % 2)
            s.push_back(std::strcmp(vr, "UI") == 0 ? '\0' : ' ');
        element(t, vr, std::span<const std::uint8_t>(reinterpret_cast<const std::uint8_t*>(s.data()), s.size()));
    }
    void us(Tag t, std::uint16_t v)
    {
        const std::uint8_t b[2] = {static_cast<std::uint8_t>(v), static_cast<std::uint8_t>(v >> 8)};
        element(t, "US", b);
    }

private:
    bool explicit_;
};

std::string ds(double v)
{
    std::string s = std::format("{:.6g}", v);
    return s;
}

std::vector<std::uint8_t> rleEncodeSegment(const std::vector<std::uint8_t>& in)
{
    // Simple PackBits encoder: replicate runs >= 3, literals otherwise.
    std::vector<std::uint8_t> out;
    std::size_t i = 0;
    while (i < in.size()) {
        std::size_t run = 1;
        while (i + run < in.size() && run < 128 && in[i + run] == in[i])
            ++run;
        if (run >= 3) {
            out.push_back(static_cast<std::uint8_t>(static_cast<std::int8_t>(1 - static_cast<int>(run))));
            out.push_back(in[i]);
            i += run;
            continue;
        }
        std::size_t litStart = i, lit = 0;
        while (i < in.size() && lit < 128) {
            std::size_t r = 1;
            while (i + r < in.size() && r < 3 && in[i + r] == in[i])
                ++r;
            if (r >= 3)
                break;
            ++i;
            ++lit;
        }
        out.push_back(static_cast<std::uint8_t>(lit - 1));
        out.insert(out.end(), in.begin() + static_cast<std::ptrdiff_t>(litStart), in.begin() + static_cast<std::ptrdiff_t>(litStart + lit));
    }
    if (out.size() % 2)
        out.push_back(0x80); // no-op padding
    return out;
}

} // namespace

void writeSeries(const Volume& vol, const std::filesystem::path& dir, const WriteOptions& opt, const ProgressFn& progress)
{
    std::filesystem::create_directories(dir);
    const std::string studyUid = generateDicomUid();
    const std::string seriesUid = generateDicomUid();
    const std::string forUid = generateDicomUid();
    const auto& g = vol.geometry;
    const glm::dvec3 rowDir = g.direction[0], colDir = g.direction[1];
    const std::string ts = opt.rleCompress ? uids::RleLossless : (opt.explicitVR ? uids::ExplicitVRLittleEndian : uids::ImplicitVRLittleEndian);
    const bool explicitData = opt.rleCompress || opt.explicitVR;
    const std::size_t sliceSize = static_cast<std::size_t>(g.dims.x) * g.dims.y;

    for (int z = 0; z < g.dims.z; ++z) {
        reportProgress(progress, static_cast<float>(z) / static_cast<float>(g.dims.z), std::format("Writing slice {}", z + 1));
        const std::string sopUid = generateDicomUid();
        const glm::dvec3 ipp = transformPoint(g.voxelToWorld(), glm::dvec3(0, 0, z));

        Encoder meta(true);
        {
            const std::uint8_t version[2] = {0, 1};
            meta.element(makeTag(0x0002, 0x0001), "OB", version);
            meta.text(makeTag(0x0002, 0x0002), "UI", uids::CTImageStorage);
            meta.text(makeTag(0x0002, 0x0003), "UI", sopUid);
            meta.text(tags::TransferSyntaxUID, "UI", ts);
            meta.text(makeTag(0x0002, 0x0012), "UI", "2.25.1");
            meta.text(makeTag(0x0002, 0x0013), "SH", "OCCLUSACAD_0_1");
        }
        Encoder ds_(explicitData);
        ds_.text(tags::SpecificCharacterSet, "CS", "ISO_IR 192");
        ds_.text(tags::SOPClassUID, "UI", uids::CTImageStorage);
        ds_.text(tags::SOPInstanceUID, "UI", sopUid);
        ds_.text(tags::StudyDate, "DA", "20260101");
        ds_.text(tags::Modality, "CS", opt.modality);
        ds_.text(tags::Manufacturer, "LO", "OcclusaCAD");
        ds_.text(tags::StudyDescription, "LO", "Synthetic study");
        ds_.text(tags::SeriesDescription, "LO", opt.seriesDescription);
        ds_.text(tags::PatientName, "PN", opt.patientName);
        ds_.text(tags::PatientID, "LO", opt.patientId);
        ds_.text(tags::SliceThickness, "DS", ds(g.spacing.z));
        ds_.text(tags::StudyInstanceUID, "UI", studyUid);
        ds_.text(tags::SeriesInstanceUID, "UI", seriesUid);
        ds_.text(tags::SeriesNumber, "IS", "1");
        ds_.text(tags::InstanceNumber, "IS", std::to_string(z + 1));
        ds_.text(tags::ImagePositionPatient, "DS", ds(ipp.x) + "\\" + ds(ipp.y) + "\\" + ds(ipp.z));
        ds_.text(tags::ImageOrientationPatient, "DS",
                 ds(rowDir.x) + "\\" + ds(rowDir.y) + "\\" + ds(rowDir.z) + "\\" + ds(colDir.x) + "\\" + ds(colDir.y) + "\\" + ds(colDir.z));
        ds_.text(tags::FrameOfReferenceUID, "UI", forUid);
        ds_.us(tags::SamplesPerPixel, 1);
        ds_.text(tags::PhotometricInterpretation, "CS", "MONOCHROME2");
        ds_.us(tags::Rows, static_cast<std::uint16_t>(g.dims.y));
        ds_.us(tags::Columns, static_cast<std::uint16_t>(g.dims.x));
        ds_.text(tags::PixelSpacing, "DS", ds(g.spacing.y) + "\\" + ds(g.spacing.x));
        ds_.us(tags::BitsAllocated, 16);
        ds_.us(tags::BitsStored, 16);
        ds_.us(tags::HighBit, 15);
        ds_.us(tags::PixelRepresentation, 1);
        if (vol.info.windowCenter && vol.info.windowWidth) {
            ds_.text(tags::WindowCenter, "DS", ds(*vol.info.windowCenter));
            ds_.text(tags::WindowWidth, "DS", ds(*vol.info.windowWidth));
        }
        ds_.text(tags::RescaleIntercept, "DS", ds(vol.rescaleIntercept));
        ds_.text(tags::RescaleSlope, "DS", ds(vol.rescaleSlope));

        const std::int16_t* src = vol.voxels.data() + static_cast<std::size_t>(z) * sliceSize;
        if (opt.rleCompress) {
            std::vector<std::uint8_t> hi(sliceSize), lo(sliceSize);
            for (std::size_t i = 0; i < sliceSize; ++i) {
                const auto u = static_cast<std::uint16_t>(src[i]);
                hi[i] = static_cast<std::uint8_t>(u >> 8);
                lo[i] = static_cast<std::uint8_t>(u & 0xFF);
            }
            const auto segHi = rleEncodeSegment(hi), segLo = rleEncodeSegment(lo);
            std::vector<std::uint8_t> frame(64, 0);
            auto put32 = [&](std::size_t off, std::uint32_t v) {
                for (int i = 0; i < 4; ++i)
                    frame[off + i] = static_cast<std::uint8_t>(v >> (8 * i));
            };
            put32(0, 2);
            put32(4, 64);
            put32(8, static_cast<std::uint32_t>(64 + segHi.size()));
            frame.insert(frame.end(), segHi.begin(), segHi.end());
            frame.insert(frame.end(), segLo.begin(), segLo.end());
            // Encapsulated: undefined length, empty basic offset table, one fragment, delimiter.
            ds_.tag(tags::PixelData);
            ds_.bytes.push_back('O');
            ds_.bytes.push_back('B');
            ds_.u16(0);
            ds_.u32(0xFFFFFFFFu);
            ds_.tag(tags::Item);
            ds_.u32(0);
            ds_.tag(tags::Item);
            ds_.u32(static_cast<std::uint32_t>(frame.size()));
            ds_.bytes.insert(ds_.bytes.end(), frame.begin(), frame.end());
            ds_.tag(tags::SequenceDelimitationItem);
            ds_.u32(0);
        } else {
            std::vector<std::uint8_t> pix(sliceSize * 2);
            for (std::size_t i = 0; i < sliceSize; ++i) {
                const auto u = static_cast<std::uint16_t>(src[i]);
                pix[2 * i] = static_cast<std::uint8_t>(u & 0xFF);
                pix[2 * i + 1] = static_cast<std::uint8_t>(u >> 8);
            }
            ds_.element(tags::PixelData, "OW", pix);
        }

        Encoder groupLen(true);
        {
            const std::uint32_t len = static_cast<std::uint32_t>(meta.bytes.size());
            const std::uint8_t b[4] = {static_cast<std::uint8_t>(len), static_cast<std::uint8_t>(len >> 8), static_cast<std::uint8_t>(len >> 16),
                                       static_cast<std::uint8_t>(len >> 24)};
            groupLen.element(makeTag(0x0002, 0x0000), "UL", b);
        }

        const auto path = dir / std::format("IM{:05d}.dcm", z + 1);
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out)
            throw DicomError("Cannot write " + path.string());
        const char preamble[128] = {};
        out.write(preamble, 128);
        out.write("DICM", 4);
        out.write(reinterpret_cast<const char*>(groupLen.bytes.data()), static_cast<std::streamsize>(groupLen.bytes.size()));
        out.write(reinterpret_cast<const char*>(meta.bytes.data()), static_cast<std::streamsize>(meta.bytes.size()));
        out.write(reinterpret_cast<const char*>(ds_.bytes.data()), static_cast<std::streamsize>(ds_.bytes.size()));
        if (!out)
            throw DicomError("Failed writing " + path.string());
    }
}

} // namespace occlusa::dicom
