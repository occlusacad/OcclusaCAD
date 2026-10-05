#pragma once

#include <cstdint>

namespace occlusa::dicom {

using Tag = std::uint32_t;

constexpr Tag makeTag(std::uint16_t group, std::uint16_t element)
{
    return (static_cast<Tag>(group) << 16) | element;
}
constexpr std::uint16_t tagGroup(Tag t) { return static_cast<std::uint16_t>(t >> 16); }
constexpr std::uint16_t tagElement(Tag t) { return static_cast<std::uint16_t>(t & 0xFFFF); }

namespace tags {
// File meta
constexpr Tag TransferSyntaxUID = makeTag(0x0002, 0x0010);
// Identification
constexpr Tag SpecificCharacterSet = makeTag(0x0008, 0x0005);
constexpr Tag SOPClassUID = makeTag(0x0008, 0x0016);
constexpr Tag SOPInstanceUID = makeTag(0x0008, 0x0018);
constexpr Tag StudyDate = makeTag(0x0008, 0x0020);
constexpr Tag Modality = makeTag(0x0008, 0x0060);
constexpr Tag Manufacturer = makeTag(0x0008, 0x0070);
constexpr Tag StudyDescription = makeTag(0x0008, 0x1030);
constexpr Tag SeriesDescription = makeTag(0x0008, 0x103E);
// Patient
constexpr Tag PatientName = makeTag(0x0010, 0x0010);
constexpr Tag PatientID = makeTag(0x0010, 0x0020);
constexpr Tag PatientBirthDate = makeTag(0x0010, 0x0030);
// Acquisition
constexpr Tag SliceThickness = makeTag(0x0018, 0x0050);
constexpr Tag SpacingBetweenSlices = makeTag(0x0018, 0x0088);
// Relationship / geometry
constexpr Tag StudyInstanceUID = makeTag(0x0020, 0x000D);
constexpr Tag SeriesInstanceUID = makeTag(0x0020, 0x000E);
constexpr Tag SeriesNumber = makeTag(0x0020, 0x0011);
constexpr Tag InstanceNumber = makeTag(0x0020, 0x0013);
constexpr Tag ImagePositionPatient = makeTag(0x0020, 0x0032);
constexpr Tag ImageOrientationPatient = makeTag(0x0020, 0x0037);
constexpr Tag FrameOfReferenceUID = makeTag(0x0020, 0x0052);
constexpr Tag SliceLocation = makeTag(0x0020, 0x1041);
constexpr Tag PlanePositionSequence = makeTag(0x0020, 0x9113);
constexpr Tag PlaneOrientationSequence = makeTag(0x0020, 0x9116);
// Image pixel
constexpr Tag SamplesPerPixel = makeTag(0x0028, 0x0002);
constexpr Tag PhotometricInterpretation = makeTag(0x0028, 0x0004);
constexpr Tag NumberOfFrames = makeTag(0x0028, 0x0008);
constexpr Tag Rows = makeTag(0x0028, 0x0010);
constexpr Tag Columns = makeTag(0x0028, 0x0011);
constexpr Tag PixelSpacing = makeTag(0x0028, 0x0030);
constexpr Tag BitsAllocated = makeTag(0x0028, 0x0100);
constexpr Tag BitsStored = makeTag(0x0028, 0x0101);
constexpr Tag HighBit = makeTag(0x0028, 0x0102);
constexpr Tag PixelRepresentation = makeTag(0x0028, 0x0103);
constexpr Tag WindowCenter = makeTag(0x0028, 0x1050);
constexpr Tag WindowWidth = makeTag(0x0028, 0x1051);
constexpr Tag RescaleIntercept = makeTag(0x0028, 0x1052);
constexpr Tag RescaleSlope = makeTag(0x0028, 0x1053);
constexpr Tag PixelMeasuresSequence = makeTag(0x0028, 0x9110);
constexpr Tag PixelValueTransformationSequence = makeTag(0x0028, 0x9145);
// Enhanced multi-frame functional groups
constexpr Tag SharedFunctionalGroupsSequence = makeTag(0x5200, 0x9229);
constexpr Tag PerFrameFunctionalGroupsSequence = makeTag(0x5200, 0x9230);
// Pixel data
constexpr Tag PixelData = makeTag(0x7FE0, 0x0010);
// Delimiters
constexpr Tag Item = makeTag(0xFFFE, 0xE000);
constexpr Tag ItemDelimitationItem = makeTag(0xFFFE, 0xE00D);
constexpr Tag SequenceDelimitationItem = makeTag(0xFFFE, 0xE0DD);
} // namespace tags

namespace uids {
constexpr const char* ImplicitVRLittleEndian = "1.2.840.10008.1.2";
constexpr const char* ExplicitVRLittleEndian = "1.2.840.10008.1.2.1";
constexpr const char* DeflatedExplicitVRLittleEndian = "1.2.840.10008.1.2.1.99";
constexpr const char* ExplicitVRBigEndian = "1.2.840.10008.1.2.2";
constexpr const char* JpegBaseline = "1.2.840.10008.1.2.4.50";
constexpr const char* JpegLossless = "1.2.840.10008.1.2.4.57";
constexpr const char* JpegLosslessSV1 = "1.2.840.10008.1.2.4.70";
constexpr const char* JpegLsLossless = "1.2.840.10008.1.2.4.80";
constexpr const char* JpegLsNearLossless = "1.2.840.10008.1.2.4.81";
constexpr const char* Jpeg2000Lossless = "1.2.840.10008.1.2.4.90";
constexpr const char* Jpeg2000 = "1.2.840.10008.1.2.4.91";
constexpr const char* RleLossless = "1.2.840.10008.1.2.5";

constexpr const char* CTImageStorage = "1.2.840.10008.5.1.4.1.1.2";
constexpr const char* EnhancedCTImageStorage = "1.2.840.10008.5.1.4.1.1.2.1";
} // namespace uids

} // namespace occlusa::dicom
