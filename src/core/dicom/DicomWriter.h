#pragma once

#include "core/Progress.h"
#include "core/Volume.h"

#include <filesystem>
#include <string>

namespace occlusa::dicom {

struct WriteOptions {
    bool explicitVR = true;      // false: implicit VR little endian
    bool rleCompress = false;    // RLE Lossless (exercises the decoder in tests)
    std::string patientName = "PHANTOM^SYNTHETIC";
    std::string patientId = "OCCLUSA-PHANTOM";
    std::string seriesDescription = "OcclusaCAD synthetic CBCT";
    std::string modality = "CT";
};

// Write a volume as a single-frame CT Image Storage series (one file per slice).
// Intended for synthetic test data; writes the minimum attribute set OcclusaCAD and
// common viewers need. Not a conformant DICOM exporter.
void writeSeries(const Volume& volume, const std::filesystem::path& directory, const WriteOptions& options = {},
                 const ProgressFn& progress = {});

} // namespace occlusa::dicom
