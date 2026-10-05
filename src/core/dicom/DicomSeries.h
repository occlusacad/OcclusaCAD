#pragma once

#include "core/Progress.h"
#include "core/Volume.h"

#include <filesystem>
#include <string>
#include <vector>

namespace occlusa::dicom {

// One loadable image stack found while scanning a folder.
struct SeriesInfo {
    std::string seriesInstanceUid;
    std::string stackKey;   // series UID + orientation + matrix size (a series may contain several stacks)
    std::string seriesDescription;
    std::string patientName;
    std::string patientId;
    std::string studyDate;
    std::string modality;
    std::string transferSyntax;
    int rows = 0;
    int columns = 0;
    int sliceCount = 0;     // number of frames across all files
    bool multiFrame = false;
    bool supported = true;  // transfer syntax decodable
    std::vector<std::filesystem::path> files;

    std::string displayName() const;
};

struct ScanResult {
    std::vector<SeriesInfo> series;
    std::size_t filesScanned = 0;
    std::size_t filesSkipped = 0;
};

// Recursively scan a folder (or a single file) for DICOM image stacks.
ScanResult scanForSeries(const std::filesystem::path& folderOrFile, const ProgressFn& progress = {});

struct LoadReport {
    std::vector<std::string> warnings;
};

// Load an image stack into a volume in patient coordinates.
Volume loadSeries(const SeriesInfo& series, const ProgressFn& progress = {}, LoadReport* report = nullptr);

// Convenience: scan and load the largest supported stack in a folder.
Volume loadLargestSeries(const std::filesystem::path& folder, const ProgressFn& progress = {}, LoadReport* report = nullptr);

} // namespace occlusa::dicom
