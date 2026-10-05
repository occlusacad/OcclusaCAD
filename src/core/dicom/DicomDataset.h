#pragma once

#include "core/dicom/DicomTags.h"

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace occlusa::dicom {

class DicomError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class Dataset;

struct Element {
    Tag tag = 0;
    char vr[2] = {'U', 'N'};
    std::span<const std::uint8_t> value; // raw bytes (views into the owning ParsedFile buffer)
    std::vector<Dataset> items;          // for sequences
    std::vector<std::span<const std::uint8_t>> fragments; // encapsulated pixel data (first = basic offset table)
    bool encapsulated = false;
    bool isSequence = false;

    bool hasVR(const char* two) const { return vr[0] == two[0] && vr[1] == two[1]; }
};

// A parsed DICOM data set. Values are views into a buffer owned by ParsedFile;
// a Dataset must not outlive the ParsedFile it came from.
class Dataset {
public:
    bool bigEndian = false;
    bool latin1 = false; // SpecificCharacterSet ISO_IR 100 (converted to UTF-8 on access)
    std::map<Tag, Element> elements;

    bool has(Tag t) const { return elements.count(t) != 0; }
    const Element* find(Tag t) const;

    // Text values (trailing padding removed, converted to UTF-8 when the character set is known).
    std::optional<std::string> getString(Tag t) const;
    std::string getString(Tag t, const std::string& fallback) const { return getString(t).value_or(fallback); }
    std::vector<std::string> getStrings(Tag t) const; // backslash separated multi-values

    // Numeric values: works for IS/DS text as well as binary US/SS/UL/SL/FL/FD.
    std::vector<double> getNumbers(Tag t) const;
    std::optional<double> getNumber(Tag t, std::size_t index = 0) const;
    std::optional<long long> getInt(Tag t, std::size_t index = 0) const;

    const Dataset* item(Tag sequence, std::size_t index = 0) const;
    std::size_t itemCount(Tag sequence) const;
};

struct ParseOptions {
    bool stopBeforePixelData = false;
    // Read only this many bytes from disk (0 = whole file). Parsing stops gracefully at the end.
    std::size_t maxBytes = 0;
};

struct ParsedFile {
    std::shared_ptr<std::vector<std::uint8_t>> buffer;
    Dataset meta;
    Dataset data;
    std::string transferSyntax;
    bool truncated = false; // parse hit end of available bytes

    bool isEncapsulated() const;
};

// Parse a DICOM Part 10 file (or a raw implicit VR little endian dataset without preamble).
ParsedFile parseFile(const std::filesystem::path& path, const ParseOptions& options = {});
ParsedFile parseBuffer(std::shared_ptr<std::vector<std::uint8_t>> buffer, const ParseOptions& options = {});

// Heuristic check (preamble + "DICM" magic) without parsing.
bool looksLikeDicom(const std::filesystem::path& path);

std::string latin1ToUtf8(std::string_view in);

} // namespace occlusa::dicom
