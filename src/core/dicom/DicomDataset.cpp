#include "core/dicom/DicomDataset.h"

#include <bit>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>

namespace occlusa::dicom {
namespace {

constexpr std::uint32_t kUndefinedLength = 0xFFFFFFFFu;

// VRs for tags we may need to interpret in implicit VR files. Anything not listed
// is treated as UN, which is fine because accessors decode text vs binary by VR.
struct DictEntry {
    Tag tag;
    const char* vr;
};
constexpr DictEntry kDictionary[] = {
    {tags::TransferSyntaxUID, "UI"},
    {tags::SpecificCharacterSet, "CS"},
    {tags::SOPClassUID, "UI"},
    {tags::SOPInstanceUID, "UI"},
    {tags::StudyDate, "DA"},
    {tags::Modality, "CS"},
    {tags::Manufacturer, "LO"},
    {tags::StudyDescription, "LO"},
    {tags::SeriesDescription, "LO"},
    {tags::PatientName, "PN"},
    {tags::PatientID, "LO"},
    {tags::PatientBirthDate, "DA"},
    {tags::SliceThickness, "DS"},
    {tags::SpacingBetweenSlices, "DS"},
    {tags::StudyInstanceUID, "UI"},
    {tags::SeriesInstanceUID, "UI"},
    {tags::SeriesNumber, "IS"},
    {tags::InstanceNumber, "IS"},
    {tags::ImagePositionPatient, "DS"},
    {tags::ImageOrientationPatient, "DS"},
    {tags::FrameOfReferenceUID, "UI"},
    {tags::SliceLocation, "DS"},
    {tags::PlanePositionSequence, "SQ"},
    {tags::PlaneOrientationSequence, "SQ"},
    {tags::SamplesPerPixel, "US"},
    {tags::PhotometricInterpretation, "CS"},
    {tags::NumberOfFrames, "IS"},
    {tags::Rows, "US"},
    {tags::Columns, "US"},
    {tags::PixelSpacing, "DS"},
    {tags::BitsAllocated, "US"},
    {tags::BitsStored, "US"},
    {tags::HighBit, "US"},
    {tags::PixelRepresentation, "US"},
    {tags::WindowCenter, "DS"},
    {tags::WindowWidth, "DS"},
    {tags::RescaleIntercept, "DS"},
    {tags::RescaleSlope, "DS"},
    {tags::PixelMeasuresSequence, "SQ"},
    {tags::PixelValueTransformationSequence, "SQ"},
    {tags::SharedFunctionalGroupsSequence, "SQ"},
    {tags::PerFrameFunctionalGroupsSequence, "SQ"},
    {tags::PixelData, "OW"},
};

const char* dictionaryVR(Tag t)
{
    for (const auto& e : kDictionary)
        if (e.tag == t)
            return e.vr;
    if (tagElement(t) == 0x0000)
        return "UL"; // group length
    return "UN";
}

bool isLongLengthVR(const char vr[2])
{
    static constexpr const char* kLong[] = {"OB", "OD", "OF", "OL", "OV", "OW", "SQ", "SV", "UC", "UN", "UR", "UT", "UV"};
    for (const char* l : kLong)
        if (vr[0] == l[0] && vr[1] == l[1])
            return true;
    return false;
}

bool isValidVRChars(const std::uint8_t* p)
{
    return p[0] >= 'A' && p[0] <= 'Z' && p[1] >= 'A' && p[1] <= 'Z';
}

class Reader {
public:
    Reader(std::span<const std::uint8_t> data, std::size_t pos) : data_(data), pos_(pos) {}

    std::size_t pos() const { return pos_; }
    void seek(std::size_t p) { pos_ = p; }
    std::size_t remaining() const { return pos_ < data_.size() ? data_.size() - pos_ : 0; }
    std::size_t size() const { return data_.size(); }
    std::span<const std::uint8_t> data() const { return data_; }

    std::uint16_t u16(bool be)
    {
        const std::uint8_t* p = data_.data() + pos_;
        pos_ += 2;
        return be ? static_cast<std::uint16_t>((p[0] << 8) | p[1]) : static_cast<std::uint16_t>(p[0] | (p[1] << 8));
    }
    std::uint32_t u32(bool be)
    {
        const std::uint8_t* p = data_.data() + pos_;
        pos_ += 4;
        if (be)
            return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16) | (static_cast<std::uint32_t>(p[2]) << 8) | p[3];
        return p[0] | (static_cast<std::uint32_t>(p[1]) << 8) | (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
    }
    const std::uint8_t* ptr() const { return data_.data() + pos_; }

private:
    std::span<const std::uint8_t> data_;
    std::size_t pos_;
};

struct Syntax {
    bool explicitVR = true;
    bool bigEndian = false;
};

struct Truncated {};

class Parser {
public:
    Parser(Reader& r, const ParseOptions& opts) : r_(r), opts_(opts) {}

    bool hitPixelData = false;

    // Parse elements until `end` (exclusive) or an item delimiter when `end` is undefined.
    void parseDataset(Dataset& ds, Syntax syn, std::size_t end, bool undefinedLength, int depth)
    {
        if (depth > 32)
            throw DicomError("DICOM sequence nesting too deep");
        ds.bigEndian = syn.bigEndian;
        while (r_.pos() < end) {
            if (r_.remaining() < 8)
                throw Truncated{};
            const std::size_t elemStart = r_.pos();
            const std::uint16_t group = r_.u16(syn.bigEndian);
            const std::uint16_t element = r_.u16(syn.bigEndian);
            const Tag tag = makeTag(group, element);

            if (tag == tags::ItemDelimitationItem) {
                r_.u32(syn.bigEndian);
                if (undefinedLength)
                    return;
                continue; // stray delimiter, ignore
            }
            if (tag == tags::SequenceDelimitationItem) {
                r_.u32(syn.bigEndian);
                continue;
            }

            Element el;
            el.tag = tag;
            std::uint32_t length = 0;
            if (group == 0xFFFE) {
                // Items have no VR, even in explicit syntaxes.
                length = r_.u32(syn.bigEndian);
            } else if (syn.explicitVR && isValidVRChars(r_.ptr())) {
                el.vr[0] = static_cast<char>(r_.ptr()[0]);
                el.vr[1] = static_cast<char>(r_.ptr()[1]);
                r_.seek(r_.pos() + 2);
                if (isLongLengthVR(el.vr)) {
                    if (r_.remaining() < 6)
                        throw Truncated{};
                    r_.seek(r_.pos() + 2);
                    length = r_.u32(syn.bigEndian);
                } else {
                    length = r_.u16(syn.bigEndian);
                }
            } else {
                const char* vr = dictionaryVR(tag);
                el.vr[0] = vr[0];
                el.vr[1] = vr[1];
                length = r_.u32(syn.bigEndian);
            }

            if (tag == tags::PixelData && opts_.stopBeforePixelData) {
                hitPixelData = true;
                ds.elements.emplace(tag, std::move(el));
                r_.seek(r_.size());
                return;
            }

            if (tag == tags::PixelData && length == kUndefinedLength) {
                el.encapsulated = true;
                parseFragments(el, syn);
                ds.elements.emplace(tag, std::move(el));
                continue;
            }

            const bool sequence = el.hasVR("SQ") || (length == kUndefinedLength) ||
                                  (!syn.explicitVR && length >= 8 && r_.remaining() >= 4 && peekIsItem(syn));
            if (sequence && tag != tags::PixelData) {
                // Undefined length UN sequences are encoded as implicit VR little endian (PS3.5 6.2.2).
                Syntax inner = syn;
                if (syn.explicitVR && el.hasVR("UN"))
                    inner = Syntax{false, false};
                el.isSequence = true;
                el.vr[0] = 'S';
                el.vr[1] = 'Q';
                parseSequence(el, inner, length, depth);
                ds.elements.emplace(tag, std::move(el));
                continue;
            }

            if (length == kUndefinedLength)
                throw DicomError("Unexpected undefined length element");
            if (length > r_.remaining()) {
                (void)elemStart;
                throw Truncated{};
            }
            el.value = std::span<const std::uint8_t>(r_.ptr(), length);
            r_.seek(r_.pos() + length);

            if (tag == tags::SpecificCharacterSet) {
                std::string cs(reinterpret_cast<const char*>(el.value.data()), el.value.size());
                ds.latin1 = cs.find("ISO_IR 100") != std::string::npos;
                // TODO: other character sets (ISO_IR 101/109/110/144/127/..., ISO 2022 escapes).
            }
            ds.elements.emplace(tag, std::move(el));
        }
    }

private:
    bool peekIsItem(Syntax syn) const
    {
        const std::uint8_t* p = r_.ptr();
        const std::uint16_t g = syn.bigEndian ? static_cast<std::uint16_t>((p[0] << 8) | p[1]) : static_cast<std::uint16_t>(p[0] | (p[1] << 8));
        const std::uint16_t e = syn.bigEndian ? static_cast<std::uint16_t>((p[2] << 8) | p[3]) : static_cast<std::uint16_t>(p[2] | (p[3] << 8));
        return g == 0xFFFE && e == 0xE000;
    }

    void parseSequence(Element& el, Syntax syn, std::uint32_t length, int depth)
    {
        const bool undefinedSeq = length == kUndefinedLength;
        if (!undefinedSeq && length > r_.remaining())
            throw Truncated{};
        const std::size_t seqEnd = undefinedSeq ? r_.size() : r_.pos() + length;
        while (r_.pos() < seqEnd) {
            if (r_.remaining() < 8)
                throw Truncated{};
            const std::uint16_t g = r_.u16(syn.bigEndian);
            const std::uint16_t e = r_.u16(syn.bigEndian);
            const std::uint32_t itemLen = r_.u32(syn.bigEndian);
            const Tag t = makeTag(g, e);
            if (t == tags::SequenceDelimitationItem)
                return;
            if (t != tags::Item)
                throw DicomError("Malformed sequence: expected item tag");
            Dataset item;
            if (itemLen == kUndefinedLength) {
                parseDataset(item, syn, r_.size(), true, depth + 1);
            } else {
                if (itemLen > r_.remaining())
                    throw Truncated{};
                parseDataset(item, syn, r_.pos() + itemLen, false, depth + 1);
            }
            el.items.push_back(std::move(item));
        }
    }

    void parseFragments(Element& el, Syntax syn)
    {
        for (;;) {
            if (r_.remaining() < 8)
                throw Truncated{};
            const std::uint16_t g = r_.u16(syn.bigEndian);
            const std::uint16_t e = r_.u16(syn.bigEndian);
            const std::uint32_t len = r_.u32(syn.bigEndian);
            const Tag t = makeTag(g, e);
            if (t == tags::SequenceDelimitationItem)
                return;
            if (t != tags::Item)
                throw DicomError("Malformed encapsulated pixel data");
            if (len > r_.remaining())
                throw Truncated{};
            el.fragments.emplace_back(r_.ptr(), len);
            r_.seek(r_.pos() + len);
        }
    }

    Reader& r_;
    const ParseOptions& opts_;
};

std::string trimValue(std::string s)
{
    while (!s.empty() && (s.back() == ' ' || s.back() == '\0'))
        s.pop_back();
    std::size_t start = 0;
    while (start < s.size() && s[start] == ' ')
        ++start;
    return s.substr(start);
}

bool isTextVR(const Element& el)
{
    static constexpr const char* kText[] = {"AE", "AS", "CS", "DA", "DS", "DT", "IS", "LO", "LT", "PN", "SH", "ST", "TM", "UC", "UI", "UR", "UT"};
    for (const char* t : kText)
        if (el.hasVR(t))
            return true;
    return false;
}

} // namespace

std::string latin1ToUtf8(std::string_view in)
{
    std::string out;
    out.reserve(in.size());
    for (unsigned char c : in) {
        if (c < 0x80) {
            out.push_back(static_cast<char>(c));
        } else {
            out.push_back(static_cast<char>(0xC0 | (c >> 6)));
            out.push_back(static_cast<char>(0x80 | (c & 0x3F)));
        }
    }
    return out;
}

const Element* Dataset::find(Tag t) const
{
    auto it = elements.find(t);
    return it == elements.end() ? nullptr : &it->second;
}

std::optional<std::string> Dataset::getString(Tag t) const
{
    const Element* el = find(t);
    if (!el || el->isSequence)
        return std::nullopt;
    std::string s(reinterpret_cast<const char*>(el->value.data()), el->value.size());
    s = trimValue(std::move(s));
    if (latin1)
        s = latin1ToUtf8(s);
    return s;
}

std::vector<std::string> Dataset::getStrings(Tag t) const
{
    std::vector<std::string> out;
    auto s = getString(t);
    if (!s)
        return out;
    std::size_t start = 0;
    for (;;) {
        const std::size_t p = s->find('\\', start);
        out.push_back(trimValue(s->substr(start, p == std::string::npos ? std::string::npos : p - start)));
        if (p == std::string::npos)
            break;
        start = p + 1;
    }
    return out;
}

std::vector<double> Dataset::getNumbers(Tag t) const
{
    std::vector<double> out;
    const Element* el = find(t);
    if (!el || el->isSequence)
        return out;

    auto readBinary = [&](std::size_t size, auto conv) {
        for (std::size_t off = 0; off + size <= el->value.size(); off += size) {
            std::uint8_t b[8];
            std::memcpy(b, el->value.data() + off, size);
            if (bigEndian != (std::endian::native == std::endian::big)) {
                for (std::size_t i = 0; i < size / 2; ++i)
                    std::swap(b[i], b[size - 1 - i]);
            }
            out.push_back(conv(b));
        }
    };

    if (el->hasVR("US"))
        readBinary(2, [](const std::uint8_t* b) { std::uint16_t v; std::memcpy(&v, b, 2); return static_cast<double>(v); });
    else if (el->hasVR("SS"))
        readBinary(2, [](const std::uint8_t* b) { std::int16_t v; std::memcpy(&v, b, 2); return static_cast<double>(v); });
    else if (el->hasVR("UL"))
        readBinary(4, [](const std::uint8_t* b) { std::uint32_t v; std::memcpy(&v, b, 4); return static_cast<double>(v); });
    else if (el->hasVR("SL"))
        readBinary(4, [](const std::uint8_t* b) { std::int32_t v; std::memcpy(&v, b, 4); return static_cast<double>(v); });
    else if (el->hasVR("FL"))
        readBinary(4, [](const std::uint8_t* b) { float v; std::memcpy(&v, b, 4); return static_cast<double>(v); });
    else if (el->hasVR("FD"))
        readBinary(8, [](const std::uint8_t* b) { double v; std::memcpy(&v, b, 8); return v; });
    else if (isTextVR(*el) || el->hasVR("UN")) {
        for (const auto& part : getStrings(t)) {
            if (part.empty())
                continue;
            char* end = nullptr;
            const double v = std::strtod(part.c_str(), &end);
            if (end != part.c_str() && std::isfinite(v))
                out.push_back(v);
        }
    }
    return out;
}

std::optional<double> Dataset::getNumber(Tag t, std::size_t index) const
{
    auto v = getNumbers(t);
    if (index >= v.size())
        return std::nullopt;
    return v[index];
}

std::optional<long long> Dataset::getInt(Tag t, std::size_t index) const
{
    auto v = getNumber(t, index);
    if (!v)
        return std::nullopt;
    return std::llround(*v);
}

const Dataset* Dataset::item(Tag sequence, std::size_t index) const
{
    const Element* el = find(sequence);
    if (!el || index >= el->items.size())
        return nullptr;
    return &el->items[index];
}

std::size_t Dataset::itemCount(Tag sequence) const
{
    const Element* el = find(sequence);
    return el ? el->items.size() : 0;
}

bool ParsedFile::isEncapsulated() const
{
    const Element* el = data.find(tags::PixelData);
    return el && el->encapsulated;
}

ParsedFile parseBuffer(std::shared_ptr<std::vector<std::uint8_t>> buffer, const ParseOptions& options)
{
    ParsedFile out;
    out.buffer = std::move(buffer);
    const std::span<const std::uint8_t> bytes(*out.buffer);

    std::size_t start = 0;
    const bool hasPreamble = bytes.size() >= 132 && std::memcmp(bytes.data() + 128, "DICM", 4) == 0;
    Syntax dataSyntax{false, false};

    try {
        if (hasPreamble) {
            start = 132;
            // File meta information: always explicit VR little endian. Its extent is given by
            // the group length, but parse group 0002 elements defensively.
            Reader r(bytes, start);
            Parser p(r, options);
            std::size_t metaEnd = bytes.size();
            {
                // Peek at (0002,0000) group length.
                if (bytes.size() >= start + 12 && bytes[start] == 0x02 && bytes[start + 1] == 0x00 && bytes[start + 2] == 0x00 &&
                    bytes[start + 3] == 0x00) {
                    Reader g(bytes, start + 8);
                    metaEnd = start + 12 + g.u32(false);
                } else {
                    // No group length: scan forward while group == 0002.
                    std::size_t pos = start;
                    while (pos + 8 <= bytes.size() && bytes[pos] == 0x02 && bytes[pos + 1] == 0x00) {
                        Reader g(bytes, pos + 4);
                        char vr[2] = {static_cast<char>(bytes[pos + 4]), static_cast<char>(bytes[pos + 5])};
                        std::uint32_t len;
                        std::size_t header;
                        if (isLongLengthVR(vr)) {
                            Reader l(bytes, pos + 8);
                            len = l.u32(false);
                            header = 12;
                        } else {
                            Reader l(bytes, pos + 6);
                            len = l.u16(false);
                            header = 8;
                        }
                        pos += header + len;
                    }
                    metaEnd = pos;
                }
            }
            metaEnd = std::min(metaEnd, bytes.size());
            p.parseDataset(out.meta, Syntax{true, false}, metaEnd, false, 0);
            out.transferSyntax = out.meta.getString(tags::TransferSyntaxUID).value_or(uids::ExplicitVRLittleEndian);
            start = metaEnd;
        } else {
            // Raw dataset: guess the syntax from the first element.
            if (bytes.size() >= 8 && isValidVRChars(bytes.data() + 4))
                out.transferSyntax = uids::ExplicitVRLittleEndian;
            else
                out.transferSyntax = uids::ImplicitVRLittleEndian;
        }

        const std::string& ts = out.transferSyntax;
        if (ts == uids::ImplicitVRLittleEndian)
            dataSyntax = {false, false};
        else if (ts == uids::ExplicitVRBigEndian)
            dataSyntax = {true, true};
        else if (ts == uids::DeflatedExplicitVRLittleEndian)
            throw DicomError("Deflated transfer syntax is not supported yet"); // TODO: inflate with a zlib-licensed decoder
        else
            dataSyntax = {true, false}; // explicit LE, and all encapsulated syntaxes

        Reader r(bytes, start);
        Parser p(r, options);
        p.parseDataset(out.data, dataSyntax, bytes.size(), false, 0);
    } catch (const Truncated&) {
        out.truncated = true;
    }
    return out;
}

ParsedFile parseFile(const std::filesystem::path& path, const ParseOptions& options)
{
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in)
        throw DicomError("Cannot open " + path.string());
    std::size_t size = static_cast<std::size_t>(in.tellg());
    const bool partial = options.maxBytes > 0 && options.maxBytes < size;
    if (partial)
        size = options.maxBytes;
    in.seekg(0);
    auto buffer = std::make_shared<std::vector<std::uint8_t>>(size);
    if (size > 0 && !in.read(reinterpret_cast<char*>(buffer->data()), static_cast<std::streamsize>(size)))
        throw DicomError("Failed to read " + path.string());
    ParsedFile pf = parseBuffer(std::move(buffer), options);
    if (pf.truncated && !partial && !options.stopBeforePixelData)
        throw DicomError("Truncated DICOM file: " + path.string());
    return pf;
}

bool looksLikeDicom(const std::filesystem::path& path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in)
        return false;
    char buf[132];
    if (!in.read(buf, 132))
        return false;
    return std::memcmp(buf + 128, "DICM", 4) == 0;
}

} // namespace occlusa::dicom
