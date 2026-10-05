#include "core/dicom/DicomCodecs.h"

#include "core/dicom/DicomDataset.h"
#include "core/dicom/DicomTags.h"

#include <array>
#include <cstring>

namespace occlusa::dicom {

bool isEncapsulatedSyntax(const std::string& uid)
{
    return !(uid == uids::ImplicitVRLittleEndian || uid == uids::ExplicitVRLittleEndian || uid == uids::ExplicitVRBigEndian ||
             uid == uids::DeflatedExplicitVRLittleEndian);
}

bool isTransferSyntaxSupported(const std::string& uid)
{
    return uid == uids::ImplicitVRLittleEndian || uid == uids::ExplicitVRLittleEndian || uid == uids::ExplicitVRBigEndian ||
           uid == uids::RleLossless || uid == uids::JpegLossless || uid == uids::JpegLosslessSV1;
}

std::string transferSyntaxName(const std::string& uid)
{
    if (uid == uids::ImplicitVRLittleEndian) return "Implicit VR Little Endian";
    if (uid == uids::ExplicitVRLittleEndian) return "Explicit VR Little Endian";
    if (uid == uids::ExplicitVRBigEndian) return "Explicit VR Big Endian";
    if (uid == uids::DeflatedExplicitVRLittleEndian) return "Deflated Explicit VR Little Endian";
    if (uid == uids::JpegBaseline) return "JPEG Baseline";
    if (uid == uids::JpegLossless) return "JPEG Lossless";
    if (uid == uids::JpegLosslessSV1) return "JPEG Lossless SV1";
    if (uid == uids::JpegLsLossless) return "JPEG-LS Lossless";
    if (uid == uids::JpegLsNearLossless) return "JPEG-LS Near Lossless";
    if (uid == uids::Jpeg2000Lossless) return "JPEG 2000 Lossless";
    if (uid == uids::Jpeg2000) return "JPEG 2000";
    if (uid == uids::RleLossless) return "RLE Lossless";
    return uid;
}

// ---------------------------------------------------------------------------
// RLE Lossless (PS3.5 Annex G)
// ---------------------------------------------------------------------------

namespace {

std::uint32_t le32(const std::uint8_t* p)
{
    return p[0] | (static_cast<std::uint32_t>(p[1]) << 8) | (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

void decodePackBits(std::span<const std::uint8_t> in, std::uint8_t* out, std::size_t outSize)
{
    std::size_t i = 0, o = 0;
    while (i < in.size() && o < outSize) {
        const auto n = static_cast<std::int8_t>(in[i++]);
        if (n >= 0) {
            const std::size_t count = static_cast<std::size_t>(n) + 1;
            if (i + count > in.size())
                throw DicomError("RLE literal run overflows segment");
            const std::size_t c = std::min(count, outSize - o);
            std::memcpy(out + o, in.data() + i, c);
            i += count;
            o += c;
        } else if (n != -128) {
            const std::size_t count = static_cast<std::size_t>(1 - n);
            if (i >= in.size())
                throw DicomError("RLE replicate run truncated");
            const std::uint8_t v = in[i++];
            const std::size_t c = std::min(count, outSize - o);
            std::memset(out + o, v, c);
            o += c;
        }
    }
    if (o < outSize)
        std::memset(out + o, 0, outSize - o); // tolerate short segments
}

} // namespace

std::vector<std::uint8_t> decodeRle(std::span<const std::uint8_t> frame, int rows, int cols, int bitsAllocated)
{
    if (frame.size() < 64)
        throw DicomError("RLE frame too short");
    const std::uint32_t segments = le32(frame.data());
    const int bytesPerSample = bitsAllocated / 8;
    if (segments != static_cast<std::uint32_t>(bytesPerSample) || segments > 15)
        throw DicomError("Unsupported RLE segment layout (only single-sample grayscale)");
    const std::size_t pixels = static_cast<std::size_t>(rows) * cols;
    std::vector<std::uint8_t> planes(pixels * segments);
    for (std::uint32_t s = 0; s < segments; ++s) {
        const std::uint32_t start = le32(frame.data() + 4 + 4 * s);
        std::uint32_t end = (s + 1 < segments) ? le32(frame.data() + 8 + 4 * s) : static_cast<std::uint32_t>(frame.size());
        if (start > frame.size() || end > frame.size() || end < start)
            throw DicomError("Corrupt RLE segment offsets");
        decodePackBits(frame.subspan(start, end - start), planes.data() + s * pixels, pixels);
    }
    // Segments are ordered most significant byte first; output little endian.
    std::vector<std::uint8_t> out(pixels * bytesPerSample);
    for (std::size_t p = 0; p < pixels; ++p)
        for (int b = 0; b < bytesPerSample; ++b)
            out[p * bytesPerSample + b] = planes[(bytesPerSample - 1 - b) * pixels + p];
    return out;
}

// ---------------------------------------------------------------------------
// JPEG Lossless, process 14 (ITU-T T.81 Annex H), single component.
// ---------------------------------------------------------------------------

namespace {

struct HuffmanTable {
    bool defined = false;
    std::array<std::uint8_t, 17> bits{};      // bits[l] = number of codes of length l (1..16)
    std::array<std::uint8_t, 256> values{};
    std::array<std::int32_t, 18> maxcode{};
    std::array<std::int32_t, 17> valptr{};
    std::array<std::int32_t, 17> mincode{};
    // Fast lookup on the first 8 bits: (length << 8) | value, 0 when the code is longer.
    std::array<std::uint16_t, 256> fast{};

    void build()
    {
        std::array<std::uint16_t, 257> codes{};
        std::array<std::uint8_t, 257> lengths{};
        int k = 0;
        std::uint16_t code = 0;
        for (int l = 1; l <= 16; ++l) {
            valptr[l] = k;
            mincode[l] = code;
            for (int i = 0; i < bits[l]; ++i) {
                if (k >= 256)
                    throw DicomError("JPEG Huffman table too large");
                codes[k] = code++;
                lengths[k] = static_cast<std::uint8_t>(l);
                ++k;
            }
            maxcode[l] = bits[l] ? code - 1 : -1;
            code <<= 1;
        }
        maxcode[17] = 0x7FFFFFFF;
        fast.fill(0);
        for (int i = 0; i < k; ++i) {
            if (lengths[i] > 8)
                continue;
            const int shift = 8 - lengths[i];
            const int first = codes[i] << shift;
            for (int j = 0; j < (1 << shift); ++j)
                fast[first + j] = static_cast<std::uint16_t>((lengths[i] << 8) | values[i]);
        }
        defined = true;
    }
};

class BitReader {
public:
    BitReader(const std::uint8_t* p, const std::uint8_t* end) : p_(p), end_(end) {}

    std::uint32_t peek16()
    {
        fill();
        return buf_ >> 16;
    }
    void skip(int n)
    {
        buf_ <<= n;
        count_ -= n;
    }
    std::uint32_t get(int n)
    {
        if (n == 0)
            return 0;
        fill();
        const std::uint32_t v = buf_ >> (32 - n);
        skip(n);
        return v;
    }

    // Handle an RSTn marker: drop remaining bits and skip the marker.
    void restart()
    {
        buf_ = 0;
        count_ = 0;
        markerHit_ = false;
        while (p_ + 1 < end_) {
            if (p_[0] == 0xFF && p_[1] >= 0xD0 && p_[1] <= 0xD7) {
                p_ += 2;
                return;
            }
            ++p_;
        }
    }

private:
    void fill()
    {
        while (count_ <= 24) {
            std::uint32_t byte = 0;
            if (!markerHit_ && p_ < end_) {
                byte = *p_;
                if (byte == 0xFF) {
                    const std::uint8_t next = (p_ + 1 < end_) ? p_[1] : 0;
                    if (next == 0x00) {
                        p_ += 2;
                    } else if (next == 0xFF) {
                        ++p_; // fill byte
                        continue;
                    } else {
                        markerHit_ = true; // don't consume markers; feed zeros
                        byte = 0;
                    }
                } else {
                    ++p_;
                }
            }
            buf_ |= byte << (24 - count_);
            count_ += 8;
        }
    }

    const std::uint8_t* p_;
    const std::uint8_t* end_;
    std::uint32_t buf_ = 0;
    int count_ = 0;
    bool markerHit_ = false;
};

int decodeHuffman(BitReader& br, const HuffmanTable& t)
{
    const std::uint32_t look = br.peek16();
    const std::uint16_t f = t.fast[look >> 8];
    if (f) {
        br.skip(f >> 8);
        return f & 0xFF;
    }
    std::int32_t code = static_cast<std::int32_t>(look >> 7); // first 9 bits
    int l = 9;
    while (l <= 16 && code > t.maxcode[l]) {
        ++l;
        code = static_cast<std::int32_t>(look >> (16 - l));
    }
    if (l > 16)
        throw DicomError("Corrupt JPEG Huffman code");
    br.skip(l);
    return t.values[t.valptr[l] + code - t.mincode[l]];
}

std::uint16_t be16(const std::uint8_t* p)
{
    return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
}

} // namespace

std::vector<std::uint16_t> decodeJpegLossless(std::span<const std::uint8_t> data, int& rows, int& cols, int& precision)
{
    const std::uint8_t* p = data.data();
    const std::uint8_t* end = p + data.size();
    if (data.size() < 4 || p[0] != 0xFF || p[1] != 0xD8)
        throw DicomError("JPEG stream missing SOI marker");
    p += 2;

    HuffmanTable tables[4];
    int restartInterval = 0;
    int componentId = -1;
    bool haveFrame = false;

    while (p + 4 <= end) {
        if (*p != 0xFF) {
            ++p;
            continue;
        }
        const std::uint8_t marker = p[1];
        p += 2;
        if (marker == 0xD8 || marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7) || marker == 0xFF) {
            if (marker == 0xFF)
                --p;
            continue;
        }
        if (marker == 0xD9)
            break;
        if (p + 2 > end)
            break;
        const std::uint16_t len = be16(p);
        const std::uint8_t* seg = p + 2;
        const std::uint8_t* segEnd = p + len;
        if (segEnd > end || len < 2)
            throw DicomError("Truncated JPEG segment");

        switch (marker) {
        case 0xC3: { // SOF3 lossless
            precision = seg[0];
            rows = be16(seg + 1);
            cols = be16(seg + 3);
            const int nf = seg[5];
            if (nf != 1)
                throw DicomError("Only single-component JPEG Lossless is supported");
            componentId = seg[6];
            haveFrame = true;
            break;
        }
        case 0xC0: case 0xC1: case 0xC2: case 0xC5: case 0xC6: case 0xC7:
        case 0xC9: case 0xCA: case 0xCB: case 0xCD: case 0xCE: case 0xCF:
            throw DicomError("Only JPEG Lossless (process 14) is supported");
        case 0xC4: { // DHT
            const std::uint8_t* q = seg;
            while (q < segEnd) {
                const int tc = q[0] >> 4, th = q[0] & 0x0F;
                if (th > 3 || tc != 0)
                    throw DicomError("Unexpected JPEG Huffman table class/id");
                HuffmanTable& t = tables[th];
                int total = 0;
                for (int l = 1; l <= 16; ++l) {
                    t.bits[l] = q[l];
                    total += q[l];
                }
                if (total > 256 || q + 17 + total > segEnd)
                    throw DicomError("Corrupt JPEG Huffman table");
                std::memcpy(t.values.data(), q + 17, static_cast<std::size_t>(total));
                t.build();
                q += 17 + total;
            }
            break;
        }
        case 0xDD: // DRI
            restartInterval = be16(seg);
            break;
        case 0xDA: { // SOS
            if (!haveFrame)
                throw DicomError("JPEG SOS before SOF");
            const int ns = seg[0];
            if (ns != 1)
                throw DicomError("Only single-component scans are supported");
            const int cs = seg[1];
            const int td = seg[2] >> 4;
            (void)cs;
            (void)componentId;
            const int predictor = seg[3];
            const int pt = seg[5] & 0x0F;
            if (td > 3 || !tables[td].defined)
                throw DicomError("JPEG scan references undefined Huffman table");
            if (predictor < 1 || predictor > 7)
                throw DicomError("Invalid JPEG lossless predictor");
            const HuffmanTable& table = tables[td];

            const std::size_t total = static_cast<std::size_t>(rows) * cols;
            std::vector<std::uint16_t> out(total);
            BitReader br(segEnd, end);
            const int initial = 1 << (precision - pt - 1);
            const std::uint32_t mask = (precision >= 16) ? 0xFFFFu : ((1u << precision) - 1u);

            bool firstLine = true, firstSample = true;
            int sinceRestart = 0;
            for (int y = 0; y < rows; ++y) {
                std::uint16_t* row = out.data() + static_cast<std::size_t>(y) * cols;
                const std::uint16_t* above = y > 0 ? row - cols : nullptr;
                for (int x = 0; x < cols; ++x) {
                    if (restartInterval > 0 && sinceRestart == restartInterval) {
                        br.restart();
                        sinceRestart = 0;
                        firstSample = true;
                        firstLine = true;
                    }
                    const int ssss = decodeHuffman(br, table);
                    int diff = 0;
                    if (ssss == 16) {
                        diff = 32768;
                    } else if (ssss > 0) {
                        const int bits = static_cast<int>(br.get(ssss));
                        diff = bits < (1 << (ssss - 1)) ? bits - (1 << ssss) + 1 : bits;
                    }
                    int pred;
                    if (firstSample) {
                        pred = initial;
                        firstSample = false;
                    } else if (firstLine) {
                        pred = row[x - 1];
                    } else if (x == 0) {
                        pred = above[0];
                    } else {
                        const int ra = row[x - 1], rb = above[x], rc = above[x - 1];
                        switch (predictor) {
                        case 1: pred = ra; break;
                        case 2: pred = rb; break;
                        case 3: pred = rc; break;
                        case 4: pred = ra + rb - rc; break;
                        case 5: pred = ra + ((rb - rc) >> 1); break;
                        case 6: pred = rb + ((ra - rc) >> 1); break;
                        default: pred = (ra + rb) >> 1; break;
                        }
                    }
                    row[x] = static_cast<std::uint16_t>(static_cast<std::uint32_t>(pred + diff) & mask);
                    ++sinceRestart;
                }
                firstLine = false;
            }
            if (pt > 0)
                for (auto& v : out)
                    v = static_cast<std::uint16_t>(v << pt);
            return out;
        }
        default:
            break; // APPn, COM, DQT etc.
        }
        p = segEnd;
    }
    throw DicomError("JPEG stream contains no scan");
}

std::vector<std::uint8_t> decodeFrame(const std::string& ts, std::span<const std::uint8_t> frame, int rows, int cols, int bitsAllocated)
{
    if (ts == uids::RleLossless)
        return decodeRle(frame, rows, cols, bitsAllocated);
    if (ts == uids::JpegLossless || ts == uids::JpegLosslessSV1) {
        int r = 0, c = 0, prec = 0;
        auto samples = decodeJpegLossless(frame, r, c, prec);
        if (r != rows || c != cols)
            throw DicomError("JPEG frame size does not match DICOM header");
        std::vector<std::uint8_t> out(samples.size() * (bitsAllocated / 8));
        if (bitsAllocated == 16) {
            for (std::size_t i = 0; i < samples.size(); ++i) {
                out[2 * i] = static_cast<std::uint8_t>(samples[i] & 0xFF);
                out[2 * i + 1] = static_cast<std::uint8_t>(samples[i] >> 8);
            }
        } else if (bitsAllocated == 8) {
            for (std::size_t i = 0; i < samples.size(); ++i)
                out[i] = static_cast<std::uint8_t>(samples[i]);
        } else {
            throw DicomError("Unsupported bits allocated for JPEG Lossless");
        }
        return out;
    }
    throw DicomError("Unsupported transfer syntax: " + transferSyntaxName(ts));
}

} // namespace occlusa::dicom
