#include "TestUtil.h"

#include "core/dicom/DicomCodecs.h"
#include "core/dicom/DicomDataset.h"
#include "core/dicom/DicomSeries.h"
#include "core/dicom/DicomWriter.h"

#include <doctest.h>

#include <cmath>

using namespace occlusa;

namespace {

void checkVolumesEqual(const Volume& a, const Volume& b)
{
    REQUIRE(a.geometry.dims == b.geometry.dims);
    for (int i = 0; i < 3; ++i) {
        CHECK(a.geometry.spacing[i] == doctest::Approx(b.geometry.spacing[i]).epsilon(1e-5));
        CHECK(a.geometry.origin[i] == doctest::Approx(b.geometry.origin[i]).epsilon(1e-5));
        for (int j = 0; j < 3; ++j)
            CHECK(a.geometry.direction[i][j] == doctest::Approx(b.geometry.direction[i][j]).epsilon(1e-5));
    }
    REQUIRE(a.voxels.size() == b.voxels.size());
    std::size_t mismatches = 0;
    for (std::size_t i = 0; i < a.voxels.size(); ++i) {
        const double va = a.voxels[i] * a.rescaleSlope + a.rescaleIntercept;
        const double vb = b.voxels[i] * b.rescaleSlope + b.rescaleIntercept;
        if (std::abs(va - vb) > 1e-6)
            ++mismatches;
    }
    CHECK(mismatches == 0);
}

// Minimal JPEG Lossless (process 14, SV1-7) encoder used to exercise the decoder.
std::vector<std::uint8_t> encodeJpegLossless(const std::vector<std::uint16_t>& img, int rows, int cols, int predictor, int restartInterval)
{
    std::vector<std::uint8_t> out = {0xFF, 0xD8};
    auto marker = [&](std::uint8_t m, const std::vector<std::uint8_t>& payload) {
        out.push_back(0xFF);
        out.push_back(m);
        const auto len = static_cast<std::uint16_t>(payload.size() + 2);
        out.push_back(static_cast<std::uint8_t>(len >> 8));
        out.push_back(static_cast<std::uint8_t>(len & 0xFF));
        out.insert(out.end(), payload.begin(), payload.end());
    };
    marker(0xC3, {16, static_cast<std::uint8_t>(rows >> 8), static_cast<std::uint8_t>(rows), static_cast<std::uint8_t>(cols >> 8),
                  static_cast<std::uint8_t>(cols), 1, 1, 0x11, 0});
    // Huffman table: 17 symbols (0..16), all 5-bit codes 0..16.
    std::vector<std::uint8_t> dht = {0x00};
    for (int l = 1; l <= 16; ++l)
        dht.push_back(l == 5 ? 17 : 0);
    for (int s = 0; s <= 16; ++s)
        dht.push_back(static_cast<std::uint8_t>(s));
    marker(0xC4, dht);
    if (restartInterval > 0)
        marker(0xDD, {static_cast<std::uint8_t>(restartInterval >> 8), static_cast<std::uint8_t>(restartInterval)});
    marker(0xDA, {1, 1, 0x00, static_cast<std::uint8_t>(predictor), 0, 0});

    std::uint32_t acc = 0;
    int nbits = 0;
    auto flushByte = [&](std::uint8_t b) {
        out.push_back(b);
        if (b == 0xFF)
            out.push_back(0x00);
    };
    auto putBits = [&](std::uint32_t v, int n) {
        for (int i = n - 1; i >= 0; --i) {
            acc = (acc << 1) | ((v >> i) & 1);
            if (++nbits == 8) {
                flushByte(static_cast<std::uint8_t>(acc));
                acc = 0;
                nbits = 0;
            }
        }
    };
    auto padByte = [&] {
        while (nbits != 0)
            putBits(1, 1);
    };

    int sinceRestart = 0, rstIndex = 0;
    bool firstLine = true, firstSample = true;
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (restartInterval > 0 && sinceRestart == restartInterval) {
                padByte();
                out.push_back(0xFF);
                out.push_back(static_cast<std::uint8_t>(0xD0 + (rstIndex++ & 7)));
                sinceRestart = 0;
                firstLine = true;
                firstSample = true;
            }
            const auto at = [&](int yy, int xx) { return static_cast<int>(img[static_cast<std::size_t>(yy) * cols + xx]); };
            int pred;
            if (firstSample) {
                pred = 1 << 15;
                firstSample = false;
            } else if (firstLine) {
                pred = at(y, x - 1);
            } else if (x == 0) {
                pred = at(y - 1, 0);
            } else {
                const int ra = at(y, x - 1), rb = at(y - 1, x), rc = at(y - 1, x - 1);
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
            int diff = (at(y, x) - pred) & 0xFFFF;
            if (diff > 32768)
                diff -= 65536;
            int ssss = 0;
            for (int a = std::abs(diff); a > 0; a >>= 1)
                ++ssss;
            putBits(static_cast<std::uint32_t>(ssss), 5);
            if (ssss > 0 && ssss < 16) {
                const int bits = diff > 0 ? diff : diff + (1 << ssss) - 1;
                putBits(static_cast<std::uint32_t>(bits), ssss);
            }
            ++sinceRestart;
        }
        firstLine = false;
    }
    padByte();
    out.push_back(0xFF);
    out.push_back(0xD9);
    return out;
}

} // namespace

TEST_CASE("DICOM series round trip (explicit, implicit, RLE)")
{
    // Oblique orientation and anisotropic spacing to exercise the geometry code.
    const glm::dmat3 rot = glm::dmat3(glm::rotate(glm::dmat4(1.0), glm::radians(12.0), glm::normalize(glm::dvec3(0.2, 0.3, 1.0))));
    Volume vol = test::makeBlobVolume({40, 36, 30}, {0.9, 0.8, 1.1}, rot);
    vol.rescaleSlope = 1.0;
    vol.rescaleIntercept = -1000.0;

    for (int variant = 0; variant < 3; ++variant) {
        CAPTURE(variant);
        test::TempDir tmp;
        dicom::WriteOptions opt;
        opt.explicitVR = variant != 1;
        opt.rleCompress = variant == 2;
        dicom::writeSeries(vol, tmp.path(), opt);

        const auto scan = dicom::scanForSeries(tmp.path());
        REQUIRE(scan.series.size() == 1);
        CHECK(scan.series[0].sliceCount == 30);
        CHECK(scan.series[0].supported);
        const Volume back = dicom::loadSeries(scan.series[0]);
        checkVolumesEqual(vol, back);
        CHECK(back.info.patientName == "PHANTOM, SYNTHETIC");
    }
}

TEST_CASE("DICOM slices are sorted by position, not file name")
{
    Volume vol = test::makeBlobVolume({16, 16, 12}, {1.0, 1.0, 1.0});
    test::TempDir tmp;
    dicom::writeSeries(vol, tmp.path());
    // Rename files in reverse order.
    std::vector<std::filesystem::path> files;
    for (auto& e : std::filesystem::directory_iterator(tmp.path()))
        files.push_back(e.path());
    std::sort(files.begin(), files.end());
    for (std::size_t i = 0; i < files.size(); ++i)
        std::filesystem::rename(files[i], tmp.path() / ("x" + std::to_string(1000 - i)));
    const Volume back = dicom::loadLargestSeries(tmp.path());
    checkVolumesEqual(vol, back);
}

TEST_CASE("Header-only parse stops before pixel data")
{
    Volume vol = test::makeBlobVolume({16, 16, 2}, {1.0, 1.0, 1.0});
    test::TempDir tmp;
    dicom::writeSeries(vol, tmp.path());
    const auto file = tmp.path() / "IM00001.dcm";
    dicom::ParseOptions opts;
    opts.stopBeforePixelData = true;
    const auto pf = dicom::parseFile(file, opts);
    CHECK(pf.data.has(dicom::tags::PixelData));
    CHECK(pf.data.getInt(dicom::tags::Rows).value() == 16);
    CHECK(pf.data.getNumbers(dicom::tags::ImagePositionPatient).size() == 3);
    CHECK(dicom::looksLikeDicom(file));
}

TEST_CASE("JPEG Lossless decoder")
{
    const int rows = 37, cols = 53;
    std::vector<std::uint16_t> img(static_cast<std::size_t>(rows) * cols);
    std::mt19937 rng(42);
    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x)
            img[static_cast<std::size_t>(y) * cols + x] =
                static_cast<std::uint16_t>(30000 + 2000 * std::sin(x * 0.3) + 1500 * std::cos(y * 0.2) + (rng() % 50) + ((x * y) % 7 == 0 ? 20000 : 0));
    // Include extreme values to exercise SSSS = 16.
    img[5] = 0;
    img[6] = 65535;

    for (int predictor = 1; predictor <= 7; ++predictor) {
        for (int restart : {0, 64}) {
            CAPTURE(predictor);
            CAPTURE(restart);
            const auto jpeg = encodeJpegLossless(img, rows, cols, predictor, restart);
            int r = 0, c = 0, p = 0;
            const auto decoded = dicom::decodeJpegLossless(jpeg, r, c, p);
            CHECK(r == rows);
            CHECK(c == cols);
            CHECK(p == 16);
            CHECK(decoded == img);
        }
    }
}

TEST_CASE("Latin-1 conversion")
{
    CHECK(dicom::latin1ToUtf8("M\xFCller") == "M\xC3\xBCller");
}
