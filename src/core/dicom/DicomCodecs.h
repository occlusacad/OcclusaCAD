#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace occlusa::dicom {

// Pixel data compression support.
//   Supported: native (uncompressed), RLE Lossless, JPEG Lossless (process 14, incl. SV1).
//   TODO: JPEG baseline/extended (lossy), JPEG-LS (CharLS, BSD-3), JPEG 2000 (OpenJPEG, BSD-2).
bool isTransferSyntaxSupported(const std::string& uid);
bool isEncapsulatedSyntax(const std::string& uid);
std::string transferSyntaxName(const std::string& uid);

// Decode one encapsulated frame into native little-endian samples
// (bitsAllocated/8 bytes per sample, single sample per pixel).
std::vector<std::uint8_t> decodeFrame(const std::string& transferSyntax, std::span<const std::uint8_t> frame, int rows, int cols,
                                      int bitsAllocated);

std::vector<std::uint8_t> decodeRle(std::span<const std::uint8_t> frame, int rows, int cols, int bitsAllocated);
std::vector<std::uint16_t> decodeJpegLossless(std::span<const std::uint8_t> data, int& rows, int& cols, int& precision);

} // namespace occlusa::dicom
