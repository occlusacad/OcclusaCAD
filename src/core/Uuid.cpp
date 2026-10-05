#include "core/Uuid.h"

#include <algorithm>
#include <mutex>
#include <random>

namespace occlusa {

std::array<std::uint8_t, 16> generateUuidBytes()
{
    static std::mutex mutex;
    static std::mt19937_64 rng = [] {
        std::random_device rd;
        std::seed_seq seq{rd(), rd(), rd(), rd(), rd(), rd(), rd(), rd()};
        return std::mt19937_64(seq);
    }();
    std::array<std::uint8_t, 16> b{};
    {
        std::lock_guard lock(mutex);
        const std::uint64_t hi = rng(), lo = rng();
        for (int i = 0; i < 8; ++i) {
            b[i] = static_cast<std::uint8_t>(hi >> (56 - 8 * i));
            b[8 + i] = static_cast<std::uint8_t>(lo >> (56 - 8 * i));
        }
    }
    b[6] = static_cast<std::uint8_t>((b[6] & 0x0F) | 0x40); // version 4
    b[8] = static_cast<std::uint8_t>((b[8] & 0x3F) | 0x80); // RFC 4122 variant
    return b;
}

std::string generateUuid()
{
    const auto b = generateUuidBytes();
    static constexpr char hex[] = "0123456789abcdef";
    std::string out;
    out.reserve(36);
    for (int i = 0; i < 16; ++i) {
        if (i == 4 || i == 6 || i == 8 || i == 10)
            out.push_back('-');
        out.push_back(hex[b[i] >> 4]);
        out.push_back(hex[b[i] & 0x0F]);
    }
    return out;
}

std::string generateDicomUid()
{
    const auto b = generateUuidBytes();
    // Convert the 128-bit big-endian integer to decimal by repeated division of 32-bit limbs.
    std::array<std::uint32_t, 4> limbs{};
    for (int i = 0; i < 4; ++i)
        limbs[i] = (static_cast<std::uint32_t>(b[4 * i]) << 24) | (static_cast<std::uint32_t>(b[4 * i + 1]) << 16) |
                   (static_cast<std::uint32_t>(b[4 * i + 2]) << 8) | b[4 * i + 3];
    std::string digits;
    auto isZero = [&] { return std::all_of(limbs.begin(), limbs.end(), [](std::uint32_t v) { return v == 0; }); };
    while (!isZero()) {
        std::uint64_t rem = 0;
        for (auto& limb : limbs) {
            const std::uint64_t cur = (rem << 32) | limb;
            limb = static_cast<std::uint32_t>(cur / 10);
            rem = cur % 10;
        }
        digits.push_back(static_cast<char>('0' + rem));
    }
    if (digits.empty())
        digits = "0";
    std::reverse(digits.begin(), digits.end());
    return "2.25." + digits;
}

} // namespace occlusa
