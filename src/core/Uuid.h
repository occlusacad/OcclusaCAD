#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace occlusa {

// Random (version 4) UUID, formatted "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx".
std::string generateUuid();
std::array<std::uint8_t, 16> generateUuidBytes();

// DICOM UID derived from a random UUID: "2.25.<128-bit decimal>" (PS3.5 B.2).
std::string generateDicomUid();

} // namespace occlusa
