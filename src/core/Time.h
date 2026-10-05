#pragma once

#include <chrono>
#include <ctime>
#include <string>

namespace occlusa::time {

// Portable local/UTC broken-down time (avoids std::chrono tzdb which is not
// available on every standard library we target).
std::tm localTime(std::time_t t);
std::tm utcTime(std::time_t t);

// "2026-10-05T14:03:22Z"
std::string nowUtcIso8601();
// "14:03:22" in local time
std::string nowLocalClock();
// "2026-10-05" in local time
std::string todayLocalDate();
// Convert an ISO-8601 UTC timestamp ("...Z") to a short local display string "2026-10-05 10:03".
std::string utcIsoToLocalDisplay(const std::string& iso);

} // namespace occlusa::time
