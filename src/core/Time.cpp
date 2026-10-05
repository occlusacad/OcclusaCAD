#include "core/Time.h"

#include <cstdio>

namespace occlusa::time {

std::tm localTime(std::time_t t)
{
    std::tm out{};
#if defined(_WIN32)
    localtime_s(&out, &t);
#else
    localtime_r(&t, &out);
#endif
    return out;
}

std::tm utcTime(std::time_t t)
{
    std::tm out{};
#if defined(_WIN32)
    gmtime_s(&out, &t);
#else
    gmtime_r(&t, &out);
#endif
    return out;
}

namespace {
std::string formatTm(const std::tm& tm, const char* fmt)
{
    char buf[64];
    const std::size_t n = std::strftime(buf, sizeof(buf), fmt, &tm);
    return std::string(buf, n);
}

// Inverse of gmtime without relying on non-standard timegm().
std::time_t utcToTimeT(std::tm tm)
{
    // Days from civil algorithm (Howard Hinnant), valid for the proleptic Gregorian calendar.
    int y = tm.tm_year + 1900;
    const unsigned m = static_cast<unsigned>(tm.tm_mon + 1);
    const unsigned d = static_cast<unsigned>(tm.tm_mday);
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    const long long days = static_cast<long long>(era) * 146097 + static_cast<long long>(doe) - 719468;
    return static_cast<std::time_t>(days * 86400 + tm.tm_hour * 3600 + tm.tm_min * 60 + tm.tm_sec);
}
} // namespace

std::string nowUtcIso8601()
{
    return formatTm(utcTime(std::time(nullptr)), "%Y-%m-%dT%H:%M:%SZ");
}

std::string nowLocalClock()
{
    return formatTm(localTime(std::time(nullptr)), "%H:%M:%S");
}

std::string todayLocalDate()
{
    return formatTm(localTime(std::time(nullptr)), "%Y-%m-%d");
}

std::string utcIsoToLocalDisplay(const std::string& iso)
{
    std::tm tm{};
    if (std::sscanf(iso.c_str(), "%d-%d-%dT%d:%d:%d", &tm.tm_year, &tm.tm_mon, &tm.tm_mday, &tm.tm_hour, &tm.tm_min, &tm.tm_sec) != 6)
        return iso;
    tm.tm_year -= 1900;
    tm.tm_mon -= 1;
    return formatTm(localTime(utcToTimeT(tm)), "%Y-%m-%d %H:%M");
}

} // namespace occlusa::time
