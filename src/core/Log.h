#pragma once

#include <cstdint>
#include <filesystem>
#include <format>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace occlusa::log {

enum class Level : std::uint8_t { Debug, Info, Warning, Error };

struct Entry {
    Level level;
    std::string timestamp; // HH:MM:SS
    std::string message;
};

// Mirror log output to a file (in addition to stderr and the in-memory ring buffer).
void setLogFile(const std::filesystem::path& path);
void setMinimumLevel(Level level);

void write(Level level, std::string_view message);

// Snapshot of recent entries for the in-app log panel (thread safe).
std::vector<Entry> recent(std::size_t maxEntries = 500);
std::uint64_t generation(); // increments on every write; cheap change detection

template <class... Args>
void debug(std::format_string<Args...> fmt, Args&&... args)
{
    write(Level::Debug, std::format(fmt, std::forward<Args>(args)...));
}
template <class... Args>
void info(std::format_string<Args...> fmt, Args&&... args)
{
    write(Level::Info, std::format(fmt, std::forward<Args>(args)...));
}
template <class... Args>
void warn(std::format_string<Args...> fmt, Args&&... args)
{
    write(Level::Warning, std::format(fmt, std::forward<Args>(args)...));
}
template <class... Args>
void error(std::format_string<Args...> fmt, Args&&... args)
{
    write(Level::Error, std::format(fmt, std::forward<Args>(args)...));
}

} // namespace occlusa::log
