#include "core/Log.h"

#include "core/Time.h"

#include <cstdio>
#include <deque>
#include <fstream>
#include <mutex>

namespace occlusa::log {
namespace {

struct State {
    std::mutex mutex;
    std::deque<Entry> ring;
    std::ofstream file;
    Level minLevel = Level::Info;
    std::uint64_t generation = 0;
};

State& state()
{
    static State s;
    return s;
}

const char* levelName(Level level)
{
    switch (level) {
    case Level::Debug: return "DEBUG";
    case Level::Info: return "INFO";
    case Level::Warning: return "WARN";
    case Level::Error: return "ERROR";
    }
    return "?";
}


} // namespace

void setLogFile(const std::filesystem::path& path)
{
    auto& s = state();
    std::lock_guard lock(s.mutex);
    s.file.close();
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    s.file.open(path, std::ios::out | std::ios::app);
}

void setMinimumLevel(Level level)
{
    auto& s = state();
    std::lock_guard lock(s.mutex);
    s.minLevel = level;
}

void write(Level level, std::string_view message)
{
    auto& s = state();
    const std::string ts = time::nowLocalClock();

    std::lock_guard lock(s.mutex);
    if (level < s.minLevel)
        return;
    std::fprintf(stderr, "[%s] %-5s %.*s\n", ts.c_str(), levelName(level), static_cast<int>(message.size()), message.data());
    if (s.file.is_open()) {
        s.file << '[' << ts << "] " << levelName(level) << ' ' << message << '\n';
        s.file.flush();
    }
    s.ring.push_back(Entry{level, ts, std::string(message)});
    while (s.ring.size() > 2000)
        s.ring.pop_front();
    ++s.generation;
}

std::vector<Entry> recent(std::size_t maxEntries)
{
    auto& s = state();
    std::lock_guard lock(s.mutex);
    const std::size_t n = std::min(maxEntries, s.ring.size());
    return {s.ring.end() - static_cast<std::ptrdiff_t>(n), s.ring.end()};
}

std::uint64_t generation()
{
    auto& s = state();
    std::lock_guard lock(s.mutex);
    return s.generation;
}

} // namespace occlusa::log
