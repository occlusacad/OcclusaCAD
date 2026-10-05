#include "core/CommandLine.h"

#include "core/Platform.h"

#if defined(_WIN32)
#include <windows.h>
#include <shellapi.h>
#endif

namespace occlusa {

CommandLine CommandLine::fromMain(int argc, char** argv)
{
    std::vector<std::string> args;
#if defined(_WIN32)
    // argv is in the ANSI code page on Windows; re-read the command line as UTF-16.
    (void)argc;
    (void)argv;
    int n = 0;
    LPWSTR* wargv = CommandLineToArgvW(GetCommandLineW(), &n);
    for (int i = 1; wargv && i < n; ++i)
        args.push_back(platform::pathToUtf8(std::filesystem::path(wargv[i])));
    LocalFree(wargv);
#else
    for (int i = 1; i < argc; ++i)
        args.emplace_back(argv[i]);
#endif
    return fromArgs(args);
}

CommandLine CommandLine::fromArgs(const std::vector<std::string>& args)
{
    CommandLine cl;
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& a = args[i];
        if (a.size() > 2 && a.rfind("--", 0) == 0) {
            std::string key = a.substr(2);
            const auto eq = key.find('=');
            if (eq != std::string::npos) {
                cl.options_[key.substr(0, eq)] = key.substr(eq + 1);
            } else if (i + 1 < args.size() && args[i + 1].rfind("--", 0) != 0) {
                cl.options_[key] = args[++i];
            } else {
                cl.options_[key] = "";
            }
        } else {
            cl.positional_.push_back(a);
        }
    }
    return cl;
}

std::optional<std::string> CommandLine::get(const std::string& key) const
{
    auto it = options_.find(key);
    if (it == options_.end())
        return std::nullopt;
    return it->second;
}

} // namespace occlusa
