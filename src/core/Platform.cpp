#include "core/Platform.h"

#include "core/Log.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(_WIN32)
#include <windows.h>
#include <shellapi.h>
#include <shlobj.h>
#include <knownfolders.h>
#else
#include <pwd.h>
#include <spawn.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif

#if !defined(_WIN32)
extern char** environ;
#endif

namespace fs = std::filesystem;

namespace occlusa::platform {

fs::path pathFromUtf8(const std::string& utf8)
{
    std::u8string u8(utf8.begin(), utf8.end());
    return fs::path(u8);
}

std::string pathToUtf8(const fs::path& path)
{
    const std::u8string u8 = path.u8string();
    return std::string(u8.begin(), u8.end());
}

namespace {

std::string getEnv(const char* name)
{
#if defined(_WIN32)
    // Use the wide API so non-ASCII values survive.
    std::wstring wname(name, name + std::strlen(name));
    DWORD n = GetEnvironmentVariableW(wname.c_str(), nullptr, 0);
    if (n == 0)
        return {};
    std::wstring value(n, L'\0');
    n = GetEnvironmentVariableW(wname.c_str(), value.data(), n);
    value.resize(n);
    return pathToUtf8(fs::path(value));
#else
    const char* v = std::getenv(name);
    return v ? std::string(v) : std::string();
#endif
}

#if !defined(_WIN32)
// Run a command and capture its stdout (used for best-effort desktop queries).
std::optional<std::string> captureCommand(const char* cmd)
{
    FILE* pipe = popen(cmd, "r");
    if (!pipe)
        return std::nullopt;
    std::string out;
    char buf[256];
    while (std::fgets(buf, sizeof(buf), pipe))
        out += buf;
    const int rc = pclose(pipe);
    if (rc != 0)
        return std::nullopt;
    return out;
}
#endif

#if defined(_WIN32)
fs::path knownFolder(REFKNOWNFOLDERID id)
{
    PWSTR raw = nullptr;
    fs::path result;
    if (SUCCEEDED(SHGetKnownFolderPath(id, 0, nullptr, &raw)))
        result = fs::path(raw);
    CoTaskMemFree(raw);
    return result;
}
#endif

} // namespace

fs::path executableDir()
{
#if defined(_WIN32)
    std::wstring buf(MAX_PATH, L'\0');
    for (;;) {
        const DWORD n = GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
        if (n < buf.size()) {
            buf.resize(n);
            break;
        }
        buf.resize(buf.size() * 2);
    }
    return fs::path(buf).parent_path();
#elif defined(__APPLE__)
    uint32_t size = 0;
    _NSGetExecutablePath(nullptr, &size);
    std::string buf(size, '\0');
    if (_NSGetExecutablePath(buf.data(), &size) != 0)
        return fs::current_path();
    std::error_code ec;
    fs::path p = fs::canonical(fs::path(buf.c_str()), ec);
    return ec ? fs::path(buf.c_str()).parent_path() : p.parent_path();
#else
    std::error_code ec;
    fs::path p = fs::read_symlink("/proc/self/exe", ec);
    return ec ? fs::current_path() : p.parent_path();
#endif
}

fs::path homeDir()
{
#if defined(_WIN32)
    fs::path p = knownFolder(FOLDERID_Profile);
    if (!p.empty())
        return p;
    return pathFromUtf8(getEnv("USERPROFILE"));
#else
    std::string home = getEnv("HOME");
    if (home.empty()) {
        if (const passwd* pw = getpwuid(getuid()))
            home = pw->pw_dir;
    }
    return fs::path(home);
#endif
}

fs::path configDir()
{
#if defined(_WIN32)
    fs::path base = knownFolder(FOLDERID_RoamingAppData);
    if (base.empty())
        base = pathFromUtf8(getEnv("APPDATA"));
    return base / "OcclusaCAD";
#elif defined(__APPLE__)
    return homeDir() / "Library" / "Application Support" / "OcclusaCAD";
#else
    const std::string xdg = getEnv("XDG_CONFIG_HOME");
    fs::path base = xdg.empty() ? homeDir() / ".config" : fs::path(xdg);
    return base / "occlusacad";
#endif
}

fs::path documentsDir()
{
#if defined(_WIN32)
    fs::path p = knownFolder(FOLDERID_Documents);
    return p.empty() ? homeDir() : p;
#elif defined(__APPLE__)
    return homeDir() / "Documents";
#else
    if (auto out = captureCommand("xdg-user-dir DOCUMENTS 2>/dev/null")) {
        std::string s = *out;
        while (!s.empty() && (s.back() == '\n' || s.back() == '\r'))
            s.pop_back();
        if (!s.empty() && fs::is_directory(s))
            return fs::path(s);
    }
    fs::path docs = homeDir() / "Documents";
    return fs::is_directory(docs) ? docs : homeDir();
#endif
}

std::string userName()
{
#if defined(_WIN32)
    wchar_t buf[256];
    DWORD n = 256;
    if (GetUserNameW(buf, &n) && n > 0)
        return pathToUtf8(fs::path(std::wstring(buf, n - 1)));
    return getEnv("USERNAME");
#else
    if (const passwd* pw = getpwuid(getuid()))
        return pw->pw_name;
    return getEnv("USER");
#endif
}

std::string hostName()
{
#if defined(_WIN32)
    wchar_t buf[MAX_COMPUTERNAME_LENGTH + 1];
    DWORD n = MAX_COMPUTERNAME_LENGTH + 1;
    if (GetComputerNameW(buf, &n))
        return pathToUtf8(fs::path(std::wstring(buf, n)));
    return getEnv("COMPUTERNAME");
#else
    char buf[256] = {};
    if (gethostname(buf, sizeof(buf) - 1) == 0)
        return buf;
    return "localhost";
#endif
}

std::optional<bool> systemPrefersDark()
{
#if defined(_WIN32)
    DWORD value = 1;
    DWORD size = sizeof(value);
    const LSTATUS rc = RegGetValueW(HKEY_CURRENT_USER,
                                    L"Software\\Microsoft\\Windows\\CurrentVersion\\Themes\\Personalize",
                                    L"AppsUseLightTheme", RRF_RT_REG_DWORD, nullptr, &value, &size);
    if (rc != ERROR_SUCCESS)
        return std::nullopt;
    return value == 0;
#elif defined(__APPLE__)
    // "Dark" when dark mode is on; the key is absent (non-zero exit) in light mode.
    auto out = captureCommand("defaults read -g AppleInterfaceStyle 2>/dev/null");
    if (!out)
        return false;
    return out->find("Dark") != std::string::npos;
#else
    // freedesktop color-scheme via GNOME settings (also honoured by many other desktops).
    if (auto out = captureCommand("gsettings get org.gnome.desktop.interface color-scheme 2>/dev/null")) {
        if (out->find("prefer-dark") != std::string::npos)
            return true;
        if (out->find("prefer-light") != std::string::npos || out->find("default") != std::string::npos)
            return false;
    }
    if (auto out = captureCommand("gsettings get org.gnome.desktop.interface gtk-theme 2>/dev/null")) {
        if (out->find("dark") != std::string::npos || out->find("Dark") != std::string::npos)
            return true;
    }
    return std::nullopt;
#endif
}

bool launchDetached(const fs::path& executable, const std::vector<std::string>& args, std::string* error)
{
#if defined(_WIN32)
    auto quote = [](const std::wstring& s) {
        std::wstring out = L"\"";
        std::size_t backslashes = 0;
        for (wchar_t c : s) {
            if (c == L'\\') {
                ++backslashes;
            } else if (c == L'"') {
                out.append(backslashes * 2 + 1, L'\\');
                out.push_back(c);
                backslashes = 0;
                continue;
            } else {
                backslashes = 0;
            }
            out.push_back(c);
        }
        out.append(backslashes, L'\\');
        out.push_back(L'"');
        return out;
    };
    std::wstring cmd = quote(executable.wstring());
    for (const auto& a : args)
        cmd += L" " + quote(pathFromUtf8(a).wstring());

    STARTUPINFOW si{};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};
    if (!CreateProcessW(executable.wstring().c_str(), cmd.data(), nullptr, nullptr, FALSE,
                        DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP, nullptr, nullptr, &si, &pi)) {
        if (error)
            *error = "CreateProcess failed with error " + std::to_string(GetLastError());
        return false;
    }
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    return true;
#else
    const std::string exe = executable.string();
    std::vector<char*> argv;
    argv.push_back(const_cast<char*>(exe.c_str()));
    for (const auto& a : args)
        argv.push_back(const_cast<char*>(a.c_str()));
    argv.push_back(nullptr);

    posix_spawnattr_t attr;
    posix_spawnattr_init(&attr);
#if defined(POSIX_SPAWN_SETSID)
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSID);
#else
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
    posix_spawnattr_setpgroup(&attr, 0);
#endif
    pid_t pid = 0;
    const int rc = posix_spawn(&pid, exe.c_str(), nullptr, &attr, argv.data(), environ);
    posix_spawnattr_destroy(&attr);
    if (rc != 0) {
        if (error)
            *error = std::string("posix_spawn failed: ") + std::strerror(rc);
        return false;
    }
    // TODO: reap the child (SIGCHLD handler) to avoid a zombie if the DB app stays open
    // long after the designer exits. Zombies are harmless and cleaned up when the DB app exits.
    return true;
#endif
}

bool openInFileBrowser(const fs::path& path)
{
#if defined(_WIN32)
    const auto rc = reinterpret_cast<INT_PTR>(ShellExecuteW(nullptr, L"open", path.wstring().c_str(), nullptr, nullptr, SW_SHOWNORMAL));
    return rc > 32;
#elif defined(__APPLE__)
    return launchDetached("/usr/bin/open", {path.string()});
#else
    return launchDetached("/usr/bin/xdg-open", {path.string()}) || launchDetached("/bin/xdg-open", {path.string()});
#endif
}

std::string executableName(const std::string& baseName)
{
#if defined(_WIN32)
    return baseName + ".exe";
#else
    return baseName;
#endif
}

} // namespace occlusa::platform
