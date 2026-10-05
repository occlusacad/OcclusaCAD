#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace occlusa {

// Minimal "--key value" / "--flag" command line parser. Arguments are UTF-8 on all platforms.
class CommandLine {
public:
    static CommandLine fromMain(int argc, char** argv);
    static CommandLine fromArgs(const std::vector<std::string>& args);

    bool has(const std::string& key) const { return options_.count(key) != 0; }
    std::optional<std::string> get(const std::string& key) const;
    const std::vector<std::string>& positional() const { return positional_; }

private:
    std::map<std::string, std::string> options_;
    std::vector<std::string> positional_;
};

} // namespace occlusa
