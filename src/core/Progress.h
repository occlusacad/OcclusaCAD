#pragma once

#include <functional>
#include <string>

namespace occlusa {

// Progress reporting for long operations. Return false to request cancellation.
using ProgressFn = std::function<bool(float fraction, const std::string& message)>;

class CancelledError : public std::exception {
public:
    const char* what() const noexcept override { return "Operation cancelled"; }
};

inline void reportProgress(const ProgressFn& fn, float fraction, const std::string& message)
{
    if (fn && !fn(fraction, message))
        throw CancelledError();
}

} // namespace occlusa
