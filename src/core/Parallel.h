#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

namespace occlusa {

inline unsigned workerCount()
{
    const unsigned hc = std::thread::hardware_concurrency();
    return std::clamp(hc == 0 ? 4u : hc, 1u, 32u);
}

// Run fn(i) for i in [0, n) on a pool of threads with dynamic scheduling.
// The first exception thrown by any worker is rethrown on the calling thread.
template <class Fn>
void parallelFor(std::size_t n, Fn&& fn, unsigned maxThreads = 0)
{
    if (n == 0)
        return;
    const unsigned threads = static_cast<unsigned>(std::min<std::size_t>(n, maxThreads ? maxThreads : workerCount()));
    if (threads <= 1) {
        for (std::size_t i = 0; i < n; ++i)
            fn(i);
        return;
    }
    std::atomic<std::size_t> next{0};
    std::exception_ptr error;
    std::mutex errorMutex;
    auto worker = [&] {
        for (;;) {
            const std::size_t i = next.fetch_add(1);
            if (i >= n)
                return;
            try {
                fn(i);
            } catch (...) {
                std::lock_guard lock(errorMutex);
                if (!error)
                    error = std::current_exception();
                next.store(n); // stop handing out work
            }
        }
    };
    std::vector<std::thread> pool;
    pool.reserve(threads - 1);
    for (unsigned t = 1; t < threads; ++t)
        pool.emplace_back(worker);
    worker();
    for (auto& th : pool)
        th.join();
    if (error)
        std::rethrow_exception(error);
}

} // namespace occlusa
