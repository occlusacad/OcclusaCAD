#pragma once

#include "core/Progress.h"

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace occlusa::ui {

// Runs one long operation at a time on a worker thread and shows a modal progress
// dialog. The worker returns a continuation that is executed on the UI thread
// (where OpenGL resources may be created).
class TaskRunner {
public:
    using Continuation = std::function<void()>;
    using Work = std::function<Continuation(const ProgressFn& progress)>;

    ~TaskRunner();

    // Returns false if another task is running.
    bool start(std::string title, Work work, bool cancellable = true);
    bool busy() const { return state_ != nullptr; }

    // Call once per frame on the UI thread: finishes completed tasks and draws the progress modal.
    void update();

private:
    struct State {
        std::string title;
        bool cancellable = true;
        std::atomic<bool> done{false};
        std::atomic<bool> cancel{false};
        std::mutex mutex;
        float progress = 0.0f;
        std::string message;
        Continuation continuation;
        std::string error;
        bool cancelled = false;
        std::thread thread;
    };
    std::unique_ptr<State> state_;
};

} // namespace occlusa::ui
