#include "ui/Task.h"

#include "core/Log.h"
#include "ui/App.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"

#include <imgui.h>

namespace occlusa::ui {

TaskRunner::~TaskRunner()
{
    if (state_) {
        state_->cancel = true;
        if (state_->thread.joinable())
            state_->thread.join();
    }
}

bool TaskRunner::start(std::string title, Work work, bool cancellable)
{
    if (state_)
        return false;
    state_ = std::make_unique<State>();
    State* s = state_.get();
    s->title = std::move(title);
    s->cancellable = cancellable;
    s->message = "Starting...";
    s->thread = std::thread([s, work = std::move(work)] {
        ProgressFn progress = [s](float f, const std::string& msg) {
            {
                std::lock_guard lock(s->mutex);
                s->progress = f;
                s->message = msg;
            }
            notifyActivity();
            return !s->cancel.load();
        };
        try {
            Continuation c = work(progress);
            std::lock_guard lock(s->mutex);
            s->continuation = std::move(c);
        } catch (const CancelledError&) {
            std::lock_guard lock(s->mutex);
            s->cancelled = true;
        } catch (const std::exception& e) {
            std::lock_guard lock(s->mutex);
            s->error = e.what();
        } catch (...) {
            std::lock_guard lock(s->mutex);
            s->error = "Unknown error";
        }
        s->done = true;
        notifyActivity();
    });
    return true;
}

void TaskRunner::update()
{
    if (!state_)
        return;
    State* s = state_.get();
    if (s->done) {
        // Close the progress modal explicitly so it never lingers on the popup stack.
        if (ImGui::BeginPopupModal("##task_progress", nullptr, ImGuiWindowFlags_NoTitleBar)) {
            ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
        s->thread.join();
        Continuation c;
        std::string error, title = s->title;
        bool cancelled = false;
        {
            std::lock_guard lock(s->mutex);
            c = std::move(s->continuation);
            error = s->error;
            cancelled = s->cancelled;
        }
        state_.reset();
        if (!error.empty()) {
            log::error("{} failed: {}", title, error);
            showError(title, error);
        } else if (cancelled) {
            toast(ToastKind::Warning, title + " cancelled");
        } else if (c) {
            try {
                c();
            } catch (const std::exception& e) {
                log::error("{} failed: {}", title, e.what());
                showError(title, e.what());
            }
        }
        return;
    }

    float progress;
    std::string message;
    {
        std::lock_guard lock(s->mutex);
        progress = s->progress;
        message = s->message;
    }
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(ImGui::GetFontSize() * 26, 0));
    if (!ImGui::IsPopupOpen("##task_progress"))
        ImGui::OpenPopup("##task_progress");
    if (ImGui::BeginPopupModal("##task_progress", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize)) {
        subheading(s->title.c_str());
        ImGui::Spacing();
        ImGui::ProgressBar(progress, ImVec2(-1, 0));
        mutedText("%s", message.c_str());
        if (s->cancellable) {
            ImGui::Spacing();
            if (button(s->cancel ? "Cancelling..." : "Cancel", ImVec2(-1, 0), !s->cancel))
                s->cancel = true;
        }
        ImGui::EndPopup();
    }
}

} // namespace occlusa::ui
