#pragma once

#include <algorithm>
#include <limits>

namespace occlusa::ui {

// Decides when the windowed render loop draws a frame. Times are in seconds.
//
// A pending redraw (input, requestRedraw) draws at most once per `minFrameInterval`; without one,
// the loop only draws the idle tick every `idleInterval`. Event waits that return early without
// a redraw (Wayland wakes the loop after each frame with wl_display.delete_id) draw nothing, so
// the loop does not keep itself busy.
class FramePacer {
public:
    explicit FramePacer(double minFrameInterval, double idleInterval = 0.5) : idleInterval_(idleInterval)
    {
        setMinFrameInterval(minFrameInterval);
    }

    void setMinFrameInterval(double seconds) { minFrameInterval_ = seconds > 0.0 ? seconds : 1.0 / 60.0; }

    bool frameDue(double now, bool redrawPending) const { return waitTimeout(now, redrawPending) <= 0.0; }

    // How long the loop may wait for events before the next frame is due.
    double waitTimeout(double now, bool redrawPending) const
    {
        const double interval = redrawPending ? minFrameInterval_ : idleInterval_;
        return std::max(0.0, lastFrame_ + interval - now);
    }

    void frameDrawn(double now) { lastFrame_ = now; }

private:
    double minFrameInterval_ = 1.0 / 60.0;
    double idleInterval_ = 0.5;
    double lastFrame_ = -std::numeric_limits<double>::infinity();
};

} // namespace occlusa::ui
