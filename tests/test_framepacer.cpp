#include "ui/FramePacer.h"

#include <doctest.h>

using occlusa::ui::FramePacer;

TEST_CASE("FramePacer draws the first frame immediately")
{
    const FramePacer pacer(1.0 / 60.0, 0.5);
    CHECK(pacer.frameDue(0.0, false));
    CHECK(pacer.waitTimeout(0.0, false) == doctest::Approx(0.0));
}

TEST_CASE("FramePacer limits pending redraws to the frame rate")
{
    FramePacer pacer(0.02, 0.5);
    pacer.frameDrawn(10.0);
    CHECK_FALSE(pacer.frameDue(10.005, true));
    CHECK(pacer.waitTimeout(10.005, true) == doctest::Approx(0.015));
    CHECK(pacer.frameDue(10.02, true));
    CHECK(pacer.waitTimeout(10.03, true) == doctest::Approx(0.0));
}

TEST_CASE("FramePacer ignores early wakeups without a redraw")
{
    // A wakeup without activity (e.g. a Wayland delete_id after each frame) must not draw a
    // frame; otherwise the loop redraws continuously.
    FramePacer pacer(0.02, 0.5);
    pacer.frameDrawn(10.0);
    CHECK_FALSE(pacer.frameDue(10.03, false));
    CHECK_FALSE(pacer.frameDue(10.49, false));
    CHECK(pacer.waitTimeout(10.1, false) == doctest::Approx(0.4));
    CHECK(pacer.frameDue(10.5, false));
}

TEST_CASE("FramePacer uses a fallback rate for an invalid interval")
{
    FramePacer pacer(0.0, 0.5);
    pacer.frameDrawn(1.0);
    CHECK_FALSE(pacer.frameDue(1.001, true));
    CHECK(pacer.frameDue(1.0 + 1.0 / 60.0, true));
}
