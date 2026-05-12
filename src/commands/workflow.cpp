#include "bash.hpp"

static void start(int argc, char** argv) {
    // Start
    if (button_state == ButtonState::STOPPED) {
        button_state = ButtonState::IDLE;
    }
}
BASH_COMMAND("start", start, "start up robot")

static void stop(int argc, char** argv) {
    // Stop
    if (button_state != ButtonState::STOPPED) {
        button_state = ButtonState::STOPPED;
        motion_state = MotionState::IDLE;
    }
}

BASH_COMMAND("stop", stop, "shutdown robot immediatly")
