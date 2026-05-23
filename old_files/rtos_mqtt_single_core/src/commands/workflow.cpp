#include "bash.hpp"
#include "motion_control.hpp"

static void start(int argc, char** argv) {
    // Start
    if (running_state == RunningState::STOPPED) {
        running_state = RunningState::IDLE;
        chassis.enable();
    }
}
BASH_COMMAND("start", start, "start up robot")

static void stop(int argc, char** argv) {
    // Stop
    if (running_state != RunningState::STOPPED) {
        running_state = RunningState::STOPPED;
        motion_state  = MotionState::IDLE;
        motion_force_stop(true);
    }
}

BASH_COMMAND("stop", stop, "shutdown robot immediatly")
