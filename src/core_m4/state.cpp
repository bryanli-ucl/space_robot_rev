#include "core_m4/state.hpp"

volatile RunningState running_state = RunningState::STOPPED;
volatile MotionState motion_state   = MotionState::IDLE;

const char* running_state_name(RunningState state) {
    switch (state) {
    case RunningState::STOPPED: return "stopped";
    case RunningState::REVIVING: return "reviving";
    case RunningState::IDLE: return "idle";
    }

    return "unknown";
}

const char* motion_state_name(MotionState state) {
    switch (state) {
    case MotionState::IDLE: return "idle";
    case MotionState::LINE_FOLLOW: return "line-follow";
    case MotionState::WALL_FOLLOW: return "wall-follow";
    case MotionState::MOUSE_FOLLOW: return "mouse-follow";
    }

    return "unknown";
}

