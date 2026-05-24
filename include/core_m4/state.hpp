#pragma once

enum class RunningState {
    STOPPED,
    REVIVING,
    IDLE,
};

enum class MotionState {
    IDLE,
    LINE_FOLLOW,
    WALL_FOLLOW,
    MOUSE_FOLLOW,
};

extern volatile RunningState running_state;
extern volatile MotionState motion_state;

const char* running_state_name(RunningState state);
const char* motion_state_name(MotionState state);

