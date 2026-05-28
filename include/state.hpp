#pragma once

enum class RunningState {
    STOPPED,
    IDLE,
};

extern volatile RunningState running_state;

const char* running_state_name(RunningState state);
void set_running_state(RunningState state);
