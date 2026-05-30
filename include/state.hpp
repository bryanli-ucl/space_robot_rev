#pragma once

enum class RunningState {
    STOPPED,
    IDLE,
};

extern volatile RunningState running_state;

const char* running_state_name(RunningState state);
void set_running_state(RunningState state);
void state_outputs_begin();
void state_update_outputs();
void state_enter_idle(const char* reason);
void state_force_stop(const char* reason);
