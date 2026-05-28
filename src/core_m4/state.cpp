#include "state.hpp"

volatile RunningState running_state = RunningState::STOPPED;

const char* running_state_name(RunningState state) {
    switch (state) {
    case RunningState::STOPPED: return "stopped";
    case RunningState::IDLE: return "idle";
    }

    return "unknown";
}

void set_running_state(RunningState state) {
    running_state = state;
}
