#include "chassis.hpp"
#include "logger.hpp"
#include "shell.hpp"
#include "state.hpp"

static void start_cmd(int argc, char** argv) {
    (void)argc;
    (void)argv;

    set_running_state(RunningState::IDLE);
    loggf("state=%s\n", running_state_name(running_state));
}

static void stop_cmd(int argc, char** argv) {
    (void)argc;
    (void)argv;

    chassis_stop();
    set_running_state(RunningState::STOPPED);
    loggf("state=%s\n", running_state_name(running_state));
}

static void state_cmd(int argc, char** argv) {
    (void)argc;
    (void)argv;

    loggf("state=%s\n", running_state_name(running_state));
}

SHELL_COMMAND("start", start_cmd, "enter idle state")
SHELL_COMMAND("stop", stop_cmd, "stop all motors and enter stopped state")
SHELL_COMMAND("state", state_cmd, "print running state")
