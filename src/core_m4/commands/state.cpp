#include "state.hpp"
#include "chassis.hpp"
#include "config.hpp"
#include "line_follower.hpp"
#include "logger.hpp"
#include "mission.hpp"
#include "sensors.hpp"
#include "shell.hpp"
#include "wall_follower.hpp"

#include <Arduino.h>

static bool state_outputs_ready = false;
static bool kill_stop_reported  = false;

static void set_red_led(bool on) {
    digitalWrite(STATE_LED_RED_PIN, on ? HIGH : LOW);
}

static void set_green_led(bool on) {
    digitalWrite(STATE_LED_GREEN_PIN, on ? HIGH : LOW);
}

void state_outputs_begin() {
    pinMode(STATE_LED_RED_PIN, OUTPUT);
    pinMode(STATE_LED_GREEN_PIN, OUTPUT);
    set_red_led(false);
    set_green_led(false);
    state_outputs_ready = true;
    state_update_outputs();
}

void state_update_outputs() {
    if (!state_outputs_ready) state_outputs_begin();

    if (sensors.revive_button_pressed()) {
        set_red_led(false);
        set_green_led(true);
        return;
    }

    if (running_state == RunningState::STOPPED) {
        set_red_led((millis() / STATE_STOPPED_BLINK_MS) & 1);
        set_green_led(false);
    } else if (running_state == RunningState::IDLE) {
        set_red_led(true);
        set_green_led(false);
    }
}

void state_force_stop(const char* reason) {
    mission_stop();
    wall_follower_stop();
    line_follower_stop();
    chassis_stop();
    set_running_state(RunningState::STOPPED);

    if (!kill_stop_reported) {
        loggf("state=stopped reason=%s\n", reason == nullptr ? "unknown" : reason);
        kill_stop_reported = true;
    }
}

void state_enter_idle(const char* reason) {
    set_running_state(RunningState::IDLE);
    kill_stop_reported = false;
    state_update_outputs();
    loggf("state=idle reason=%s\n", reason == nullptr ? "unknown" : reason);
}

static void start_cmd(int argc, char** argv) {
    (void)argc;
    (void)argv;

    state_enter_idle("shell start");
}

static void stop_cmd(int argc, char** argv) {
    (void)argc;
    (void)argv;

    state_force_stop("shell stop");
    state_update_outputs();
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
