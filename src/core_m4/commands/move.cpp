#include "chassis.hpp"
#include "fast_line_follower.hpp"
#include "line_follower.hpp"
#include "logger.hpp"
#include "mission.hpp"
#include "motor.hpp"
#include "shell.hpp"
#include "state.hpp"
#include "wall_follower.hpp"

#include <mbed.h>
#include <stdlib.h>
#include <string.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

static bool parse_float(const char* text, float* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end         = nullptr;
    const float value = strtof(text, &end);
    if (end == text || *end != '\0') return false;

    *out = value;
    return true;
}

static bool parse_uint32(const char* text, uint32_t* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end                 = nullptr;
    const unsigned long value = strtoul(text, &end, 0);
    if (end == text || *end != '\0') return false;

    *out = static_cast<uint32_t>(value);
    return true;
}

static void print_chassis() {
    loggf("chassis target=%.1f/%.1f/%.1f wheel=%.1f/%.1f/%.1f/%.1f speed=%.1f/%.1f/%.1f/%.1f raw=%.1f/%.1f/%.1f/%.1f pwm=%d/%d/%d/%d\n",
    chassis.target_vx(),
    chassis.target_vy(),
    chassis.target_w(),
    chassis.wheel_fl(),
    chassis.wheel_fr(),
    chassis.wheel_rl(),
    chassis.wheel_rr(),
    motor_fl().current_speed(),
    motor_fr().current_speed(),
    motor_rl().current_speed(),
    motor_rr().current_speed(),
    motor_fl().raw_speed(),
    motor_fr().raw_speed(),
    motor_rl().raw_speed(),
    motor_rr().raw_speed(),
    motor_fl().applied_pwm(),
    motor_fr().applied_pwm(),
    motor_rl().applied_pwm(),
    motor_rr().applied_pwm());
}

static void move_cmd(int argc, char** argv) {
    if (argc == 1 || (argc == 2 && strcmp(argv[1], "status") == 0)) {
        print_chassis();
        return;
    }

    if (argc == 2 && strcmp(argv[1], "stop") == 0) {
        mission_stop();
        wall_follower_stop();
        line_follower_stop();
    fast_line_follower_stop();
        chassis_stop();
        print_chassis();
        return;
    }

    if (argc < 4 || argc > 5) {
        loggf("usage: move <vx> <vy> <w> [duration_ms] | move stop | move status\n");
        return;
    }

    if (running_state == RunningState::STOPPED) {
        loggf("move aborted: robot is stopped. use start first.\n");
        return;
    }

    float vx = 0.0f;
    float vy = 0.0f;
    float w  = 0.0f;
    if (!parse_float(argv[1], &vx) || !parse_float(argv[2], &vy) || !parse_float(argv[3], &w)) {
        loggf("move vx vy w must be numbers\n");
        return;
    }

    uint32_t duration_ms = 0;
    if (argc == 5 && !parse_uint32(argv[4], &duration_ms)) {
        loggf("move duration_ms must be an integer\n");
        return;
    }

    mission_stop();
    wall_follower_stop();
    line_follower_stop();
    fast_line_follower_stop();
    chassis.set_target(vx, vy, w);
    print_chassis();

    if (duration_ms == 0) return;

    const uint32_t start_ms = millis();
    while (millis() - start_ms < duration_ms) {
        if (running_state == RunningState::STOPPED) {
            chassis_stop();
            loggf("move stopped by state\n");
            return;
        }
        ThisThread::sleep_for(20ms);
    }

    chassis_stop();
    loggf("move done\n");
}

SHELL_COMMAND("move", move_cmd, "open-loop chassis move: move <vx> <vy> <w> [duration_ms]")
