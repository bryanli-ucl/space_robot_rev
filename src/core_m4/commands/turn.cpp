#include "chassis.hpp"
#include "config.hpp"
#include "fast_line_follower.hpp"
#include "imu.hpp"
#include "line_follower.hpp"
#include "logger.hpp"
#include "mission.hpp"
#include "motion_primitives.hpp"
#include "sensors.hpp"
#include "shell.hpp"
#include "state.hpp"
#include "wall_follower.hpp"

#include <Arduino.h>
#include <math.h>
#include <mbed.h>
#include <stdlib.h>
#include <string.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

static bool parse_float(const char* text, float* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end = nullptr;
    const float value = strtof(text, &end);
    if (end == text || *end != '\0') return false;

    *out = value;
    return true;
}

static bool parse_uint32(const char* text, uint32_t* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end = nullptr;
    const unsigned long value = strtoul(text, &end, 0);
    if (end == text || *end != '\0') return false;

    *out = static_cast<uint32_t>(value);
    return true;
}

static bool parse_turn_degrees(const char* text, float* out) {
    if (text == nullptr || out == nullptr) return false;

    if (strcmp(text, "left") == 0) {
        *out = -90.0f;
        return true;
    }

    if (strcmp(text, "right") == 0) {
        *out = 90.0f;
        return true;
    }

    return parse_float(text, out);
}

static bool parse_ir_turn_direction(const char* text, float* out) {
    if (text == nullptr || out == nullptr) return false;

    if (strcmp(text, "left") == 0) {
        *out = -1.0f;
        return true;
    }

    if (strcmp(text, "right") == 0) {
        *out = 1.0f;
        return true;
    }

    return false;
}

static void turn_ir_cmd(int argc, char** argv) {
    if (argc < 3 || argc > 5) {
        loggf("usage: turn ir <deg|left|right> [timeout_ms] | turn ir <left|right> <w> [timeout_ms]\n");
        return;
    }

    if (running_state == RunningState::STOPPED) {
        loggf("turn ir aborted: robot is stopped. use start first.\n");
        return;
    }

    float direction = 0.0f;
    float search_w = MISSION_LINE_SEARCH_W;
    uint32_t timeout_ms = MISSION_LINE_SEARCH_TIMEOUT_MS;

    if (argc == 3) {
        if (!parse_turn_degrees(argv[2], &direction)) {
            loggf("turn ir deg must be a number, left, or right\n");
            return;
        }
    } else if (parse_ir_turn_direction(argv[2], &direction)) {
        if (!parse_float(argv[3], &search_w)) {
            loggf("turn ir w must be a number\n");
            return;
        }

        if (argc == 5 && !parse_uint32(argv[4], &timeout_ms)) {
            loggf("turn ir timeout_ms must be an integer\n");
            return;
        }
    } else {
        if (!parse_turn_degrees(argv[2], &direction)) {
            loggf("turn ir deg must be a number, left, or right\n");
            return;
        }

        if (!parse_uint32(argv[3], &timeout_ms)) {
            loggf("turn ir timeout_ms must be an integer\n");
            return;
        }
    }

    mission_stop();
    wall_follower_stop();
    line_follower_stop();
    fast_line_follower_stop();

    search_w = constrain(fabsf(search_w), 1.0f, CHASSIS_MAX_WHEEL_SPEED);
    if (motion_turn_to_line_blocking(direction, search_w, timeout_ms)) {
        loggf("turn ir done direction=%.1f\n", direction);
    } else {
        loggf("turn ir failed direction=%.1f\n", direction);
    }
}

static void turn_cmd(int argc, char** argv) {
    if (argc >= 2 && strcmp(argv[1], "ir") == 0) {
        turn_ir_cmd(argc, argv);
        return;
    }

    if (argc != 2) {
        loggf("usage: turn <deg|left|right> | turn ir <deg|left|right> [timeout_ms] | turn ir <left|right> <w> [timeout_ms]\n");
        return;
    }

    if (running_state == RunningState::STOPPED) {
        loggf("turn aborted: robot is stopped. use start first.\n");
        return;
    }

    if (!imu.yaw_is_ready()) {
        loggf("turn aborted: imu yaw is not ready\n");
        return;
    }

    float delta_deg = 0.0f;
    if (!parse_turn_degrees(argv[1], &delta_deg)) {
        loggf("turn deg must be a number, left, or right\n");
        return;
    }

    mission_stop();
    wall_follower_stop();
    line_follower_stop();
    fast_line_follower_stop();

    if (motion_turn_imu_then_line_blocking(delta_deg)) {
        loggf("turn done hybrid delta=%.1f\n", delta_deg);
    } else {
        loggf("turn failed hybrid delta=%.1f\n", delta_deg);
    }
}

SHELL_COMMAND("turn", turn_cmd, "hybrid turn: turn <deg|left|right>; IR only: turn ir <deg|left|right>")
