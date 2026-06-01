#include "wall_follower.hpp"

#include "config.hpp"
#include "imu.hpp"
#include "line_follower.hpp"
#include "logger.hpp"
#include "mission.hpp"
#include "sensors.hpp"
#include "shell.hpp"
#include "state.hpp"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static bool parse_float(const char* text, float* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end         = nullptr;
    const float value = strtof(text, &end);
    if (end == text || *end != '\0') return false;

    *out = value;
    return true;
}

static bool parse_int16(const char* text, int16_t* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end        = nullptr;
    const long value = strtol(text, &end, 0);
    if (end == text || *end != '\0') return false;

    *out = static_cast<int16_t>(value);
    return true;
}

static bool ensure_running() {
    if (running_state != RunningState::STOPPED) return true;

    loggf("wall aborted: robot is stopped. use start first.\n");
    return false;
}

static void wall_cmd(int argc, char** argv) {
    if (argc == 1 || (argc == 2 && strcmp(argv[1], "status") == 0)) {
        wall_follower.print_status();
        return;
    }

    if (argc == 2 && strcmp(argv[1], "stop") == 0) {
        mission_stop();
        wall_follower.stop();
        return;
    }

    if (argc >= 2 && strcmp(argv[1], "pid") == 0) {
        if (argc == 2) {
            loggf("wall pid dist=%.3f yaw=%.3f max_w=%.1f strategy: w=TURN_DIRECTION*(yaw_kp*yaw_error + side_direction*dist_kp*wall_error), then clamp to max_w\n",
            wall_follower.distance_kp(),
            wall_follower.yaw_kp(),
            wall_follower.max_w());
            return;
        }

        if (argc != 4 && argc != 5) {
            loggf("usage: wall pid <dist_kp> <yaw_kp> [max_w]\n");
            return;
        }

        float dist_kp = 0.0f;
        float yaw_kp  = 0.0f;
        float max_w   = wall_follower.max_w();
        if (!parse_float(argv[2], &dist_kp) || !parse_float(argv[3], &yaw_kp)) {
            loggf("wall pid dist_kp and yaw_kp must be numbers\n");
            return;
        }

        if (argc == 5 && !parse_float(argv[4], &max_w)) {
            loggf("wall pid max_w must be a number\n");
            return;
        }

        wall_follower.set_pid(dist_kp, yaw_kp, max_w);
        loggf("wall pid dist=%.3f yaw=%.3f max_w=%.1f\n",
        wall_follower.distance_kp(),
        wall_follower.yaw_kp(),
        wall_follower.max_w());
        return;
    }

    if (argc < 4 || argc > 5 || (strcmp(argv[1], "left") != 0 && strcmp(argv[1], "right") != 0)) {
        loggf("usage: wall left|right <speed> <dist_cm> [target_cm|auto] | wall pid [dist_kp yaw_kp max_w] | wall stop | wall status\n");
        return;
    }

    if (!ensure_running()) return;

    if (!imu.yaw_is_ready()) {
        loggf("wall aborted: imu yaw is not ready\n");
        return;
    }

    float speed = 0.0f;
    float distance_cm = 0.0f;
    int16_t target_cm = WALL_DEFAULT_TARGET_CM;
    if (!parse_float(argv[2], &speed) || !parse_float(argv[3], &distance_cm) || speed <= 0.0f || distance_cm <= 0.0f) {
        loggf("wall speed and dist_cm must be positive numbers\n");
        return;
    }

    const auto side = strcmp(argv[1], "left") == 0 ? WallFollower::Side::Left : WallFollower::Side::Right;

    if (argc == 5) {
        if (strcmp(argv[4], "auto") == 0) {
            target_cm = WALL_DEFAULT_TARGET_CM;
        } else if (!parse_int16(argv[4], &target_cm) || target_cm <= 0) {
            loggf("wall target_cm must be a positive integer or auto\n");
            return;
        }
    }

    if (target_cm <= 0) {
        target_cm = side == WallFollower::Side::Left ? sensors.ultrasonic_left_cm() : sensors.ultrasonic_right_cm();
        if (target_cm <= 0) {
            loggf("wall auto target failed: no valid %s wall distance\n", side == WallFollower::Side::Left ? "left" : "right");
            return;
        }
    }

    mission_stop();
    line_follower_stop();
    wall_follower.start(side, fabsf(speed), distance_cm, target_cm);
}

SHELL_COMMAND("wall", wall_cmd, "wall follow: wall left|right <speed> <dist_cm> [target_cm|auto], wall pid")
