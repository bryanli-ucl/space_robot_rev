#include "fast_line_follower.hpp"

#include "line_follower.hpp"
#include "logger.hpp"
#include "mission.hpp"
#include "shell.hpp"
#include "state.hpp"
#include "wall_follower.hpp"

#include <stdlib.h>
#include <string.h>

static bool parse_float(const char* text, float* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end = nullptr;
    const float value = strtof(text, &end);
    if (end == text || *end != '\0') return false;

    *out = value;
    return true;
}

static bool ensure_running() {
    if (running_state != RunningState::STOPPED) return true;

    loggf("linef aborted: robot is stopped. use start first.\n");
    return false;
}

static void prepare_linef_start() {
    mission_stop();
    wall_follower_stop();
    line_follower_stop();
}

static void set_linef_speed(float speed) {
    if (speed <= 0.0f) {
        fast_line_follower.stop();
        return;
    }

    if (!ensure_running()) return;

    prepare_linef_start();
    fast_line_follower.set_speed(speed);
    fast_line_follower.start();
}

static void linef_cmd(int argc, char** argv) {
    if (argc == 1 || (argc == 2 && strcmp(argv[1], "status") == 0)) {
        fast_line_follower.print_status();
        return;
    }

    if (argc == 2 && strcmp(argv[1], "stop") == 0) {
        mission_stop();
        fast_line_follower.stop();
        return;
    }

    if (argc >= 2 && strcmp(argv[1], "pid") == 0) {
        if (argc == 2) {
            loggf("linef pid=%.3f/%.3f/%.3f\n",
            fast_line_follower.kp(),
            fast_line_follower.ki(),
            fast_line_follower.kd());
            return;
        }

        if (argc != 5) {
            loggf("usage: linef pid <kp> <ki> <kd>\n");
            return;
        }

        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        if (!parse_float(argv[2], &kp) || !parse_float(argv[3], &ki) || !parse_float(argv[4], &kd)) {
            loggf("linef pid args must be numbers\n");
            return;
        }

        fast_line_follower.set_pid(kp, ki, kd);
        loggf("linef pid=%.3f/%.3f/%.3f\n",
        fast_line_follower.kp(),
        fast_line_follower.ki(),
        fast_line_follower.kd());
        return;
    }

    if (argc >= 2 && strcmp(argv[1], "speed") == 0) {
        if (argc == 2) {
            loggf("linef speed=%.1f active=%d\n",
            fast_line_follower.target_speed(),
            fast_line_follower.is_active() ? 1 : 0);
            return;
        }

        if (argc != 3) {
            loggf("usage: linef speed <speed>\n");
            return;
        }

        float speed = 0.0f;
        if (!parse_float(argv[2], &speed)) {
            loggf("linef speed must be a number\n");
            return;
        }

        set_linef_speed(speed);
        return;
    }

    if (argc >= 2 && strcmp(argv[1], "rot") == 0) {
        if (argc == 2) {
            loggf("linef rot=%.0f%%\n", fast_line_follower.recovery_speed_scale() * 100.0f);
            return;
        }

        if (argc != 3) {
            loggf("usage: linef rot <percent>\n");
            return;
        }

        float percent = 0.0f;
        if (!parse_float(argv[2], &percent)) {
            loggf("linef rot percent must be a number\n");
            return;
        }

        fast_line_follower.set_recovery_speed_scale(percent * 0.01f);
        loggf("linef rot=%.0f%%\n", fast_line_follower.recovery_speed_scale() * 100.0f);
        return;
    }

    if (argc == 2) {
        float speed = 0.0f;
        if (parse_float(argv[1], &speed)) {
            set_linef_speed(speed);
            return;
        }
    }

    loggf("usage: linef [status]|stop|speed <speed>|rot <percent>|pid <kp> <ki> <kd>\n");
}

SHELL_COMMAND("linef", linef_cmd, "fast line follow: linef speed <speed>, linef rot <percent>, linef pid <kp> <ki> <kd>, linef stop/status")
