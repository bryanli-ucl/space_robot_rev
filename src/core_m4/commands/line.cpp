#include "line_follower.hpp"

#include "config.hpp"
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

static bool parse_int16(const char* text, int16_t* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end = nullptr;
    const long value = strtol(text, &end, 0);
    if (end == text || *end != '\0') return false;

    *out = static_cast<int16_t>(value);
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

static bool ensure_running() {
    if (running_state != RunningState::STOPPED) return true;

    loggf("line aborted: robot is stopped. use start first.\n");
    return false;
}

static bool parse_speed(const char* text, float* speed) {
    if (!parse_float(text, speed)) {
        loggf("line speed must be a number\n");
        return false;
    }

    if (*speed <= 0.0f) {
        line_follower.stop();
        return false;
    }

    return true;
}

static bool parse_optional_front(int argc, char** argv, int index, int16_t* front_cm) {
    *front_cm = LINE_DEFAULT_FRONT_STOP_CM;
    if (argc <= index) return true;

    if (!parse_int16(argv[index], front_cm)) {
        loggf("line front_cm must be an integer\n");
        return false;
    }

    return true;
}

static void line_cmd(int argc, char** argv) {
    if (argc == 1 || (argc == 2 && strcmp(argv[1], "status") == 0)) {
        line_follower.print_status();
        return;
    }

    if (argc == 2 && strcmp(argv[1], "stop") == 0) {
        mission_stop();
        line_follower.stop();
        return;
    }

    if (argc >= 2 && strcmp(argv[1], "cross") == 0) {
        if (argc < 3 || argc > 4) {
            loggf("usage: line cross <speed> [front_cm]\n");
            return;
        }

        if (!ensure_running()) return;

        float speed = 0.0f;
        int16_t front_cm = -1;
        if (!parse_speed(argv[2], &speed) || !parse_optional_front(argc, argv, 3, &front_cm)) return;

        mission_stop();
        wall_follower_stop();
        line_follower.start(speed, LineFollower::StopMode::Cross, front_cm);
        return;
    }

    if (argc >= 2 && strcmp(argv[1], "front") == 0) {
        if (argc != 4) {
            loggf("usage: line front <speed> <front_cm>\n");
            return;
        }

        if (!ensure_running()) return;

        float speed = 0.0f;
        int16_t front_cm = -1;
        if (!parse_speed(argv[2], &speed) || !parse_int16(argv[3], &front_cm)) {
            loggf("line front args must be numbers\n");
            return;
        }

        mission_stop();
        wall_follower_stop();
        line_follower.start(speed, LineFollower::StopMode::Front, front_cm);
        return;
    }

    if (argc >= 2 && (strcmp(argv[1], "dist") == 0 || strcmp(argv[1], "distance") == 0)) {
        if (argc < 4 || argc > 5) {
            loggf("usage: line dist <speed> <cm> [front_cm]\n");
            return;
        }

        if (!ensure_running()) return;

        float speed = 0.0f;
        float distance_cm = 0.0f;
        int16_t front_cm = -1;
        if (!parse_speed(argv[2], &speed) || !parse_float(argv[3], &distance_cm) || distance_cm <= 0.0f ||
            !parse_optional_front(argc, argv, 4, &front_cm)) {
            loggf("line dist args must be numbers\n");
            return;
        }

        mission_stop();
        wall_follower_stop();
        line_follower.start(speed, LineFollower::StopMode::Distance, front_cm, distance_cm);
        return;
    }

    if (argc >= 2 && strcmp(argv[1], "rfid") == 0) {
        if (argc < 4 || argc > 5) {
            loggf("usage: line rfid <speed> <any|notsame|uid> [front_cm]\n");
            return;
        }

        if (!ensure_running()) return;

        float speed = 0.0f;
        int16_t front_cm = -1;
        uint32_t uid = 0;
        bool any_uid = false;
        bool not_same = false;

        if (!parse_speed(argv[2], &speed) || !parse_optional_front(argc, argv, 4, &front_cm)) return;

        if (strcmp(argv[3], "any") == 0) {
            any_uid = true;
        } else if (strcmp(argv[3], "notsame") == 0) {
            not_same = true;
            any_uid = true;
        } else if (!parse_uint32(argv[3], &uid) || uid == 0) {
            loggf("line rfid target must be any, notsame, or uid\n");
            return;
        }

        mission_stop();
        wall_follower_stop();
        line_follower.start_rfid(speed, uid, any_uid, not_same, front_cm);
        return;
    }

    if (argc >= 2 && strcmp(argv[1], "corner") == 0) {
        if (argc < 4 || argc > 5) {
            loggf("usage: line corner <left|right|any> <speed> [front_cm]\n");
            return;
        }

        if (!ensure_running()) return;

        LineFollower::StopMode mode = LineFollower::StopMode::AnyCorner;
        if (strcmp(argv[2], "left") == 0) {
            mode = LineFollower::StopMode::LeftCorner;
        } else if (strcmp(argv[2], "right") == 0) {
            mode = LineFollower::StopMode::RightCorner;
        } else if (strcmp(argv[2], "any") == 0) {
            mode = LineFollower::StopMode::AnyCorner;
        } else {
            loggf("line corner side must be left, right, or any\n");
            return;
        }

        float speed = 0.0f;
        int16_t front_cm = -1;
        if (!parse_speed(argv[3], &speed) || !parse_optional_front(argc, argv, 4, &front_cm)) return;

        mission_stop();
        wall_follower_stop();
        line_follower.start(speed, mode, front_cm);
        return;
    }

    if (argc != 2) {
        loggf("usage: line <speed>|stop|status|cross <speed> [front_cm]|front <speed> <front_cm>|dist <speed> <cm> [front_cm]|rfid <speed> <any|notsame|uid> [front_cm]|corner <left|right|any> <speed> [front_cm]\n");
        return;
    }

    if (!ensure_running()) return;

    float speed = 0.0f;
    if (!parse_speed(argv[1], &speed)) return;

    mission_stop();
    wall_follower_stop();
    line_follower.start(speed);
}

SHELL_COMMAND("line", line_cmd, "line follow: line <speed>|cross|front|dist|rfid|corner|stop|status")
