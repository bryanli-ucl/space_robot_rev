#include "bash.hpp"
#include "motion_control.hpp"

namespace {

bool parse_float_arg(const char* text, float* value) {
    char* end = nullptr;
    float val = strtof(text, &end);

    if (end == text || *end != '\0') {
        return false;
    }

    *value = val;
    return true;
}

bool parse_uid_arg(const char* text, uint32_t* value) {
    char* end = nullptr;
    unsigned long val = strtoul(text, &end, 0);

    if (end == text || *end != '\0') {
        return false;
    }

    *value = static_cast<uint32_t>(val);
    return true;
}

bool parse_wall_side(const char* text, WallSide* side) {
    if (strcmp(text, "r") == 0 || strcmp(text, "right") == 0) {
        *side = WallSide::Right;
        return true;
    }

    if (strcmp(text, "l") == 0 || strcmp(text, "left") == 0) {
        *side = WallSide::Left;
        return true;
    }

    return false;
}

void print_result(const char* name, MotionResult result) {
    command_tx("%s done: %s\n", name, motion_result_name(result));
}

void line_cmd(int argc, char** argv) {
    if (argc < 2) {
        command_tx("usage: line front [front_cm] [speed_cm_s] | line motor <distance_cm> [speed_cm_s] [front_cm] | line rfid [uid|any] [speed_cm_s] [front_cm] | line cross [speed_cm_s] [front_cm]\n");
        return;
    }

    MotionResult result = MotionResult::SensorInvalid;

    if (strcmp(argv[1], "front") == 0) {
        float front_cm = MOTION_DEFAULT_FRONT_STOP_CM;
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        if (argc >= 3 && !parse_float_arg(argv[2], &front_cm)) {
            command_tx("line front front_cm must be a number.\n");
            return;
        }
        if (argc >= 4 && !parse_float_arg(argv[3], &speed)) {
            command_tx("line front speed_cm_s must be a number.\n");
            return;
        }

        result = run_line_follow_until_front_cm(front_cm, speed);
    } else if (strcmp(argv[1], "motor") == 0) {
        if (argc < 3) {
            command_tx("usage: line motor <distance_cm> [speed_cm_s] [front_cm]\n");
            return;
        }

        float distance = 0.0f;
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front_cm = MOTION_DEFAULT_FRONT_STOP_CM;
        if (!parse_float_arg(argv[2], &distance)) {
            command_tx("line motor distance_cm must be a number.\n");
            return;
        }
        if (argc >= 4 && !parse_float_arg(argv[3], &speed)) {
            command_tx("line motor speed_cm_s must be a number.\n");
            return;
        }
        if (argc >= 5 && !parse_float_arg(argv[4], &front_cm)) {
            command_tx("line motor front_cm must be a number.\n");
            return;
        }

        result = run_line_follow_until_motor_cm(distance, speed, front_cm);
    } else if (strcmp(argv[1], "rfid") == 0) {
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front_cm = MOTION_DEFAULT_FRONT_STOP_CM;
        uint32_t uid = 0;
        bool use_uid = false;

        if (argc >= 3 && strcmp(argv[2], "any") != 0) {
            if (parse_uid_arg(argv[2], &uid)) {
                use_uid = true;
            } else if (!parse_float_arg(argv[2], &speed)) {
                command_tx("line rfid argument must be uid, any, or speed_cm_s.\n");
                return;
            }
        }
        if (argc >= 4 && !parse_float_arg(argv[3], &speed)) {
            command_tx("line rfid speed_cm_s must be a number.\n");
            return;
        }
        if (argc >= 5 && !parse_float_arg(argv[4], &front_cm)) {
            command_tx("line rfid front_cm must be a number.\n");
            return;
        }

        result = use_uid ? run_line_follow_until_rfid_uid(uid, speed, front_cm) :
                           run_line_follow_until_rfid(speed, front_cm);
    } else if (strcmp(argv[1], "cross") == 0) {
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front_cm = MOTION_DEFAULT_FRONT_STOP_CM;
        if (argc >= 3 && !parse_float_arg(argv[2], &speed)) {
            command_tx("line cross speed_cm_s must be a number.\n");
            return;
        }
        if (argc >= 4 && !parse_float_arg(argv[3], &front_cm)) {
            command_tx("line cross front_cm must be a number.\n");
            return;
        }

        result = run_line_follow_until_cross(speed, front_cm);
    } else {
        command_tx("usage: line front|motor|rfid|cross ...\n");
        return;
    }

    print_result("line", result);
}

void wall_cmd(int argc, char** argv) {
    if (argc < 4) {
        command_tx("usage: wall front <l|r> <wall_cm> [front_cm] [speed_cm_s] | wall motor <l|r> <wall_cm> <distance_cm> [speed_cm_s] [front_cm] | wall rfid <l|r> <wall_cm> [uid|any] [speed_cm_s] [front_cm]\n");
        return;
    }

    WallSide side = WallSide::Right;
    if (!parse_wall_side(argv[2], &side)) {
        command_tx("wall side must be l/left or r/right.\n");
        return;
    }

    float wall_cm = 0.0f;
    if (!parse_float_arg(argv[3], &wall_cm) || wall_cm <= 0.0f) {
        command_tx("wall_cm must be a positive number.\n");
        return;
    }

    MotionResult result = MotionResult::SensorInvalid;

    if (strcmp(argv[1], "front") == 0) {
        float front_cm = MOTION_DEFAULT_FRONT_STOP_CM;
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        if (argc >= 5 && !parse_float_arg(argv[4], &front_cm)) {
            command_tx("wall front front_cm must be a number.\n");
            return;
        }
        if (argc >= 6 && !parse_float_arg(argv[5], &speed)) {
            command_tx("wall front speed_cm_s must be a number.\n");
            return;
        }

        result = run_wall_follow_until_front_cm(side, wall_cm, front_cm, speed);
    } else if (strcmp(argv[1], "motor") == 0) {
        if (argc < 5) {
            command_tx("usage: wall motor <l|r> <wall_cm> <distance_cm> [speed_cm_s] [front_cm]\n");
            return;
        }

        float distance = 0.0f;
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front_cm = MOTION_DEFAULT_FRONT_STOP_CM;
        if (!parse_float_arg(argv[4], &distance)) {
            command_tx("wall motor distance_cm must be a number.\n");
            return;
        }
        if (argc >= 6 && !parse_float_arg(argv[5], &speed)) {
            command_tx("wall motor speed_cm_s must be a number.\n");
            return;
        }
        if (argc >= 7 && !parse_float_arg(argv[6], &front_cm)) {
            command_tx("wall motor front_cm must be a number.\n");
            return;
        }

        result = run_wall_follow_until_motor_cm(side, wall_cm, distance, speed, front_cm);
    } else if (strcmp(argv[1], "rfid") == 0) {
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front_cm = MOTION_DEFAULT_FRONT_STOP_CM;
        uint32_t uid = 0;
        bool use_uid = false;

        if (argc >= 5 && strcmp(argv[4], "any") != 0) {
            if (parse_uid_arg(argv[4], &uid)) {
                use_uid = true;
            } else if (!parse_float_arg(argv[4], &speed)) {
                command_tx("wall rfid argument must be uid, any, or speed_cm_s.\n");
                return;
            }
        }
        if (argc >= 6 && !parse_float_arg(argv[5], &speed)) {
            command_tx("wall rfid speed_cm_s must be a number.\n");
            return;
        }
        if (argc >= 7 && !parse_float_arg(argv[6], &front_cm)) {
            command_tx("wall rfid front_cm must be a number.\n");
            return;
        }

        result = use_uid ? run_wall_follow_until_rfid_uid(side, wall_cm, uid, speed, front_cm) :
                           run_wall_follow_until_rfid(side, wall_cm, speed, front_cm);
    } else {
        command_tx("usage: wall front|motor|rfid ...\n");
        return;
    }

    print_result("wall", result);
}

void drive_cmd(int argc, char** argv) {
    if (argc < 2) {
        command_tx("usage: drive front [front_cm] [speed_cm_s] | drive motor <distance_cm> [speed_cm_s] [front_cm] | drive rfid [uid|any] [speed_cm_s] [front_cm]\n");
        return;
    }

    MotionResult result = MotionResult::SensorInvalid;

    if (strcmp(argv[1], "front") == 0) {
        float front_cm = MOTION_DEFAULT_FRONT_STOP_CM;
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        if (argc >= 3 && !parse_float_arg(argv[2], &front_cm)) {
            command_tx("drive front front_cm must be a number.\n");
            return;
        }
        if (argc >= 4 && !parse_float_arg(argv[3], &speed)) {
            command_tx("drive front speed_cm_s must be a number.\n");
            return;
        }

        result = run_drive_until_front_cm(front_cm, speed);
    } else if (strcmp(argv[1], "motor") == 0) {
        if (argc < 3) {
            command_tx("usage: drive motor <distance_cm> [speed_cm_s] [front_cm]\n");
            return;
        }

        float distance = 0.0f;
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front_cm = MOTION_DEFAULT_FRONT_STOP_CM;
        if (!parse_float_arg(argv[2], &distance)) {
            command_tx("drive motor distance_cm must be a number.\n");
            return;
        }
        if (argc >= 4 && !parse_float_arg(argv[3], &speed)) {
            command_tx("drive motor speed_cm_s must be a number.\n");
            return;
        }
        if (argc >= 5 && !parse_float_arg(argv[4], &front_cm)) {
            command_tx("drive motor front_cm must be a number.\n");
            return;
        }

        result = run_drive_until_motor_cm(distance, speed, front_cm);
    } else if (strcmp(argv[1], "rfid") == 0) {
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front_cm = MOTION_DEFAULT_FRONT_STOP_CM;
        uint32_t uid = 0;
        bool use_uid = false;

        if (argc >= 3 && strcmp(argv[2], "any") != 0) {
            if (parse_uid_arg(argv[2], &uid)) {
                use_uid = true;
            } else if (!parse_float_arg(argv[2], &speed)) {
                command_tx("drive rfid argument must be uid, any, or speed_cm_s.\n");
                return;
            }
        }
        if (argc >= 4 && !parse_float_arg(argv[3], &speed)) {
            command_tx("drive rfid speed_cm_s must be a number.\n");
            return;
        }
        if (argc >= 5 && !parse_float_arg(argv[4], &front_cm)) {
            command_tx("drive rfid front_cm must be a number.\n");
            return;
        }

        result = use_uid ? run_drive_until_rfid_uid(uid, speed, front_cm) :
                           run_drive_until_rfid(speed, front_cm);
    } else {
        command_tx("usage: drive front|motor|rfid ...\n");
        return;
    }

    print_result("drive", result);
}

} // namespace

BASH_COMMAND("line", line_cmd, "blocking line follow: line front|motor|rfid|cross ...")
BASH_COMMAND("wall", wall_cmd, "blocking wall follow: wall front|motor|rfid ...")
BASH_COMMAND("drive", drive_cmd, "blocking straight drive: drive front|motor|rfid ...")
