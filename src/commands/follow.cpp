#include "bash.hpp"

static bool parse_float_arg(const char* text, float* value) {
    char* end = nullptr;
    float val = strtof(text, &end);

    if (end == text || *end != '\0') {
        return false;
    }

    *value = val;
    return true;
}

static void follow(int argc, char** argv) {

    if (argc < 2) {
        command_tx("usage: follow line|mouse OR follow wall <r|l> <dist_cm>\n");
        return;
    }

    if (argc == 2 && strcmp(argv[1], "line") == 0) {
        // follow line
        motion_state = MotionState::LINE_FOLLOW;
    } else if (argc == 4 && strcmp(argv[1], "wall") == 0) {
        // follow wall
        if (strcmp(argv[2], "r") == 0 || strcmp(argv[2], "right") == 0) {
            wall_follow_side = 1;
        } else if (strcmp(argv[2], "l") == 0 || strcmp(argv[2], "left") == 0) {
            wall_follow_side = -1;
        } else {
            command_tx("follow wall side must be r/right or l/left.\n");
            return;
        }

        float target_cm = 0.0f;
        if (!parse_float_arg(argv[3], &target_cm) || target_cm <= 0.0f) {
            command_tx("follow wall dist_cm must be a positive number.\n");
            return;
        }

        wall_follow_target_cm = target_cm;
        motion_state = MotionState::WALL_FOLLOW;
        command_tx("follow wall %s %.1fcm\n", wall_follow_side > 0 ? "right" : "left", target_cm);
    } else if (argc == 2 && strcmp(argv[1], "mouse") == 0) {
        // follow mouse
        motion_state = MotionState::MOUSE_FOLLOW;
    } else {
        command_tx("usage: follow line|mouse OR follow wall <r|l> <dist_cm>\n");
    }
}

BASH_COMMAND("follow", follow, "follow line|mouse OR follow wall <r|l> <dist_cm>")
