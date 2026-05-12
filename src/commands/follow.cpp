#include "bash.hpp"

static void follow(int argc, char** argv) {

    if (argc != 2) {
        wifi_tx("follow needs exactly 1 arguments.\n");
        return;
    }

    if (strcmp(argv[1], "line") == 0) {
        // follow line
        motion_state = MotionState::LINE_FOLLOW;
    } else if (strcmp(argv[1], "wall") == 0) {
        // follow wall
        motion_state = MotionState::WALL_FOLLOW;
    } else if (strcmp(argv[1], "mouse") == 0) {
        // follow mouse
        motion_state = MotionState::MOUSE_FOLLOW;
    }
}

BASH_COMMAND("follow", follow, "follow line/wall/mouse")
