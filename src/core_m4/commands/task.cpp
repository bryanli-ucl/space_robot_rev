#include "task_controller.hpp"

#include "logger.hpp"
#include "mission.hpp"
#include "shell.hpp"

#include <stdlib.h>
#include <string.h>

static bool parse_task_id(const char* text, uint8_t* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end = nullptr;
    const long value = strtol(text, &end, 10);
    if (end == text || *end != '\0' || value < 1 || value > 8) return false;

    *out = static_cast<uint8_t>(value);
    return true;
}

static void print_task_help() {
    loggf("usage: task <1..8> [start] | task stop | task status\n");
    loggf("task 1=line, 2=intersection+rfid, 3=solid-grid, 4=open-field drive, 5=ramp drive, 6=wall, 7=obstacle avoid, 8=revive approach\n");
}

static void task_cmd(int argc, char** argv) {
    if (argc == 1 || (argc == 2 && strcmp(argv[1], "status") == 0)) {
        mission_print_status();
        task_controller.print_status();
        return;
    }

    if (argc == 2 && strcmp(argv[1], "stop") == 0) {
        mission_stop();
        task_controller.stop();
        return;
    }

    uint8_t task_id = 0;
    if (argc < 2 || argc > 3 || !parse_task_id(argv[1], &task_id)) {
        print_task_help();
        return;
    }

    if (argc == 3 && strcmp(argv[2], "start") != 0) {
        print_task_help();
        return;
    }

    if (task_id == 2 || task_id == 3 || task_id == 7 || task_id == 8) {
        mission_start_task(task_id);
    } else {
        mission_stop();
        task_controller.start(task_id);
    }
}

SHELL_COMMAND("task", task_cmd, "trial task control: task <1..8> [start], task status, task stop")
