#include "bash.hpp"

#include "mbed_stats.h"

static void print_stat() {
    mbed_stats_heap_t heap;
    mbed_stats_heap_get(&heap);

    command_tx("\nHEAP     total       used        peak        free\n");
    command_tx("Heap:    %10lu  %10lu  %10lu  %10lu\n",
    heap.reserved_size,
    heap.current_size,
    heap.max_size,
    heap.reserved_size - heap.current_size);

    command_tx("\nPID          STACK_USED   STACK_TOTAL   USE%%\n");

    mbed_stats_stack_t stack[16];
    size_t count = mbed_stats_stack_get_each(stack, 16);

    for (size_t i = 0; i < count; i++) {
        unsigned long used  = stack[i].max_size;
        unsigned long total = stack[i].reserved_size;
        unsigned long usage = total ? used * 100 / total : 0;

        command_tx("%-12lu %-12lu %-13lu %3lu%%\n",
        (unsigned long)stack[i].thread_id,
        used,
        total,
        usage);
    }

    command_tx("\n");
}

static void print(int argc, char** argv) {

    if (argc != 2) {
        command_tx("print needs exactly 1 arguments.\n");
        return;
    }

    if (strcmp(argv[1], "ir") == 0) {
        // show ir
        command_tx("IR: %d %d %d %d %d %d %d %d %d\n",
        ir_vals[0], ir_vals[1], ir_vals[2],
        ir_vals[3], ir_vals[4], ir_vals[5],
        ir_vals[6], ir_vals[7], ir_vals[8]);

    } else if (strcmp(argv[1], "dist") == 0) {
        // show dist
        command_tx("Dist: Front: %dcm, Left %dcm, Right: %dcm\n",
        dist_front, dist_left, dist_right);

    } else if (strcmp(argv[1], "sunlight") == 0) {
        // show sunlight
        command_tx("Sun Light: %u\n", sun_light);

    } else if (strcmp(argv[1], "orien") == 0) {
        // show orien
        command_tx("orientation: roll: %.2fdeg pitch: %.2fdeg, yaw: %.2f\n",
        ahrs.getRoll(), ahrs.getPitch(), ahrs.getYaw());
    } else if (strcmp(argv[1], "stat") == 0) {
        print_stat();
    } else {
        command_tx("usage: print ir|dist|sunlight|orien|stat\n");
    }
}

BASH_COMMAND("print", print, "print some infos depends on 1st para")
