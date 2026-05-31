#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "common.h"
#include "occupancy.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void move_forward_one_cell(void);
bool move_forward_rfid_cells(int cell_count);
void move_backward_one_cell(void);
void strafe_left_one_cell(void);
void strafe_right_one_cell(void);

void align_to_hole_midpoint(void);

void drop_seed(void);

bool wait_for_rfid_confirmation(void);

bool read_rfid_tag(
    char* tag_id,
    size_t tag_id_size
);

void movement_observe_adjacent_cells(
    OccupancyMap* occ,
    Cell robot_pos,
    GridHeading heading
);

void execute_move(MoveDirection dir);

void turn_left_90(void);
void turn_right_90(void);

void movement_stop_all(void);

bool movement_start_button_pressed(void);

bool movement_killswitch_pressed(void);

void execute_move_to_heading(
    GridHeading* heading,
    GridHeading desired_heading
);

#ifdef __cplusplus
}
#endif

#endif
