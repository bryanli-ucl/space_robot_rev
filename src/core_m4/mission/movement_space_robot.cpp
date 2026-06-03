#include "movement.h"

#include "config.hpp"
#include "line_follower.hpp"
#include "motion_primitives.hpp"
#include "rfid.hpp"
#include "seed_dropper.hpp"
#include "sensors.hpp"
#include "state.hpp"

#include <Arduino.h>
#include <mbed.h>
#include <stdio.h>

using namespace ::std::chrono_literals;
using namespace ::rtos;

static constexpr float CELL_DISTANCE_CM             = TASK4_NODE_CM;
static constexpr float DRIVE_SPEED                  = TASK_DRIVE_SPEED;
static constexpr float LINE_SPEED                   = TASK_LINE_SPEED;
static constexpr float TURN_EXIT_FORWARD_CM         = 5.0f;
static constexpr int16_t FRONT_BLOCKED_CM           = TASK_OBSTACLE_FRONT_CM;
static constexpr uint32_t RFID_TIMEOUT_MS           = 2500;
static constexpr uint32_t RFID_CELL_TIMEOUT_MS      = 12000;

static bool movement_should_stop() {
    return running_state == RunningState::STOPPED || sensors.kill_switch_pressed();
}

static void observe_cell_if_valid(OccupancyMap* occ, int x, int y, bool blocked) {
    if (x < 0 || x >= GRID_W || y < 0 || y >= GRID_H) return;
    occupancy_observe_dynamic(occ, x, y, blocked, false);
}

static bool follow_line_to_next_rfid(uint32_t* detected_uid) {
    motion_stop_all();
    rfid_update();

    line_follower.start_rfid(LINE_SPEED, 0, true, true, LINE_DEFAULT_FRONT_STOP_CM);
    if (!motion_wait_line_done(RFID_CELL_TIMEOUT_MS)) return false;

    const uint32_t uid = rfid.last_uid();
    if (detected_uid != NULL) *detected_uid = uid;
    if (uid == 0) return false;

    return true;
}

void move_forward_one_cell(void) {
    move_forward_rfid_cells(1);
}

bool move_forward_rfid_cells(int cell_count) {
    if (cell_count <= 0) return false;

    for (int i = 0; i < cell_count; i++) {
        uint32_t uid = 0;
        if (!follow_line_to_next_rfid(&uid)) return false;
    }

    return true;
}

void move_backward_one_cell(void) {
    motion_stop_all();
    motion_drive_blocking(-DRIVE_SPEED, CELL_DISTANCE_CM, -1, MISSION_DRIVE_TIMEOUT_MS);
}

void strafe_left_one_cell(void) {
    turn_left_90();
    move_forward_one_cell();
}

void strafe_right_one_cell(void) {
    turn_right_90();
    move_forward_one_cell();
}

void align_to_hole_midpoint(void) {
    motion_stop_all();
    motion_drive_blocking(DRIVE_SPEED * 0.5f, 8.0f, -1, MISSION_DRIVE_TIMEOUT_MS);
}

void drop_seed(void) {
    wait_for_rfid_confirmation();
    motion_stop_all();
    seed_dropper.drop_one();
}

bool wait_for_rfid_confirmation(void) {
    const uint32_t start_ms = millis();
    while (millis() - start_ms < RFID_TIMEOUT_MS) {
        rfid_update();
        if (rfid.last_uid() != 0) return true;
        if (movement_should_stop()) return false;
        ThisThread::sleep_for(20ms);
    }

    return false;
}

bool read_rfid_tag(char* tag_id, size_t tag_id_size) {
    if (tag_id == NULL || tag_id_size == 0) return false;
    tag_id[0] = '\0';

    const uint32_t start_ms = millis();
    while (millis() - start_ms < RFID_TIMEOUT_MS) {
        rfid_update();
        const uint32_t uid = rfid.last_uid();
        if (uid != 0) {
            snprintf(tag_id, tag_id_size, "%08lX", static_cast<unsigned long>(uid));
            return true;
        }

        if (movement_should_stop()) return false;
        ThisThread::sleep_for(20ms);
    }

    return false;
}

void movement_observe_adjacent_cells(OccupancyMap* occ, Cell robot_pos, GridHeading heading) {
    int front_dx = 0;
    int front_dy = 1;

    switch (heading) {
    case HEADING_POS_X:
        front_dx = 1;
        front_dy = 0;
        break;

    case HEADING_NEG_Y:
        front_dx = 0;
        front_dy = -1;
        break;

    case HEADING_NEG_X:
        front_dx = -1;
        front_dy = 0;
        break;

    default:
        break;
    }

    const int left_dx  = -front_dy;
    const int left_dy  = front_dx;
    const int right_dx = front_dy;
    const int right_dy = -front_dx;

    const int16_t front_cm = sensors.ultrasonic_front_cm();
    const int16_t left_cm  = sensors.ultrasonic_left_cm();
    const int16_t right_cm = sensors.ultrasonic_right_cm();

    observe_cell_if_valid(occ, robot_pos.x + front_dx, robot_pos.y + front_dy, front_cm > 0 && front_cm <= FRONT_BLOCKED_CM);
    observe_cell_if_valid(occ, robot_pos.x + left_dx, robot_pos.y + left_dy, left_cm > 0 && left_cm <= FRONT_BLOCKED_CM);
    observe_cell_if_valid(occ, robot_pos.x + right_dx, robot_pos.y + right_dy, right_cm > 0 && right_cm <= FRONT_BLOCKED_CM);
}

void execute_move(MoveDirection dir) {
    switch (dir) {
    case MOVE_FORWARD:
        move_forward_one_cell();
        break;

    case MOVE_BACKWARD:
        move_backward_one_cell();
        break;

    case MOVE_LEFT:
        strafe_left_one_cell();
        break;

    case MOVE_RIGHT:
        strafe_right_one_cell();
        break;

    default:
        motion_stop_all();
        break;
    }
}

void turn_left_90(void) {
    motion_stop_all();
    if (motion_turn_imu_then_line_blocking(TASK2_LEFT_TURN_DEG)) {
        motion_drive_blocking(DRIVE_SPEED * 0.5f, TURN_EXIT_FORWARD_CM, -1, MISSION_DRIVE_TIMEOUT_MS);
    }
}

void turn_right_90(void) {
    motion_stop_all();
    if (motion_turn_imu_then_line_blocking(TASK2_RIGHT_TURN_DEG)) {
        motion_drive_blocking(DRIVE_SPEED * 0.5f, TURN_EXIT_FORWARD_CM, -1, MISSION_DRIVE_TIMEOUT_MS);
    }
}

void movement_stop_all(void) {
    motion_stop_all();
}

bool movement_start_button_pressed(void) {
    return sensors.revive_button_pressed();
}

bool movement_killswitch_pressed(void) {
    return movement_should_stop();
}

void execute_move_to_heading(GridHeading* heading, GridHeading desired_heading) {
    if (heading == NULL) return;

    const int turns = (static_cast<int>(desired_heading) - static_cast<int>(*heading) + 4) % 4;

    if (turns == 1) {
        turn_right_90();
    } else if (turns == 2) {
        turn_right_90();
        turn_right_90();
    } else if (turns == 3) {
        turn_left_90();
    }

    *heading = desired_heading;
    move_forward_one_cell();
}
