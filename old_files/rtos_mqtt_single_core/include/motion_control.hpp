#pragma once

#include "main.hpp"

enum class MotionResult {
    DistanceReached,
    FrontObstacle,
    RfidDetected,
    CrossLineDetected,
    AngleReached,
    StopButton,
    SensorInvalid,
    Timeout,
};

enum class WallSide {
    Left,
    Right,
};

struct RfidStop {
    bool enabled;
    bool any;
    uint32_t uid;

    static RfidStop none() { return { false, true, 0 }; }
    static RfidStop any_uid() { return { true, true, 0 }; }
    static RfidStop uid_match(uint32_t uid) { return { true, false, uid }; }
};

constexpr float MOTION_DEFAULT_SPEED_CM_S = 10.0f;
constexpr float MOTION_DEFAULT_FRONT_STOP_CM = 10.0f;

const char* motion_result_name(MotionResult result);
void motion_force_stop(bool disable_chassis = true);

MotionResult run_line_follow_until_front_cm(
float front_stop_cm = MOTION_DEFAULT_FRONT_STOP_CM,
float speed_cm_s = MOTION_DEFAULT_SPEED_CM_S);

MotionResult run_line_follow_until_motor_cm(
float distance_cm,
float speed_cm_s = MOTION_DEFAULT_SPEED_CM_S,
float front_stop_cm = MOTION_DEFAULT_FRONT_STOP_CM);

MotionResult run_line_follow_until_rfid(
float speed_cm_s = MOTION_DEFAULT_SPEED_CM_S,
float front_stop_cm = MOTION_DEFAULT_FRONT_STOP_CM);

MotionResult run_line_follow_until_rfid_uid(
uint32_t uid,
float speed_cm_s = MOTION_DEFAULT_SPEED_CM_S,
float front_stop_cm = MOTION_DEFAULT_FRONT_STOP_CM);

MotionResult run_line_follow_until_cross(
float speed_cm_s = MOTION_DEFAULT_SPEED_CM_S,
float front_stop_cm = MOTION_DEFAULT_FRONT_STOP_CM);

MotionResult run_wall_follow_until_front_cm(
WallSide side,
float wall_dist_cm,
float front_stop_cm = MOTION_DEFAULT_FRONT_STOP_CM,
float speed_cm_s = MOTION_DEFAULT_SPEED_CM_S);

MotionResult run_wall_follow_until_motor_cm(
WallSide side,
float wall_dist_cm,
float distance_cm,
float speed_cm_s = MOTION_DEFAULT_SPEED_CM_S,
float front_stop_cm = MOTION_DEFAULT_FRONT_STOP_CM);

MotionResult run_wall_follow_until_rfid(
WallSide side,
float wall_dist_cm,
float speed_cm_s = MOTION_DEFAULT_SPEED_CM_S,
float front_stop_cm = MOTION_DEFAULT_FRONT_STOP_CM);

MotionResult run_wall_follow_until_rfid_uid(
WallSide side,
float wall_dist_cm,
uint32_t uid,
float speed_cm_s = MOTION_DEFAULT_SPEED_CM_S,
float front_stop_cm = MOTION_DEFAULT_FRONT_STOP_CM);

MotionResult run_drive_until_front_cm(
float front_stop_cm = MOTION_DEFAULT_FRONT_STOP_CM,
float speed_cm_s = MOTION_DEFAULT_SPEED_CM_S);

MotionResult run_drive_until_motor_cm(
float distance_cm,
float speed_cm_s = MOTION_DEFAULT_SPEED_CM_S,
float front_stop_cm = MOTION_DEFAULT_FRONT_STOP_CM);

MotionResult run_drive_until_rfid(
float speed_cm_s = MOTION_DEFAULT_SPEED_CM_S,
float front_stop_cm = MOTION_DEFAULT_FRONT_STOP_CM);

MotionResult run_drive_until_rfid_uid(
uint32_t uid,
float speed_cm_s = MOTION_DEFAULT_SPEED_CM_S,
float front_stop_cm = MOTION_DEFAULT_FRONT_STOP_CM);

MotionResult run_turn_deg(
float delta_deg,
float max_w = CONFIG::TURN_MAX_W,
float tolerance_deg = CONFIG::TURN_TOLERANCE_DEG,
uint32_t timeout_ms = CONFIG::TURN_TIMEOUT_MS);
