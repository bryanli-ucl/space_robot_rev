#include "mission.hpp"

#include "chassis.hpp"
#include "config.hpp"
#include "imu.hpp"
#include "line_follower.hpp"
#include "logger.hpp"
#include "motor.hpp"
#include "rfid.hpp"
#include "sensors.hpp"
#include "state.hpp"
#include "wall_follower.hpp"
#include "mission/termthree_navigation.h"

#include <Arduino.h>
#include <math.h>
#include <mbed.h>
#include <stdio.h>
#include <string.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

struct MissionRequest {
    uint8_t task_id;
};

static Mail<MissionRequest, 4> mission_mail;
static volatile bool mission_active         = false;
static volatile bool mission_stop_requested = false;
static volatile uint8_t mission_task_id     = 0;
static char mission_phase[40]               = "idle";
static uint32_t mission_start_ms            = 0;

static float wrap_deg_180(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

static float wrap_deg_360(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}

static bool should_stop() {
    return mission_stop_requested || running_state == RunningState::STOPPED;
}

static void set_phase(const char* phase) {
    if (phase == nullptr) phase = "unknown";
    snprintf(mission_phase, sizeof(mission_phase), "%s", phase);
    loggf("mission phase=%s\n", mission_phase);
}

static void set_phase_cross(const char* prefix, uint8_t index) {
    if (prefix == nullptr) prefix = "cross";
    snprintf(mission_phase, sizeof(mission_phase), "%s_%u", prefix, index);
    loggf("mission phase=%s\n", mission_phase);
}

static void stop_actions() {
    line_follower_stop();
    wall_follower_stop();
    chassis_stop();
}

static void reset_drive_counts(int32_t& fl, int32_t& fr, int32_t& rl, int32_t& rr) {
    fl = motor_fl().count();
    fr = motor_fr().count();
    rl = motor_rl().count();
    rr = motor_rr().count();
}

static float traveled_cm(int32_t fl0, int32_t fr0, int32_t rl0, int32_t rr0) {
    const float fl             = static_cast<float>(motor_fl().count() - fl0);
    const float fr             = static_cast<float>(motor_fr().count() - fr0);
    const float rl             = static_cast<float>(motor_rl().count() - rl0);
    const float rr             = static_cast<float>(motor_rr().count() - rr0);
    const float forward_counts = fabsf((fl + fr + rl + rr) * 0.25f);
    return forward_counts / CHASSIS_ENCODER_COUNTS_PER_CM;
}

static bool wait_line_done(uint32_t timeout_ms) {
    const uint32_t start_ms = millis();
    while (line_follower.is_active()) {
        if (should_stop()) {
            line_follower_stop();
            return false;
        }

        if (millis() - start_ms >= timeout_ms) {
            loggf("mission line timeout phase=%s\n", mission_phase);
            line_follower_stop();
            return false;
        }

        ThisThread::sleep_for(20ms);
    }

    return !should_stop();
}

static bool wait_wall_done(uint32_t timeout_ms) {
    const uint32_t start_ms = millis();
    while (wall_follower.is_active()) {
        if (should_stop()) {
            wall_follower_stop();
            return false;
        }

        if (millis() - start_ms >= timeout_ms) {
            loggf("mission wall timeout phase=%s\n", mission_phase);
            wall_follower_stop();
            return false;
        }

        ThisThread::sleep_for(20ms);
    }

    return !should_stop();
}

static bool drive_blocking(float speed, float target_cm, int16_t front_stop_cm) {
    int32_t fl0 = 0;
    int32_t fr0 = 0;
    int32_t rl0 = 0;
    int32_t rr0 = 0;
    reset_drive_counts(fl0, fr0, rl0, rr0);
    chassis.set_target(speed, 0.0f, 0.0f);

    const uint32_t start_ms = millis();
    while (true) {
        if (should_stop()) {
            chassis_stop();
            return false;
        }

        const float dist_cm    = traveled_cm(fl0, fr0, rl0, rr0);
        const int16_t front_cm = sensors.ultrasonic_front_cm();
        if (front_stop_cm > 0 && front_cm > 0 && front_cm <= front_stop_cm) {
            chassis_stop();
            loggf("mission drive front stop phase=%s front=%d threshold=%d dist=%.1f\n",
            mission_phase,
            front_cm,
            front_stop_cm,
            dist_cm);
            return true;
        }

        if (target_cm > 0.0f && dist_cm >= target_cm) {
            chassis_stop();
            loggf("mission drive done phase=%s dist=%.1f target=%.1f\n", mission_phase, dist_cm, target_cm);
            return true;
        }

        if (millis() - start_ms >= MISSION_DRIVE_TIMEOUT_MS) {
            chassis_stop();
            loggf("mission drive timeout phase=%s dist=%.1f target=%.1f\n", mission_phase, dist_cm, target_cm);
            return false;
        }

        ThisThread::sleep_for(20ms);
    }
}

static bool turn_blocking(float delta_deg, float max_w, float tolerance_deg, uint32_t timeout_ms) {
    if (!imu.yaw_is_ready()) {
        loggf("mission turn aborted: imu yaw is not ready\n");
        return false;
    }

    max_w         = constrain(fabsf(max_w), TURN_MIN_WHEEL_SPEED, TURN_MAX_WHEEL_SPEED);
    tolerance_deg = fmaxf(0.2f, fabsf(tolerance_deg));

    const float start_yaw   = imu.yaw_deg();
    const float target_yaw  = wrap_deg_360(start_yaw + delta_deg);
    float prev_err          = wrap_deg_180(target_yaw - start_yaw);
    uint8_t confirm         = 0;
    const uint32_t start_ms = millis();
    uint32_t last_ms        = start_ms;

    loggf("mission turn begin delta=%.2f start=%.2f target=%.2f\n", delta_deg, start_yaw, target_yaw);

    while (millis() - start_ms < timeout_ms) {
        if (should_stop()) {
            chassis_stop();
            return false;
        }

        const uint32_t now_ms = millis();
        const float dt_s      = fmaxf(0.001f, static_cast<float>(now_ms - last_ms) * 0.001f);
        last_ms               = now_ms;

        const float yaw          = imu.yaw_deg();
        const float err          = wrap_deg_180(target_yaw - yaw);
        const float yaw_rate_dps = -wrap_deg_180(err - prev_err) / dt_s;
        const float derr         = -yaw_rate_dps;
        prev_err                 = err;

        if (fabsf(err) <= tolerance_deg && fabsf(yaw_rate_dps) <= TURN_STOP_SPEED_DPS) {
            confirm++;
            chassis.set_target(0.0f, 0.0f, 0.0f);
            if (confirm >= TURN_CONFIRM_COUNT) {
                chassis_stop();
                loggf("mission turn done yaw=%.2f target=%.2f err=%.2f\n", yaw, target_yaw, err);
                return true;
            }
        } else {
            confirm                   = 0;
            float w                   = TURN_KP * err + TURN_KD * derr;
            const float slow_scale    = constrain(fabsf(err) / TURN_SLOW_ZONE_DEG, 0.25f, 1.0f);
            const float limited_max_w = fmaxf(TURN_MIN_WHEEL_SPEED, max_w * slow_scale);
            w                         = constrain(w, -limited_max_w, limited_max_w);
            if (fabsf(err) <= tolerance_deg) w = 0.0f;
            if (fabsf(w) > 0.001f && fabsf(w) < TURN_MIN_WHEEL_SPEED) w = w >= 0.0f ? TURN_MIN_WHEEL_SPEED : -TURN_MIN_WHEEL_SPEED;
            chassis.set_target(0.0f, 0.0f, TURN_DIRECTION * w);
        }

        ThisThread::sleep_for(20ms);
    }

    chassis_stop();
    loggf("mission turn timeout yaw=%.2f target=%.2f err=%.2f\n",
    imu.yaw_deg(),
    target_yaw,
    wrap_deg_180(target_yaw - imu.yaw_deg()));
    return false;
}

static uint8_t count_black_sensors() {
    uint8_t count          = 0;
    const uint16_t* values = sensors.ir_values();
    for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
        if (values[i] >= LINE_BLACK_THRESHOLD) count++;
    }
    return count;
}

static bool turn_to_line_blocking(float direction_deg) {
    const float direction   = direction_deg >= 0.0f ? 1.0f : -1.0f;
    const float command_w   = TURN_DIRECTION * direction * MISSION_LINE_SEARCH_W;
    const uint32_t start_ms = millis();
    uint8_t confirm         = 0;

    loggf("mission turn-to-line begin direction=%.0f command_w=%.1f\n", direction_deg, command_w);

    chassis.set_target(0.0f, 0.0f, command_w);
    while (millis() - start_ms < MISSION_LINE_SEARCH_TIMEOUT_MS) {
        if (should_stop()) {
            chassis_stop();
            return false;
        }

        const uint32_t elapsed_ms = millis() - start_ms;
        const uint8_t black_count = count_black_sensors();
        if (elapsed_ms >= MISSION_LINE_SEARCH_IGNORE_MS && black_count >= MISSION_LINE_SEARCH_BLACK_COUNT) {
            confirm++;
            if (confirm >= MISSION_LINE_SEARCH_CONFIRM) {
                chassis_stop();
                loggf("mission turn-to-line done black=%u pos=%u elapsed=%lums\n",
                black_count,
                sensors.ir_position(),
                static_cast<unsigned long>(elapsed_ms));
                return true;
            }
        } else {
            confirm = 0;
        }

        ThisThread::sleep_for(20ms);
    }

    chassis_stop();
    loggf("mission turn-to-line timeout black=%u pos=%u\n", count_black_sensors(), sensors.ir_position());
    return false;
}

static bool turn_imu_then_line_blocking(float delta_deg) {
    const float imu_delta_deg = delta_deg * MISSION_TURN_IMU_RATIO;
    loggf("mission turn hybrid delta=%.1f imu=%.1f ir_dir=%.1f\n", delta_deg, imu_delta_deg, delta_deg);

    if (!turn_blocking(imu_delta_deg, TURN_MAX_WHEEL_SPEED, TURN_TOLERANCE_DEG, MISSION_TURN_TIMEOUT_MS)) return false;
    return turn_to_line_blocking(delta_deg);
}

static bool wait_blocking(uint32_t duration_ms) {
    const uint32_t start_ms = millis();
    while (millis() - start_ms < duration_ms) {
        if (should_stop()) return false;
        ThisThread::sleep_for(20ms);
    }

    return true;
}

static bool run_standard_line() {
    set_phase("standard_line");
    line_follower.start(TASK1_LINE_FOLLOW_SPEED);
    return wait_line_done(TASK1_LINE_FOLLOW_TIMEOUT_MS);
}

static bool run_open_field() {
    set_phase("task4_forward_2_nodes");
    if (!drive_blocking(TASK4_DRIVE_SPEED, TASK4_NODE_CM * 2.0f, -1)) return false;
    if (!wait_blocking(TASK4_ACTION_SETTLE_MS)) return false;

    set_phase("task4_turn_right");
    if (!turn_blocking(TASK4_RIGHT_TURN_DEG, TURN_MAX_WHEEL_SPEED, TURN_TOLERANCE_DEG, MISSION_TURN_TIMEOUT_MS)) return false;
    if (!wait_blocking(TASK4_ACTION_SETTLE_MS)) return false;

    set_phase("task4_forward_half_node");
    if (!drive_blocking(TASK4_DRIVE_SPEED, TASK4_NODE_CM * 0.5f, -1)) return false;
    if (!wait_blocking(TASK4_ACTION_SETTLE_MS)) return false;

    set_phase("task4_turn_left");
    if (!turn_blocking(TASK4_LEFT_TURN_DEG, TURN_MAX_WHEEL_SPEED, TURN_TOLERANCE_DEG, MISSION_TURN_TIMEOUT_MS)) return false;
    if (!wait_blocking(TASK4_ACTION_SETTLE_MS)) return false;

    set_phase("task4_forward_2_nodes_final");
    if (!drive_blocking(TASK4_DRIVE_SPEED, TASK4_NODE_CM * 2.0f, -1)) return false;
    return wait_blocking(TASK4_ACTION_SETTLE_MS);
}

static bool run_ramp() {
    set_phase("ramp_approach_left_wall");
    int32_t fl0 = 0;
    int32_t fr0 = 0;
    int32_t rl0 = 0;
    int32_t rr0 = 0;
    reset_drive_counts(fl0, fr0, rl0, rr0);

    const uint32_t start_ms = millis();
    uint32_t last_status_ms = 0;

    loggf("mission ramp stage1 approach speed=%.1f max=%.1f left_trigger=%d\n",
    TASK_RAMP_SPEED,
    TASK5_APPROACH_MAX_CM,
    TASK5_LEFT_WALL_TRIGGER_CM);

    while (true) {
        if (should_stop()) {
            chassis_stop();
            return false;
        }

        const float dist_cm    = traveled_cm(fl0, fr0, rl0, rr0);
        const int16_t front_cm = sensors.ultrasonic_front_cm();
        const int16_t left_cm  = sensors.ultrasonic_left_cm();

        if (front_cm > 0 && front_cm <= LINE_DEFAULT_FRONT_STOP_CM) {
            chassis_stop();
            loggf("mission ramp front stop front=%d threshold=%d dist=%.1f\n",
            front_cm,
            LINE_DEFAULT_FRONT_STOP_CM,
            dist_cm);
            return true;
        }

        if (left_cm > 0 && left_cm < TASK5_LEFT_WALL_TRIGGER_CM) {
            chassis_stop();
            loggf("mission ramp left wall found left=%d threshold=%d dist=%.1f\n",
            left_cm,
            TASK5_LEFT_WALL_TRIGGER_CM,
            dist_cm);
            break;
        }

        if (dist_cm >= TASK5_APPROACH_MAX_CM || millis() - start_ms >= MISSION_DRIVE_TIMEOUT_MS) {
            chassis_stop();
            loggf("mission ramp left wall not found left=%d dist=%.1f max=%.1f\n",
            left_cm,
            dist_cm,
            TASK5_APPROACH_MAX_CM);
            return false;
        }

        chassis.set_target(TASK_RAMP_SPEED, 0.0f, 0.0f);

        const uint32_t now_ms = millis();
        if (last_status_ms == 0 || now_ms - last_status_ms >= WALL_STATUS_INTERVAL_MS) {
            last_status_ms = now_ms;
            loggf("mission ramp approach dist=%.1f/%.1f left=%d/%d front=%d\n",
            dist_cm,
            TASK5_APPROACH_MAX_CM,
            left_cm,
            TASK5_LEFT_WALL_TRIGGER_CM,
            front_cm);
        }

        ThisThread::sleep_for(20ms);
    }

    if (!wait_blocking(TASK4_ACTION_SETTLE_MS)) return false;

    set_phase("ramp_wall_follow_left");
    wall_follower.set_pid(TASK5_WALL_DIST_KP, TASK5_WALL_YAW_KP, TASK5_WALL_MAX_WHEEL_SPEED);
    wall_follower.start(WallFollower::Side::Left, TASK_RAMP_SPEED, TASK_RAMP_DISTANCE_CM, TASK5_LEFT_WALL_TARGET_CM);
    return wait_wall_done(MISSION_LINE_TIMEOUT_MS);
}

static bool run_wall_task() {
    int16_t target_cm = TASK6_WALL_TARGET_CM;
    if (target_cm <= 0) target_cm = sensors.ultrasonic_left_cm();
    if (target_cm <= 0) target_cm = 20;

    set_phase("wall_follow_left");
    loggf("mission task6 wall left speed=%.1f dist=%.1f target=%d\n",
    TASK6_WALL_SPEED,
    TASK6_WALL_DISTANCE_CM,
    target_cm);
    wall_follower.start(WallFollower::Side::Left, TASK6_WALL_SPEED, TASK6_WALL_DISTANCE_CM, target_cm);
    return wait_wall_done(MISSION_LINE_TIMEOUT_MS);
}

static bool run_exit_base() {
    set_phase("find_cross");
    line_follower.start(TASK_LINE_SPEED, LineFollower::StopMode::Cross, LINE_DEFAULT_FRONT_STOP_CM);
    if (!wait_line_done(MISSION_LINE_TIMEOUT_MS)) return false;

    set_phase("center_cross");
    if (!drive_blocking(TASK_DRIVE_SPEED, TASK2_CENTER_DRIVE_CM, -1)) return false;

    set_phase("turn_cross_right");
    if (!turn_imu_then_line_blocking(TASK2_RIGHT_TURN_DEG)) return false;

    set_phase("find_left_corner_1");
    line_follower.start(TASK_LINE_SPEED, LineFollower::StopMode::LeftCorner, LINE_DEFAULT_FRONT_STOP_CM);
    if (!wait_line_done(MISSION_LINE_TIMEOUT_MS)) return false;

    set_phase("center_left_corner_1");
    if (!drive_blocking(TASK_DRIVE_SPEED, TASK2_CENTER_DRIVE_CM, -1)) return false;

    set_phase("turn_left_1");
    if (!turn_imu_then_line_blocking(TASK2_LEFT_TURN_DEG)) return false;

    set_phase("find_rfid");
    line_follower.start_rfid(TASK_LINE_SPEED, 0, true, true, LINE_DEFAULT_FRONT_STOP_CM);
    if (!wait_line_done(MISSION_LINE_TIMEOUT_MS)) return false;

    set_phase("wait_rfid");
    if (!wait_blocking(TASK2_RFID_WAIT_MS)) return false;

    set_phase("find_left_corner_2");
    line_follower.start(TASK_LINE_SPEED, LineFollower::StopMode::LeftCorner, LINE_DEFAULT_FRONT_STOP_CM);
    if (!wait_line_done(MISSION_LINE_TIMEOUT_MS)) return false;

    set_phase("center_left_corner_2");
    if (!drive_blocking(TASK_DRIVE_SPEED, TASK2_CENTER_DRIVE_CM, -1)) return false;

    set_phase("turn_left_2");
    if (!turn_imu_then_line_blocking(TASK2_LEFT_TURN_DEG)) return false;

    set_phase("find_right_corner");
    line_follower.start(TASK_LINE_SPEED, LineFollower::StopMode::RightCorner, LINE_DEFAULT_FRONT_STOP_CM);
    if (!wait_line_done(MISSION_LINE_TIMEOUT_MS)) return false;

    set_phase("center_right_corner");
    if (!drive_blocking(TASK_DRIVE_SPEED, TASK2_CENTER_DRIVE_CM, -1)) return false;

    set_phase("turn_right_2");
    if (!turn_imu_then_line_blocking(TASK2_RIGHT_TURN_DEG)) return false;

    set_phase("go_to_wall");
    line_follower.start(TASK_LINE_SPEED, LineFollower::StopMode::Front, TASK2_DOOR_FRONT_CM);
    if (!wait_line_done(MISSION_LINE_TIMEOUT_MS)) return false;

    return true;
}

static bool follow_cross_node(const char* phase_prefix, uint8_t index, bool clear_after) {
    set_phase_cross(phase_prefix, index);
    line_follower.start(TASK_LINE_SPEED, LineFollower::StopMode::Cross, -1);
    if (!wait_line_done(MISSION_LINE_TIMEOUT_MS)) return false;

    if (clear_after) {
        set_phase("clear_cross");
        if (!drive_blocking(TASK_DRIVE_SPEED, TASK7_CROSS_CLEAR_CM, -1)) return false;
    }

    return true;
}

static bool follow_cross_nodes(const char* phase_prefix, uint8_t count, bool clear_after_last) {
    for (uint8_t i = 0; i < count; i++) {
        const bool clear_after = clear_after_last || i + 1 < count;
        if (!follow_cross_node(phase_prefix, i + 1, clear_after)) return false;
    }

    return true;
}

static bool run_solid_grid() {
    if (!follow_cross_nodes("grid_first", TASK3_FIRST_STRAIGHT_NODES, true)) return false;

    set_phase("grid_turn_right");
    if (!turn_imu_then_line_blocking(TASK2_RIGHT_TURN_DEG)) return false;

    if (!follow_cross_nodes("grid_middle", TASK3_MIDDLE_STRAIGHT_NODES, true)) return false;

    set_phase("grid_turn_left");
    if (!turn_imu_then_line_blocking(TASK2_LEFT_TURN_DEG)) return false;

    return follow_cross_nodes("grid_last", TASK3_LAST_STRAIGHT_NODES, false);
}

static bool find_obstacle_at_cross() {
    for (uint8_t i = 0; i < TASK7_DETECT_MAX_NODES; i++) {
        if (!follow_cross_node("obstacle_detect_cross", i + 1, false)) return false;

        const int16_t front_cm = sensors.ultrasonic_front_cm();
        if (front_cm > 0 && front_cm <= TASK_OBSTACLE_FRONT_CM) {
            loggf("mission obstacle detected node=%u front=%d threshold=%d\n", i + 1, front_cm, TASK_OBSTACLE_FRONT_CM);
            set_phase("obstacle_turn_center");
            if (!drive_blocking(TASK_DRIVE_SPEED, TASK2_CENTER_DRIVE_CM, -1)) return false;
            return true;
        }

        loggf("mission obstacle not detected node=%u front=%d threshold=%d\n", i + 1, front_cm, TASK_OBSTACLE_FRONT_CM);
        if (i + 1 < TASK7_DETECT_MAX_NODES) {
            set_phase("obstacle_detect_clear");
            if (!drive_blocking(TASK_DRIVE_SPEED, TASK7_CROSS_CLEAR_CM, -1)) return false;
        }
    }

    loggf("mission obstacle detect failed after %u nodes front=%d threshold=%d\n",
    TASK7_DETECT_MAX_NODES,
    sensors.ultrasonic_front_cm(),
    TASK_OBSTACLE_FRONT_CM);
    return false;
}

static bool run_obstacle_avoidance() {
    set_phase("obstacle_detect");
    if (!find_obstacle_at_cross()) return false;

    set_phase("obstacle_turn_right_1");
    if (!turn_imu_then_line_blocking(TASK2_RIGHT_TURN_DEG)) return false;

    if (!follow_cross_nodes("obstacle_side_out", TASK7_SIDE_NODES, true)) return false;

    set_phase("obstacle_turn_left_1");
    if (!turn_imu_then_line_blocking(TASK2_LEFT_TURN_DEG)) return false;

    if (!follow_cross_nodes("obstacle_pass", TASK7_PASS_NODES, true)) return false;

    set_phase("obstacle_turn_left_2");
    if (!turn_imu_then_line_blocking(TASK2_LEFT_TURN_DEG)) return false;

    if (!follow_cross_nodes("obstacle_side_back", TASK7_SIDE_NODES, true)) return false;

    set_phase("obstacle_turn_right_2");
    if (!turn_imu_then_line_blocking(TASK2_RIGHT_TURN_DEG)) return false;

    return follow_cross_nodes("obstacle_finish", TASK7_FINISH_NODES, false);
}

static bool run_revive() {
    set_phase("revive_fast");

    int32_t fl0 = 0;
    int32_t fr0 = 0;
    int32_t rl0 = 0;
    int32_t rr0 = 0;
    reset_drive_counts(fl0, fr0, rl0, rr0);

    uint8_t zone            = 0;
    float speed             = TASK8_FAST_SPEED;
    uint32_t last_status_ms = 0;
    const uint32_t start_ms = millis();
    line_follower.start(speed);

    loggf("mission revive line begin fast=%.1f mid=%.1f contact=%.1f mid_front=%d contact_front=%d max_dist=%.1f timeout=%lums\n",
    TASK8_FAST_SPEED,
    TASK8_MID_SPEED,
    TASK8_CONTACT_SPEED,
    TASK8_MID_FRONT_CM,
    TASK8_CONTACT_FRONT_CM,
    TASK8_MAX_DISTANCE_CM,
    static_cast<unsigned long>(TASK8_TIMEOUT_MS));

    while (millis() - start_ms < TASK8_TIMEOUT_MS) {
        if (should_stop()) {
            line_follower_stop();
            return false;
        }

        const float dist_cm    = traveled_cm(fl0, fr0, rl0, rr0);
        const int16_t front_cm = sensors.ultrasonic_front_cm();
        const bool touched     = sensors.revive_button_pressed();

        if (touched) {
            line_follower_stop();
            set_phase("revive_contact");
            loggf("mission revive contact front=%d dist=%.1f elapsed=%lums\n",
            front_cm,
            dist_cm,
            static_cast<unsigned long>(millis() - start_ms));
            return true;
        }

        if (!line_follower.is_active()) {
            loggf("mission revive line stopped before contact front=%d dist=%.1f button=0\n", front_cm, dist_cm);
            return false;
        }

        if (dist_cm >= TASK8_MAX_DISTANCE_CM) {
            line_follower_stop();
            loggf("mission revive max distance front=%d dist=%.1f target=%.1f button=0\n",
            front_cm,
            dist_cm,
            TASK8_MAX_DISTANCE_CM);
            return false;
        }

        uint8_t next_zone = zone;
        float next_speed  = speed;
        if (front_cm > 0 && front_cm <= TASK8_CONTACT_FRONT_CM) {
            next_zone  = 2;
            next_speed = TASK8_CONTACT_SPEED;
        } else if (front_cm > 0 && front_cm <= TASK8_MID_FRONT_CM) {
            next_zone  = 1;
            next_speed = TASK8_MID_SPEED;
        } else if (front_cm > TASK8_MID_FRONT_CM) {
            next_zone  = 0;
            next_speed = TASK8_FAST_SPEED;
        }

        if (next_zone != zone || fabsf(next_speed - speed) > 0.1f) {
            zone  = next_zone;
            speed = next_speed;
            if (zone == 0)
                set_phase("revive_fast");
            else if (zone == 1)
                set_phase("revive_mid");
            else
                set_phase("revive_touch");
            line_follower.set_speed(speed);
        }

        const uint32_t now_ms = millis();
        if (last_status_ms == 0 || now_ms - last_status_ms >= 500) {
            last_status_ms = now_ms;
            loggf("mission revive active phase=%s speed=%.1f front=%d dist=%.1f button=0\n",
            mission_phase,
            speed,
            front_cm,
            dist_cm);
        }

        ThisThread::sleep_for(20ms);
    }

    line_follower_stop();
    loggf("mission revive timeout front=%d dist=%.1f button=%d\n",
    sensors.ultrasonic_front_cm(),
    traveled_cm(fl0, fr0, rl0, rr0),
    sensors.revive_button_pressed() ? 1 : 0);
    return false;
}

static void run_task(uint8_t task_id) {
    mission_active         = true;
    mission_stop_requested = false;
    mission_task_id        = task_id;
    mission_start_ms       = millis();
    set_phase("start");
    stop_actions();

    loggf("mission start task=%u\n", task_id);

    bool ok = false;
    if (task_id == 1) {
        ok = run_standard_line();
    } else if (task_id == 2) {
        ok = run_exit_base();
    } else if (task_id == 3) {
        ok = run_solid_grid();
    } else if (task_id == 4) {
        ok = run_open_field();
    } else if (task_id == 5) {
        ok = run_ramp();
    } else if (task_id == 6) {
        ok = run_wall_task();
    } else if (task_id == 7) {
        ok = run_obstacle_avoidance();
    } else if (task_id == 8) {
        ok = run_revive();
    } else {
        loggf("mission task %u not implemented yet\n", task_id);
    }

    stop_actions();
    loggf("mission %s task=%u phase=%s elapsed=%lums\n",
    ok ? "done" : "stopped",
    task_id,
    mission_phase,
    static_cast<unsigned long>(millis() - mission_start_ms));

    mission_active  = false;
    mission_task_id = 0;
    set_phase("idle");
}

bool mission_start_task(uint8_t task_id) {
    if (mission_active) {
        loggf("mission busy task=%u phase=%s\n", mission_task_id, mission_phase);
        return false;
    }

    auto* request = mission_mail.try_alloc();
    if (request == nullptr) {
        loggf("mission queue full\n");
        return false;
    }

    request->task_id = task_id;
    mission_mail.put(request);
    loggf("mission queued task=%u\n", task_id);
    return true;
}

void mission_stop() {
    mission_stop_requested = true;
    if (mission_active) loggf("mission stop requested task=%u phase=%s\n", mission_task_id, mission_phase);
    stop_actions();
}

void mission_print_status() {
    loggf("mission active=%d task=%u phase=%s elapsed=%lums stop=%d\n",
    mission_active ? 1 : 0,
    mission_task_id,
    mission_phase,
    mission_active ? static_cast<unsigned long>(millis() - mission_start_ms) : 0UL,
    mission_stop_requested ? 1 : 0);
}

void func_mission_entry() {
    termthree_navigation_begin();

    while (true) {
        auto* request = mission_mail.try_get_for(100ms);
        termthree_navigation_tick();
        if (request == nullptr) continue;

        const uint8_t task_id = request->task_id;
        mission_mail.free(request);
        run_task(task_id);
    }
}
