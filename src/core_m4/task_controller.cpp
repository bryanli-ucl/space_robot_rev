#include "task_controller.hpp"

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

#include <Arduino.h>
#include <math.h>

TaskController& task_controller = TaskController::instance();

TaskController& TaskController::instance() {
    static TaskController instance;
    return instance;
}

void TaskController::stop_actions() {
    line_follower_stop();
    wall_follower_stop();
    chassis_stop();
    drive_active = false;
}

void TaskController::stop() {
    if (active) loggf("task stop id=%u phase=%s\n", task_id, phase);
    stop_actions();
    active = false;
    task_id = 0;
    phase = "idle";
}

void TaskController::begin_drive(float speed, float target_cm, int16_t front_stop_cm, const char* next_phase) {
    start_fl_count = motor_fl().count();
    start_fr_count = motor_fr().count();
    start_rl_count = motor_rl().count();
    start_rr_count = motor_rr().count();
    drive_speed = speed;
    drive_target_cm = target_cm;
    drive_front_stop_cm = front_stop_cm;
    drive_traveled_cm = 0.0f;
    drive_active = true;
    phase = next_phase;
    chassis.set_target(speed, 0.0f, 0.0f);
}

void TaskController::start(uint8_t next_task_id) {
    if (running_state == RunningState::STOPPED) {
        loggf("task aborted: robot is stopped. use start first.\n");
        return;
    }

    stop_actions();
    active = true;
    task_id = next_task_id;
    start_ms = millis();
    last_status_ms = 0;

    switch (task_id) {
    case 1:
        phase = "standard-line";
        line_follower.start(TASK_LINE_SPEED);
        break;
    case 2:
        phase = "intersection-rfid";
        line_follower.start_rfid(TASK_LINE_SPEED, 0, true, false, LINE_DEFAULT_FRONT_STOP_CM);
        break;
    case 3:
        phase = "solid-grid-rfid";
        line_follower.start_rfid(TASK_LINE_SPEED, 0, true, true, LINE_DEFAULT_FRONT_STOP_CM);
        break;
    case 4:
        begin_drive(TASK_DRIVE_SPEED, TASK_OPEN_FIELD_DISTANCE_CM, -1, "open-field-drive");
        break;
    case 5:
        begin_drive(TASK_RAMP_SPEED, TASK_RAMP_DISTANCE_CM, LINE_DEFAULT_FRONT_STOP_CM, "ramp-drive");
        break;
    case 6: {
        int16_t target_cm = sensors.ultrasonic_right_cm();
        if (target_cm <= 0) target_cm = 20;
        phase = "wall-follow-right";
        wall_follower.start(WallFollower::Side::Right, TASK_WALL_SPEED, 100.0f, target_cm);
        break;
    }
    case 7:
        phase = "obstacle-detect-line";
        line_follower.start(TASK_LINE_SPEED, LineFollower::StopMode::Front, TASK_OBSTACLE_FRONT_CM);
        break;
    case 8:
        begin_drive(TASK_REVIVE_SPEED, TASK_REVIVE_MAX_DISTANCE_CM, TASK_REVIVE_FRONT_CM, "revive-approach");
        break;
    default:
        active = false;
        phase = "idle";
        loggf("task id must be 1..8\n");
        return;
    }

    loggf("task start id=%u phase=%s\n", task_id, phase);
}

void TaskController::update_drive_distance() {
    const float fl             = static_cast<float>(motor_fl().count() - start_fl_count);
    const float fr             = static_cast<float>(motor_fr().count() - start_fr_count);
    const float rl             = static_cast<float>(motor_rl().count() - start_rl_count);
    const float rr             = static_cast<float>(motor_rr().count() - start_rr_count);
    const float forward_counts = fabsf((fl + fr + rl + rr) * 0.25f);
    drive_traveled_cm          = forward_counts / CHASSIS_ENCODER_COUNTS_PER_CM;
}

void TaskController::update_drive() {
    if (!drive_active) return;

    update_drive_distance();
    const int16_t front_cm = sensors.ultrasonic_front_cm();
    if (drive_front_stop_cm > 0 && front_cm > 0 && front_cm <= drive_front_stop_cm) {
        chassis_stop();
        drive_active = false;
        active = false;
        loggf("task drive front stop id=%u front=%d threshold=%d traveled=%.1f\n",
        task_id,
        front_cm,
        drive_front_stop_cm,
        drive_traveled_cm);
        return;
    }

    if (drive_target_cm > 0.0f && drive_traveled_cm >= drive_target_cm) {
        chassis_stop();
        drive_active = false;
        active = false;
        loggf("task drive done id=%u traveled=%.1f target=%.1f\n", task_id, drive_traveled_cm, drive_target_cm);
    }
}

void TaskController::update(float dt_s) {
    (void)dt_s;
    if (!active) return;

    if (running_state == RunningState::STOPPED) {
        stop();
        return;
    }

    update_drive();

    const uint32_t now_ms = millis();
    if (last_status_ms == 0 || now_ms - last_status_ms >= 1000) {
        last_status_ms = now_ms;
        print_status();
    }
}

void TaskController::print_status() const {
    loggf("task active=%d id=%u phase=%s elapsed=%lums line=%d wall=%d drive=%d dist=%.1f/%.1f front=%d rfid=%lu yaw=%.2f\n",
    active ? 1 : 0,
    task_id,
    phase,
    active ? static_cast<unsigned long>(millis() - start_ms) : 0UL,
    line_follower.is_active() ? 1 : 0,
    wall_follower.is_active() ? 1 : 0,
    drive_active ? 1 : 0,
    drive_traveled_cm,
    drive_target_cm,
    sensors.ultrasonic_front_cm(),
    static_cast<unsigned long>(rfid.last_uid()),
    imu.yaw_deg());
}

void task_controller_update(float dt_s) {
    task_controller.update(dt_s);
}

void task_controller_stop() {
    task_controller.stop();
}
