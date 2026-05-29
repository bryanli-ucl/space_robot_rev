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
#include "task_controller.hpp"
#include "wall_follower.hpp"

#include <Arduino.h>
#include <math.h>
#include <mbed.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

struct MissionRequest {
    uint8_t task_id;
};

static Mail<MissionRequest, 4> mission_mail;
static volatile bool mission_active = false;
static volatile bool mission_stop_requested = false;
static volatile uint8_t mission_task_id = 0;
static const char* mission_phase = "idle";
static uint32_t mission_start_ms = 0;

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
    mission_phase = phase;
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

        const float dist_cm = traveled_cm(fl0, fr0, rl0, rr0);
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

    max_w = constrain(fabsf(max_w), TURN_MIN_WHEEL_SPEED, TURN_MAX_WHEEL_SPEED);
    tolerance_deg = fmaxf(0.2f, fabsf(tolerance_deg));

    const float start_yaw = imu.yaw_deg();
    const float target_yaw = wrap_deg_360(start_yaw + delta_deg);
    float prev_err = wrap_deg_180(target_yaw - start_yaw);
    uint8_t confirm = 0;
    const uint32_t start_ms = millis();
    uint32_t last_ms = start_ms;

    loggf("mission turn begin delta=%.2f start=%.2f target=%.2f\n", delta_deg, start_yaw, target_yaw);

    while (millis() - start_ms < timeout_ms) {
        if (should_stop()) {
            chassis_stop();
            return false;
        }

        const uint32_t now_ms = millis();
        const float dt_s = fmaxf(0.001f, static_cast<float>(now_ms - last_ms) * 0.001f);
        last_ms = now_ms;

        const float yaw = imu.yaw_deg();
        const float err = wrap_deg_180(target_yaw - yaw);
        const float yaw_rate_dps = -wrap_deg_180(err - prev_err) / dt_s;
        const float derr = -yaw_rate_dps;
        prev_err = err;

        if (fabsf(err) <= tolerance_deg && fabsf(yaw_rate_dps) <= TURN_STOP_SPEED_DPS) {
            confirm++;
            chassis.set_target(0.0f, 0.0f, 0.0f);
            if (confirm >= TURN_CONFIRM_COUNT) {
                chassis_stop();
                loggf("mission turn done yaw=%.2f target=%.2f err=%.2f\n", yaw, target_yaw, err);
                return true;
            }
        } else {
            confirm = 0;
            float w = TURN_KP * err + TURN_KD * derr;
            const float slow_scale = constrain(fabsf(err) / TURN_SLOW_ZONE_DEG, 0.25f, 1.0f);
            const float limited_max_w = fmaxf(TURN_MIN_WHEEL_SPEED, max_w * slow_scale);
            w = constrain(w, -limited_max_w, limited_max_w);
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
    uint8_t count = 0;
    const uint16_t* values = sensors.ir_values();
    for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
        if (values[i] >= LINE_BLACK_THRESHOLD) count++;
    }
    return count;
}

static bool turn_to_line_blocking(float direction_deg) {
    const float direction = direction_deg >= 0.0f ? 1.0f : -1.0f;
    const float command_w = TURN_DIRECTION * direction * MISSION_LINE_SEARCH_W;
    const uint32_t start_ms = millis();
    uint8_t confirm = 0;

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

static void run_task(uint8_t task_id) {
    mission_active = true;
    mission_stop_requested = false;
    mission_task_id = task_id;
    mission_start_ms = millis();
    mission_phase = "start";
    task_controller_stop();
    stop_actions();

    loggf("mission start task=%u\n", task_id);

    bool ok = false;
    if (task_id == 2) {
        ok = run_exit_base();
    } else {
        loggf("mission task %u not implemented yet\n", task_id);
    }

    stop_actions();
    loggf("mission %s task=%u phase=%s elapsed=%lums\n",
    ok ? "done" : "stopped",
    task_id,
    mission_phase,
    static_cast<unsigned long>(millis() - mission_start_ms));

    mission_active = false;
    mission_task_id = 0;
    mission_phase = "idle";
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
    while (true) {
        auto* request = mission_mail.try_get_for(100ms);
        if (request == nullptr) continue;

        const uint8_t task_id = request->task_id;
        mission_mail.free(request);
        run_task(task_id);
    }
}
