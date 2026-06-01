#include "motion_primitives.hpp"

#include "chassis.hpp"
#include "config.hpp"
#include "imu.hpp"
#include "line_follower.hpp"
#include "logger.hpp"
#include "motor.hpp"
#include "sensors.hpp"
#include "state.hpp"
#include "wall_follower.hpp"

#include <Arduino.h>
#include <math.h>
#include <mbed.h>

using namespace ::std::chrono_literals;
using namespace ::rtos;

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

static bool motion_should_stop() {
    return running_state == RunningState::STOPPED || sensors.kill_switch_pressed();
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

static uint8_t count_black_sensors() {
    uint8_t count          = 0;
    const uint16_t* values = sensors.ir_values();
    for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
        if (values[i] >= LINE_BLACK_THRESHOLD) count++;
    }
    return count;
}

void motion_stop_all() {
    line_follower_stop();
    wall_follower_stop();
    chassis_stop();
}

bool motion_wait_line_done(uint32_t timeout_ms) {
    const uint32_t start_ms = millis();
    while (line_follower.is_active()) {
        if (motion_should_stop()) {
            line_follower_stop();
            return false;
        }

        if (millis() - start_ms >= timeout_ms) {
            loggf("motion line timeout mode=%s\n", line_follower.stop_mode_name());
            line_follower_stop();
            return false;
        }

        ThisThread::sleep_for(20ms);
    }

    return !motion_should_stop();
}

bool motion_drive_blocking(float speed, float target_cm, int16_t front_stop_cm, uint32_t timeout_ms) {
    int32_t fl0 = 0;
    int32_t fr0 = 0;
    int32_t rl0 = 0;
    int32_t rr0 = 0;
    reset_drive_counts(fl0, fr0, rl0, rr0);
    chassis.set_target(speed, 0.0f, 0.0f);

    const uint32_t start_ms = millis();
    while (true) {
        if (motion_should_stop()) {
            chassis_stop();
            return false;
        }

        const float dist_cm    = traveled_cm(fl0, fr0, rl0, rr0);
        const int16_t front_cm = sensors.ultrasonic_front_cm();
        if (front_stop_cm > 0 && front_cm > 0 && front_cm <= front_stop_cm) {
            chassis_stop();
            loggf("motion drive front stop front=%d threshold=%d dist=%.1f\n", front_cm, front_stop_cm, dist_cm);
            return true;
        }

        if (target_cm > 0.0f && dist_cm >= target_cm) {
            chassis_stop();
            loggf("motion drive done dist=%.1f target=%.1f\n", dist_cm, target_cm);
            return true;
        }

        if (millis() - start_ms >= timeout_ms) {
            chassis_stop();
            loggf("motion drive timeout dist=%.1f target=%.1f\n", dist_cm, target_cm);
            return false;
        }

        ThisThread::sleep_for(20ms);
    }
}

bool motion_turn_blocking(float delta_deg, float max_w, float tolerance_deg, uint32_t timeout_ms) {
    if (!imu.yaw_is_ready()) {
        loggf("motion turn aborted: imu yaw is not ready\n");
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

    loggf("motion turn begin delta=%.2f start=%.2f target=%.2f\n", delta_deg, start_yaw, target_yaw);

    while (millis() - start_ms < timeout_ms) {
        if (motion_should_stop()) {
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
                loggf("motion turn done yaw=%.2f target=%.2f err=%.2f\n", yaw, target_yaw, err);
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
    loggf("motion turn timeout yaw=%.2f target=%.2f err=%.2f\n",
    imu.yaw_deg(),
    target_yaw,
    wrap_deg_180(target_yaw - imu.yaw_deg()));
    return false;
}

bool motion_turn_to_line_blocking(float direction_deg, float search_w, uint32_t timeout_ms) {
    const float direction   = direction_deg >= 0.0f ? 1.0f : -1.0f;
    const float command_w   = TURN_DIRECTION * direction * fabsf(search_w);
    const uint32_t start_ms = millis();
    uint8_t confirm         = 0;

    loggf("motion turn-to-line begin direction=%.0f command_w=%.1f\n", direction_deg, command_w);

    chassis.set_target(0.0f, 0.0f, command_w);
    while (millis() - start_ms < timeout_ms) {
        if (motion_should_stop()) {
            chassis_stop();
            return false;
        }

        const uint32_t elapsed_ms = millis() - start_ms;
        const uint8_t black_count = count_black_sensors();
        if (elapsed_ms >= MISSION_LINE_SEARCH_IGNORE_MS && black_count >= MISSION_LINE_SEARCH_BLACK_COUNT) {
            confirm++;
            if (confirm >= MISSION_LINE_SEARCH_CONFIRM) {
                chassis_stop();
                loggf("motion turn-to-line done black=%u pos=%u elapsed=%lums\n",
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
    loggf("motion turn-to-line timeout black=%u pos=%u\n", count_black_sensors(), sensors.ir_position());
    return false;
}

bool motion_turn_imu_then_line_blocking(float delta_deg) {
    const float imu_delta_deg = delta_deg * MISSION_TURN_IMU_RATIO;
    loggf("motion turn hybrid delta=%.1f imu=%.1f ir_dir=%.1f\n", delta_deg, imu_delta_deg, delta_deg);

    if (!motion_turn_blocking(imu_delta_deg, TURN_MAX_WHEEL_SPEED, TURN_TOLERANCE_DEG, MISSION_TURN_TIMEOUT_MS)) return false;
    return motion_turn_to_line_blocking(delta_deg, MISSION_LINE_SEARCH_W, MISSION_LINE_SEARCH_TIMEOUT_MS);
}

bool motion_wait_blocking(uint32_t duration_ms) {
    const uint32_t start_ms = millis();
    while (millis() - start_ms < duration_ms) {
        if (motion_should_stop()) return false;
        ThisThread::sleep_for(20ms);
    }

    return true;
}
