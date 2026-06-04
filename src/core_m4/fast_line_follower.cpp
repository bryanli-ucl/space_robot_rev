#include "fast_line_follower.hpp"

#include "chassis.hpp"
#include "config.hpp"
#include "logger.hpp"
#include "sensors.hpp"

#include <Arduino.h>
#include <math.h>

FastLineFollower& fast_line_follower = FastLineFollower::instance();

FastLineFollower& FastLineFollower::instance() {
    static FastLineFollower instance;
    return instance;
}

void FastLineFollower::start() {
    active         = true;
    recovery       = RecoveryMode::None;
    integral       = 0.0f;
    error          = 0.0f;
    prev_error     = 0.0f;
    vx             = 0.0f;
    w              = 0.0f;
    center_confirm_count = 0;
    last_status_ms       = 0;
    loggf("linef start speed=%.1f pid=%.3f/%.3f/%.3f\n", speed, kp_value, ki_value, kd_value);
}

void FastLineFollower::stop() {
    if (active) loggf("linef stop\n");
    active   = false;
    recovery = RecoveryMode::None;
    vx       = 0.0f;
    w        = 0.0f;
    center_confirm_count = 0;
    chassis_stop();
}

void FastLineFollower::set_speed(float next_speed) {
    speed = next_speed;
}

void FastLineFollower::set_pid(float kp, float ki, float kd) {
    kp_value = kp;
    ki_value = ki;
    kd_value = kd;
    integral = 0.0f;
}

void FastLineFollower::set_recovery_speed_scale(float scale) {
    recovery_speed_scale_value = constrain(scale, 0.0f, 1.0f);
}

bool FastLineFollower::center_sees_line() const {
    return sensors.ir_value(LINEF_CENTER_SENSOR) >= LINEF_CENTER_THRESHOLD;
}

const char* FastLineFollower::recovery_name() const {
    switch (recovery) {
    case RecoveryMode::None: return "none";
    case RecoveryMode::Left: return "left";
    case RecoveryMode::Right: return "right";
    }
    return "unknown";
}

void FastLineFollower::update(float dt_s) {
    if (!active) return;

    if (recovery == RecoveryMode::None) {
        if (sensors.ir_side_left_detected()) {
            recovery             = RecoveryMode::Left;
            integral             = 0.0f;
            center_confirm_count = 0;
            chassis_stop();
            loggf("linef edge left value=%u\n", sensors.ir_side_left_value());
        } else if (sensors.ir_side_right_detected()) {
            recovery             = RecoveryMode::Right;
            integral             = 0.0f;
            center_confirm_count = 0;
            chassis_stop();
            loggf("linef edge right value=%u\n", sensors.ir_side_right_value());
        }
    }

    if (recovery != RecoveryMode::None) {
        vx = speed * recovery_speed_scale_value;

        if (center_sees_line()) {
            if (center_confirm_count < LINEF_CENTER_CONFIRM) center_confirm_count++;
            w = 0.0f;
            chassis.set_target(vx, 0.0f, 0.0f);

            if (center_confirm_count >= LINEF_CENTER_CONFIRM) {
                recovery             = RecoveryMode::None;
                center_confirm_count = 0;
                integral             = 0.0f;
                error                = 0.0f;
                prev_error           = 0.0f;
                loggf("linef center found value=%u pos=%u\n",
                sensors.ir_value(LINEF_CENTER_SENSOR),
                sensors.ir_position());
            }
            return;
        }

        center_confirm_count = 0;
        const float direction = recovery == RecoveryMode::Left ? -1.0f : 1.0f;
        w                     = TURN_DIRECTION * direction * LINEF_EDGE_SEARCH_W;
        chassis.set_target(vx, 0.0f, w);

        const uint32_t now_ms = millis();
        if (last_status_ms == 0 || now_ms - last_status_ms >= LINEF_STATUS_INTERVAL_MS) {
            last_status_ms = now_ms;
            print_status();
        }
        return;
    }

    error = static_cast<float>(static_cast<int32_t>(sensors.ir_position()) - LINE_CENTER_POS);
    integral += error * dt_s;
    integral = constrain(integral, -LINEF_INTEGRAL_LIMIT, LINEF_INTEGRAL_LIMIT);

    const float derivative = dt_s > 0.0f ? (error - prev_error) / dt_s : 0.0f;
    prev_error             = error;

    const float abs_err_norm = constrain(fabsf(error) / static_cast<float>(LINE_CENTER_POS), 0.0f, 1.0f);
    vx                       = speed * (1.0f - (1.0f - LINEF_MIN_SPEED_SCALE) * abs_err_norm);
    w                        = LINE_DIRECTION * (kp_value * error + ki_value * integral + kd_value * derivative);
    w                        = constrain(w, -LINEF_MAX_WHEEL_SPEED, LINEF_MAX_WHEEL_SPEED);

    chassis.set_target(vx, 0.0f, w);

    const uint32_t now_ms = millis();
    if (last_status_ms == 0 || now_ms - last_status_ms >= LINEF_STATUS_INTERVAL_MS) {
        last_status_ms = now_ms;
        print_status();
    }
}

void FastLineFollower::print_status() const {
    loggf("linef active=%d speed=%.1f rot=%.0f%% pid=%.3f/%.3f/%.3f recovery=%s center_ok=%u pos=%u center=%u side=%u/%u det=%d/%d err=%.0f int=%.1f vx=%.1f w=%.1f\n",
    active ? 1 : 0,
    speed,
    recovery_speed_scale_value * 100.0f,
    kp_value,
    ki_value,
    kd_value,
    recovery_name(),
    center_confirm_count,
    sensors.ir_position(),
    sensors.ir_value(LINEF_CENTER_SENSOR),
    sensors.ir_side_left_value(),
    sensors.ir_side_right_value(),
    sensors.ir_side_left_detected() ? 1 : 0,
    sensors.ir_side_right_detected() ? 1 : 0,
    error,
    integral,
    vx,
    w);
}

void fast_line_follower_update(float dt_s) {
    fast_line_follower.update(dt_s);
}

void fast_line_follower_stop() {
    fast_line_follower.stop();
}
