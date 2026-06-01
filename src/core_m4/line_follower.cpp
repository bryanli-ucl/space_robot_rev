#include "line_follower.hpp"

#include "chassis.hpp"
#include "config.hpp"
#include "logger.hpp"
#include "motor.hpp"
#include "rfid.hpp"
#include "sensors.hpp"

#include <Arduino.h>
#include <math.h>

LineFollower& line_follower = LineFollower::instance();

static uint8_t count_black_sensors() {
    uint8_t count          = 0;
    const uint16_t* values = sensors.ir_values();
    for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
        if (values[i] >= LINE_BLACK_THRESHOLD) count++;
    }
    return count;
}

LineFollower& LineFollower::instance() {
    static LineFollower instance;
    return instance;
}

void LineFollower::start(float next_speed, StopMode next_mode, int16_t next_front_stop_cm, float next_target_distance_cm) {
    speed                = next_speed;
    mode                 = next_mode;
    front_stop_cm        = next_front_stop_cm;
    target_distance_cm   = next_target_distance_cm;
    traveled_distance_cm = 0.0f;
    rfid_target_uid      = 0;
    rfid_ignore_uid      = 0;
    rfid_any_uid         = false;
    rfid_not_same        = false;
    start_fl_count       = motor_fl().count();
    start_fr_count       = motor_fr().count();
    start_rl_count       = motor_rl().count();
    start_rr_count       = motor_rr().count();
    active               = true;
    error                = 0.0f;
    prev_error           = 0.0f;
    w                    = 0.0f;
    vx                   = 0.0f;
    black_count          = 0;
    no_line_count        = 0;
    cross_count          = 0;
    left_corner_count    = 0;
    right_corner_count   = 0;
    last_status_ms       = 0;
    loggf("line start speed=%.1f mode=%s front=%d\n", speed, stop_mode_name(), front_stop_cm);
}

void LineFollower::start_rfid(float next_speed, uint32_t target_uid, bool any_uid, bool not_same, int16_t next_front_stop_cm) {
    start(next_speed, StopMode::Rfid, next_front_stop_cm);
    rfid_target_uid = target_uid;
    rfid_ignore_uid = not_same ? last_rfid_stop_uid : 0;
    rfid_any_uid    = any_uid;
    rfid_not_same   = not_same;

    loggf("line rfid start speed=%.1f target=%lu any=%d notsame=%d ignore=%lu current=%lu front=%d ready=%d\n",
    speed,
    static_cast<unsigned long>(rfid_target_uid),
    rfid_any_uid ? 1 : 0,
    rfid_not_same ? 1 : 0,
    static_cast<unsigned long>(rfid_ignore_uid),
    static_cast<unsigned long>(rfid.last_uid()),
    front_stop_cm,
    rfid.is_ready() ? 1 : 0);
}

void LineFollower::set_speed(float next_speed) {
    speed = next_speed;
}

void LineFollower::set_pid(float kp, float kd) {
    kp_value = kp;
    kd_value = kd;
}

void LineFollower::stop() {
    if (active) loggf("line stop\n");
    active               = false;
    mode                 = StopMode::None;
    front_stop_cm        = -1;
    target_distance_cm   = -1.0f;
    traveled_distance_cm = 0.0f;
    rfid_target_uid      = 0;
    rfid_ignore_uid      = 0;
    rfid_any_uid         = false;
    rfid_not_same        = false;
    speed                = 0.0f;
    w                    = 0.0f;
    vx                   = 0.0f;
    chassis_stop();
}

const char* LineFollower::stop_mode_name() const {
    switch (mode) {
    case StopMode::None: return "none";
    case StopMode::Cross: return "cross";
    case StopMode::Front: return "front";
    case StopMode::Distance: return "distance";
    case StopMode::Rfid: return "rfid";
    case StopMode::LeftCorner: return "left-corner";
    case StopMode::RightCorner: return "right-corner";
    case StopMode::AnyCorner: return "any-corner";
    }
    return "unknown";
}

void LineFollower::update_traveled_distance() {
    const float fl             = static_cast<float>(motor_fl().count() - start_fl_count);
    const float fr             = static_cast<float>(motor_fr().count() - start_fr_count);
    const float rl             = static_cast<float>(motor_rl().count() - start_rl_count);
    const float rr             = static_cast<float>(motor_rr().count() - start_rr_count);
    const float forward_counts = fabsf((fl + fr + rl + rr) * 0.25f);
    traveled_distance_cm       = forward_counts / CHASSIS_ENCODER_COUNTS_PER_CM;
}

bool LineFollower::should_stop_for_event() {
    const int16_t front_cm = sensors.ultrasonic_front_cm();
    if (front_stop_cm > 0 && front_cm > 0 && front_cm <= front_stop_cm) {
        loggf("line front stop front=%d threshold=%d\n", front_cm, front_stop_cm);
        return true;
    }

    const bool left_side_detected  = sensors.ir_side_left_detected();
    const bool right_side_detected = sensors.ir_side_right_detected();

    if (left_side_detected && right_side_detected) {
        if (cross_count < LINE_CROSS_CONFIRM) cross_count++;
    } else {
        cross_count = 0;
    }

    if (left_side_detected) {
        if (left_corner_count < LINE_CORNER_CONFIRM) left_corner_count++;
    } else {
        left_corner_count = 0;
    }

    if (right_side_detected) {
        if (right_corner_count < LINE_CORNER_CONFIRM) right_corner_count++;
    } else {
        right_corner_count = 0;
    }

    if (mode == StopMode::Cross && cross_count >= LINE_CROSS_CONFIRM) {
        loggf("line cross stop left=%u right=%u\n",
        sensors.ir_side_left_value(),
        sensors.ir_side_right_value());
        return true;
    }

    if (mode == StopMode::Distance && target_distance_cm > 0.0f && traveled_distance_cm >= target_distance_cm) {
        loggf("line distance stop traveled=%.1f target=%.1f\n", traveled_distance_cm, target_distance_cm);
        return true;
    }

    if (mode == StopMode::Rfid) {
        const uint32_t uid = rfid.last_uid();
        if (uid != 0 && uid != rfid_ignore_uid && (rfid_any_uid || rfid_not_same || uid == rfid_target_uid)) {
            last_rfid_stop_uid = uid;
            loggf("line rfid stop uid=%lu target=%lu ignore=%lu\n",
            static_cast<unsigned long>(uid),
            static_cast<unsigned long>(rfid_target_uid),
            static_cast<unsigned long>(rfid_ignore_uid));
            return true;
        }
    }

    if (mode == StopMode::LeftCorner && left_corner_count >= LINE_CORNER_CONFIRM) {
        loggf("line left corner stop side=%u\n", sensors.ir_side_left_value());
        return true;
    }

    if (mode == StopMode::RightCorner && right_corner_count >= LINE_CORNER_CONFIRM) {
        loggf("line right corner stop side=%u\n", sensors.ir_side_right_value());
        return true;
    }

    if (mode == StopMode::AnyCorner &&
    (left_corner_count >= LINE_CORNER_CONFIRM || right_corner_count >= LINE_CORNER_CONFIRM)) {
        loggf("line corner stop left=%u/%d right=%u/%d\n",
        sensors.ir_side_left_value(),
        sensors.ir_side_left_detected() ? 1 : 0,
        sensors.ir_side_right_value(),
        sensors.ir_side_right_detected() ? 1 : 0);
        return true;
    }

    return false;
}

void LineFollower::update(float dt_s) {
    if (!active) return;

    update_traveled_distance();

    black_count = count_black_sensors();
    if (black_count <= LINE_NO_LINE_MAX_BLACK) {
        if (no_line_count < LINE_NO_LINE_CONFIRM) no_line_count++;
    } else {
        no_line_count = 0;
    }

    if (mode != StopMode::None && no_line_count >= LINE_NO_LINE_CONFIRM) {
        loggf("line lost pos=%u black=%u\n", sensors.ir_position(), black_count);
        stop();
        return;
    }

    if (should_stop_for_event()) {
        stop();
        return;
    }

    error            = static_cast<float>(static_cast<int32_t>(sensors.ir_position()) - LINE_CENTER_POS);
    const float derr = error - prev_error;
    prev_error       = error;

    const float abs_err_norm = constrain(fabsf(error) / static_cast<float>(LINE_CENTER_POS), 0.0f, 1.0f);
    vx                       = speed * (1.0f - (1.0f - LINE_MIN_SPEED_SCALE) * abs_err_norm);
    w                        = LINE_DIRECTION * (kp_value * error + kd_value * derr);
    w                        = constrain(w, -LINE_MAX_WHEEL_SPEED, LINE_MAX_WHEEL_SPEED);

    chassis.set_target(vx, 0.0f, w);

    const uint32_t now_ms = millis();
    if (last_status_ms == 0 || now_ms - last_status_ms >= LINE_STATUS_INTERVAL_MS) {
        last_status_ms = now_ms;
        print_status();
    }
}

void LineFollower::print_status() const {
    loggf("line active=%d mode=%s speed=%.1f pid=%.3f/%.3f pos=%u err=%.0f black=%u noln=%u cross=%u corner=%u/%u front=%d/%d dist=%.1f/%.1f rfid=%lu/%lu last=%lu vx=%.1f w=%.1f\n",
    active ? 1 : 0,
    stop_mode_name(),
    speed,
    kp_value,
    kd_value,
    sensors.ir_position(),
    error,
    black_count,
    no_line_count,
    cross_count,
    left_corner_count,
    right_corner_count,
    sensors.ultrasonic_front_cm(),
    front_stop_cm,
    traveled_distance_cm,
    target_distance_cm,
    static_cast<unsigned long>(rfid.last_uid()),
    static_cast<unsigned long>(rfid_ignore_uid),
    static_cast<unsigned long>(last_rfid_stop_uid),
    vx,
    w);
}

void line_follower_update(float dt_s) {
    line_follower.update(dt_s);
}

void line_follower_stop() {
    line_follower.stop();
}
