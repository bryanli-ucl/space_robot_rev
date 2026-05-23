#include "motion_control.hpp"

namespace {

enum class StopMode {
    Front,
    Motor,
    Rfid,
    Cross,
};

float wrap_deg(float angle) {
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

float cm_s_to_chassis_vx(float speed_cm_s) {
    float counts_s = speed_cm_s * CONFIG::COUNTS_PER_CM;
    return counts_s * CONFIG::CHASSIS_R;
}

float motor_distance_cm(int32_t fl_start, int32_t fr_start, int32_t rl_start, int32_t rr_start) {
    float fl_delta = fabsf(static_cast<float>(mfl.count() - fl_start));
    float fr_delta = fabsf(static_cast<float>(mfr.count() - fr_start));
    float rl_delta = fabsf(static_cast<float>(mrl.count() - rl_start));
    float rr_delta = fabsf(static_cast<float>(mrr.count() - rr_start));
    return (fl_delta + fr_delta + rl_delta + rr_delta) * 0.25f / CONFIG::COUNTS_PER_CM;
}

bool front_obstacle(float front_stop_cm) {
    return dist_front > 0 && static_cast<float>(dist_front) <= front_stop_cm;
}

bool rfid_matched(const RfidStop& rfid_stop) {
    if (!rfid_stop.enabled || detected_uid == 0) {
        return false;
    }

    return rfid_stop.any || detected_uid == rfid_stop.uid;
}

uint8_t black_line_count() {
    uint8_t count = 0;
    for (uint8_t i = 0; i < 9; i++) {
        if (ir_vals[i] >= CONFIG::LINE_BLACK_THRESHOLD) {
            count++;
        }
    }
    return count;
}

bool cross_line_detected(uint8_t* confirm_count) {
    if (black_line_count() >= CONFIG::LINE_CROSS_MIN_BLACK) {
        if (*confirm_count < CONFIG::LINE_CROSS_CONFIRM) {
            (*confirm_count)++;
        }
    } else {
        *confirm_count = 0;
    }

    return *confirm_count >= CONFIG::LINE_CROSS_CONFIRM;
}

void prepare_blocking_motion(const RfidStop& rfid_stop) {
    motion_state = MotionState::IDLE;
    chassis.enable();
    chassis.clear_position_control();
    chassis.set_target(0.0f, 0.0f, 0.0f);
    mfl.clear_manual_pwm();
    mfr.clear_manual_pwm();
    mrl.clear_manual_pwm();
    mrr.clear_manual_pwm();

    if (rfid_stop.enabled) {
        detected_uid = 0;
    }
}

MotionResult stop_and_return(MotionResult result) {
    motion_force_stop(false);
    return result;
}

MotionResult check_common_stop(float front_stop_cm, const RfidStop& rfid_stop) {
    if (running_state == RunningState::STOPPED) {
        return MotionResult::StopButton;
    }

    if (front_obstacle(front_stop_cm)) {
        return MotionResult::FrontObstacle;
    }

    if (rfid_matched(rfid_stop)) {
        return MotionResult::RfidDetected;
    }

    return MotionResult::Timeout;
}

MotionResult run_line_follow(StopMode mode, float distance_cm, float speed_cm_s, float front_stop_cm, const RfidStop& rfid_stop) {
    prepare_blocking_motion(rfid_stop);

    int32_t fl_start = mfl.count();
    int32_t fr_start = mfr.count();
    int32_t rl_start = mrl.count();
    int32_t rr_start = mrr.count();

    uint8_t cross_confirm = 0;
    int16_t last_err      = 0;
    float vx              = cm_s_to_chassis_vx(fabsf(speed_cm_s));

    while (1) {
        MotionResult common = check_common_stop(front_stop_cm, rfid_stop);
        if (common != MotionResult::Timeout) {
            return stop_and_return(common);
        }

        if (mode == StopMode::Motor && motor_distance_cm(fl_start, fr_start, rl_start, rr_start) >= fabsf(distance_cm)) {
            return stop_and_return(MotionResult::DistanceReached);
        }

        if (mode == StopMode::Cross && cross_line_detected(&cross_confirm)) {
            return stop_and_return(MotionResult::CrossLineDetected);
        }

        int16_t err        = static_cast<int16_t>(ir_pos) - 4000;
        int16_t derr       = err - last_err;
        float abs_err_norm = constrain(fabsf(static_cast<float>(err)) / 4000.0f, 0.0f, 1.0f);

        float vx_cmd = vx * (1.0f - 0.55f * abs_err_norm);
        float w      = CONFIG::LINE_KP * err + CONFIG::LINE_KD * derr;
        w            = constrain(w, -CONFIG::LINE_MAX_W, CONFIG::LINE_MAX_W);
        last_err     = err;

        chassis.set_target(vx_cmd, 0.0f, w);
        ThisThread::sleep_for(20ms);
    }
}

MotionResult run_wall_follow(
StopMode mode,
WallSide wall_side,
float wall_dist_cm,
float distance_cm,
float speed_cm_s,
float front_stop_cm,
const RfidStop& rfid_stop) {
    prepare_blocking_motion(rfid_stop);

    int32_t fl_start = mfl.count();
    int32_t fr_start = mfr.count();
    int32_t rl_start = mrl.count();
    int32_t rr_start = mrr.count();

    int8_t side         = wall_side == WallSide::Right ? 1 : -1;
    float yaw_ref       = imu_yaw_deg;
    bool yaw_ready      = imu_yaw_ready;
    float last_dist_err = 0.0f;
    float vx            = cm_s_to_chassis_vx(fabsf(speed_cm_s));

    while (1) {
        MotionResult common = check_common_stop(front_stop_cm, rfid_stop);
        if (common != MotionResult::Timeout) {
            return stop_and_return(common);
        }

        if (mode == StopMode::Motor && motor_distance_cm(fl_start, fr_start, rl_start, rr_start) >= fabsf(distance_cm)) {
            return stop_and_return(MotionResult::DistanceReached);
        }

        int16_t side_dist = side > 0 ? dist_right : dist_left;
        bool side_valid   = side_dist > 0 && side_dist < 300;

        float vx_cmd = vx;
        float w      = 0.0f;
        if (!side_valid) {
            vx_cmd = vx * 0.5f;
            w      = side * CONFIG::WALL_MAX_W * 0.35f;
        } else {
            float dist_err = static_cast<float>(side_dist) - wall_dist_cm;
            float dist_der = dist_err - last_dist_err;
            float w_dist   = -side * (CONFIG::WALL_KP * dist_err + CONFIG::WALL_KD * dist_der);
            float w_yaw    = 0.0f;

            if (yaw_ready && imu_yaw_ready) {
                w_yaw = CONFIG::WALL_YAW_KP * wrap_deg(yaw_ref - imu_yaw_deg);
            }

            w             = constrain(w_dist + w_yaw, -CONFIG::WALL_MAX_W, CONFIG::WALL_MAX_W);
            last_dist_err = dist_err;
        }

        chassis.set_target(vx_cmd, 0.0f, w);
        ThisThread::sleep_for(20ms);
    }
}

MotionResult run_drive(
StopMode mode,
float distance_cm,
float speed_cm_s,
float front_stop_cm,
const RfidStop& rfid_stop) {
    prepare_blocking_motion(rfid_stop);

    int32_t fl_start = mfl.count();
    int32_t fr_start = mfr.count();
    int32_t rl_start = mrl.count();
    int32_t rr_start = mrr.count();

    float direction_ref = mode == StopMode::Motor ? distance_cm : speed_cm_s;
    float direction     = direction_ref < 0.0f ? -1.0f : 1.0f;
    float vx            = direction * cm_s_to_chassis_vx(fabsf(speed_cm_s));
    float yaw_ref       = imu_yaw_deg;
    bool yaw_ready      = imu_yaw_ready;

    while (1) {
        MotionResult common = check_common_stop(front_stop_cm, rfid_stop);
        if (common != MotionResult::Timeout) {
            return stop_and_return(common);
        }

        if (mode == StopMode::Motor && motor_distance_cm(fl_start, fr_start, rl_start, rr_start) >= fabsf(distance_cm)) {
            return stop_and_return(MotionResult::DistanceReached);
        }

        float w = 0.0f;
        if (yaw_ready && imu_yaw_ready) {
            w = constrain(0.02f * wrap_deg(yaw_ref - imu_yaw_deg), -0.8f, 0.8f);
        }

        chassis.set_target(vx, 0.0f, w);
        ThisThread::sleep_for(20ms);
    }
}

} // namespace

const char* motion_result_name(MotionResult result) {
    switch (result) {
    case MotionResult::DistanceReached: return "distance";
    case MotionResult::FrontObstacle: return "front";
    case MotionResult::RfidDetected: return "rfid";
    case MotionResult::CrossLineDetected: return "cross";
    case MotionResult::AngleReached: return "angle";
    case MotionResult::StopButton: return "stop";
    case MotionResult::SensorInvalid: return "sensor-invalid";
    case MotionResult::Timeout: return "timeout";
    }

    return "unknown";
}

void motion_force_stop(bool disable_chassis) {
    chassis.clear_position_control();
    chassis.set_target(0.0f, 0.0f, 0.0f);
    mfl.clear_manual_pwm();
    mfr.clear_manual_pwm();
    mrl.clear_manual_pwm();
    mrr.clear_manual_pwm();

    if (disable_chassis) {
        chassis.disable();
    }
}

MotionResult run_line_follow_until_front_cm(float front_stop_cm, float speed_cm_s) {
    return run_line_follow(StopMode::Front, 0.0f, speed_cm_s, front_stop_cm, RfidStop::none());
}

MotionResult run_line_follow_until_motor_cm(float distance_cm, float speed_cm_s, float front_stop_cm) {
    return run_line_follow(StopMode::Motor, distance_cm, speed_cm_s, front_stop_cm, RfidStop::none());
}

MotionResult run_line_follow_until_rfid(float speed_cm_s, float front_stop_cm) {
    return run_line_follow(StopMode::Rfid, 0.0f, speed_cm_s, front_stop_cm, RfidStop::any_uid());
}

MotionResult run_line_follow_until_rfid_uid(uint32_t uid, float speed_cm_s, float front_stop_cm) {
    return run_line_follow(StopMode::Rfid, 0.0f, speed_cm_s, front_stop_cm, RfidStop::uid_match(uid));
}

MotionResult run_line_follow_until_cross(float speed_cm_s, float front_stop_cm) {
    return run_line_follow(StopMode::Cross, 0.0f, speed_cm_s, front_stop_cm, RfidStop::none());
}

MotionResult run_wall_follow_until_front_cm(WallSide side, float wall_dist_cm, float front_stop_cm, float speed_cm_s) {
    return run_wall_follow(StopMode::Front, side, wall_dist_cm, 0.0f, speed_cm_s, front_stop_cm, RfidStop::none());
}

MotionResult run_wall_follow_until_motor_cm(WallSide side, float wall_dist_cm, float distance_cm, float speed_cm_s, float front_stop_cm) {
    return run_wall_follow(StopMode::Motor, side, wall_dist_cm, distance_cm, speed_cm_s, front_stop_cm, RfidStop::none());
}

MotionResult run_wall_follow_until_rfid(WallSide side, float wall_dist_cm, float speed_cm_s, float front_stop_cm) {
    return run_wall_follow(StopMode::Rfid, side, wall_dist_cm, 0.0f, speed_cm_s, front_stop_cm, RfidStop::any_uid());
}

MotionResult run_wall_follow_until_rfid_uid(WallSide side, float wall_dist_cm, uint32_t uid, float speed_cm_s, float front_stop_cm) {
    return run_wall_follow(StopMode::Rfid, side, wall_dist_cm, 0.0f, speed_cm_s, front_stop_cm, RfidStop::uid_match(uid));
}

MotionResult run_drive_until_front_cm(float front_stop_cm, float speed_cm_s) {
    return run_drive(StopMode::Front, 0.0f, speed_cm_s, front_stop_cm, RfidStop::none());
}

MotionResult run_drive_until_motor_cm(float distance_cm, float speed_cm_s, float front_stop_cm) {
    return run_drive(StopMode::Motor, distance_cm, speed_cm_s, front_stop_cm, RfidStop::none());
}

MotionResult run_drive_until_rfid(float speed_cm_s, float front_stop_cm) {
    return run_drive(StopMode::Rfid, 0.0f, speed_cm_s, front_stop_cm, RfidStop::any_uid());
}

MotionResult run_drive_until_rfid_uid(uint32_t uid, float speed_cm_s, float front_stop_cm) {
    return run_drive(StopMode::Rfid, 0.0f, speed_cm_s, front_stop_cm, RfidStop::uid_match(uid));
}

MotionResult run_turn_deg(float delta_deg, float max_w, float tolerance_deg, uint32_t timeout_ms) {
    if (!imu_yaw_ready) {
        motion_force_stop(false);
        return MotionResult::SensorInvalid;
    }

    prepare_blocking_motion(RfidStop::none());

    float target_yaw       = wrap_deg(imu_yaw_deg + delta_deg);
    float integral         = 0.0f;
    float last_err         = wrap_deg(target_yaw - imu_yaw_deg);
    uint8_t stable_count   = 0;
    unsigned long start_ms = millis();
    unsigned long last_ms  = start_ms;

    while (millis() - start_ms < timeout_ms) {
        if (running_state == RunningState::STOPPED) {
            return stop_and_return(MotionResult::StopButton);
        }

        if (!imu_yaw_ready) {
            return stop_and_return(MotionResult::SensorInvalid);
        }

        unsigned long now_ms = millis();
        float dt_s           = (now_ms - last_ms) * 0.001f;
        if (dt_s < 0.001f) {
            dt_s = 0.001f;
        }
        last_ms = now_ms;

        float err = wrap_deg(target_yaw - imu_yaw_deg);
        integral += err * dt_s;
        integral = constrain(integral, -80.0f, 80.0f);

        float derivative = (err - last_err) / dt_s;
        last_err         = err;

        if (fabsf(err) <= tolerance_deg) {
            if (++stable_count >= CONFIG::TURN_STABLE_COUNT) {
                return stop_and_return(MotionResult::AngleReached);
            }
        } else {
            stable_count = 0;
        }

        float w = CONFIG::TURN_KP * err + CONFIG::TURN_KI * integral + CONFIG::TURN_KD * derivative;
        w       = constrain(w, -fabsf(max_w), fabsf(max_w));

        chassis.set_target(0.0f, 0.0f, w);
        ThisThread::sleep_for(20ms);
    }

    return stop_and_return(MotionResult::Timeout);
}
