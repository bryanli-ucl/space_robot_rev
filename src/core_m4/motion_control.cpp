#include "core_m4/motion_control.hpp"

#include "core_m4/chassis.hpp"
#include "core_m4/imu.hpp"
#include "core_m4/rpc_bridge.hpp"
#include "core_m4/state.hpp"

#include <Arduino.h>
#include <mbed.h>

using namespace ::rtos;
using namespace std::chrono_literals;

namespace {

enum class StopMode {
    Front,
    Motor,
    Rfid,
    Cross,
};

constexpr auto CONTROL_INTERVAL = 20ms;

uint32_t ignored_rfid_uid = 0;

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
    const float counts_s = speed_cm_s * CONFIG::M4::COUNTS_PER_CM;
    return counts_s * CONFIG::M4::CHASSIS_R;
}

float motor_distance_cm(int32_t fl_start, int32_t fr_start, int32_t rl_start, int32_t rr_start) {
    const float fl_delta = fabsf(static_cast<float>(m4_motor_fl().count() - fl_start));
    const float fr_delta = fabsf(static_cast<float>(m4_motor_fr().count() - fr_start));
    const float rl_delta = fabsf(static_cast<float>(m4_motor_rl().count() - rl_start));
    const float rr_delta = fabsf(static_cast<float>(m4_motor_rr().count() - rr_start));
    return (fl_delta + fr_delta + rl_delta + rr_delta) * 0.25f / CONFIG::M4::COUNTS_PER_CM;
}

bool front_obstacle(float front_stop_cm) {
    const int front_cm = rpc_bridge_ultrasonic_front_cm();
    return front_cm > 0 && static_cast<float>(front_cm) <= front_stop_cm;
}

bool rfid_matched(const RfidStop& rfid_stop) {
    const uint32_t uid = static_cast<uint32_t>(rpc_bridge_rfid_uid());
    if (!rfid_stop.enabled || uid == 0 || uid == ignored_rfid_uid) {
        return false;
    }

    return rfid_stop.any || uid == rfid_stop.uid;
}

uint8_t black_line_count() {
    uint8_t count = 0;
    for (uint8_t i = 0; i < 9; i++) {
        if (rpc_bridge_ir_raw(i) >= CONFIG::M4::LINE_BLACK_THRESHOLD) {
            count++;
        }
    }
    return count;
}

bool cross_line_detected(uint8_t* confirm_count) {
    if (black_line_count() >= CONFIG::M4::LINE_CROSS_MIN_BLACK) {
        if (*confirm_count < CONFIG::M4::LINE_CROSS_CONFIRM) {
            (*confirm_count)++;
        }
    } else {
        *confirm_count = 0;
    }

    return *confirm_count >= CONFIG::M4::LINE_CROSS_CONFIRM;
}

void clear_manual_pwm_all() {
    m4_motor_fl().clear_manual_pwm();
    m4_motor_fr().clear_manual_pwm();
    m4_motor_rl().clear_manual_pwm();
    m4_motor_rr().clear_manual_pwm();
}

void prepare_blocking_motion(const RfidStop& rfid_stop) {
    motion_state = MotionState::IDLE;

    Chassis& chassis = m4_chassis();
    chassis.enable();
    chassis.clear_position_control();
    chassis.set_target(0.0f, 0.0f, 0.0f);
    clear_manual_pwm_all();

    ignored_rfid_uid = rfid_stop.enabled ? static_cast<uint32_t>(rpc_bridge_rfid_uid()) : 0;
}

void wait_control_interval() {
    ThisThread::sleep_for(CONTROL_INTERVAL);
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

    const int32_t fl_start = m4_motor_fl().count();
    const int32_t fr_start = m4_motor_fr().count();
    const int32_t rl_start = m4_motor_rl().count();
    const int32_t rr_start = m4_motor_rr().count();

    uint8_t cross_confirm = 0;
    int16_t last_err = 0;
    const float vx = cm_s_to_chassis_vx(fabsf(speed_cm_s));

    while (true) {
        const MotionResult common = check_common_stop(front_stop_cm, rfid_stop);
        if (common != MotionResult::Timeout) {
            return stop_and_return(common);
        }

        if (mode == StopMode::Motor && motor_distance_cm(fl_start, fr_start, rl_start, rr_start) >= fabsf(distance_cm)) {
            return stop_and_return(MotionResult::DistanceReached);
        }

        if (mode == StopMode::Cross && cross_line_detected(&cross_confirm)) {
            return stop_and_return(MotionResult::CrossLineDetected);
        }

        const int16_t err = static_cast<int16_t>(rpc_bridge_ir_pos()) - 4000;
        const int16_t derr = err - last_err;
        const float abs_err_norm = constrain(fabsf(static_cast<float>(err)) / 4000.0f, 0.0f, 1.0f);
        const float vx_cmd = vx * (1.0f - 0.55f * abs_err_norm);
        float w = CONFIG::M4::LINE_KP * err + CONFIG::M4::LINE_KD * derr;
        w = constrain(w, -CONFIG::M4::LINE_MAX_W, CONFIG::M4::LINE_MAX_W);
        last_err = err;

        m4_chassis().set_target(vx_cmd, 0.0f, w);
        wait_control_interval();
    }
}

MotionResult run_wall_follow(StopMode mode,
                             WallSide wall_side,
                             float wall_dist_cm,
                             float distance_cm,
                             float speed_cm_s,
                             float front_stop_cm,
                             const RfidStop& rfid_stop) {
    prepare_blocking_motion(rfid_stop);

    const int32_t fl_start = m4_motor_fl().count();
    const int32_t fr_start = m4_motor_fr().count();
    const int32_t rl_start = m4_motor_rl().count();
    const int32_t rr_start = m4_motor_rr().count();

    const int8_t side = wall_side == WallSide::Right ? 1 : -1;
    const float yaw_ref = imu_yaw_deg();
    const bool yaw_ref_ready = imu_yaw_ready();
    float last_dist_err = 0.0f;
    const float vx = cm_s_to_chassis_vx(fabsf(speed_cm_s));

    while (true) {
        const MotionResult common = check_common_stop(front_stop_cm, rfid_stop);
        if (common != MotionResult::Timeout) {
            return stop_and_return(common);
        }

        if (mode == StopMode::Motor && motor_distance_cm(fl_start, fr_start, rl_start, rr_start) >= fabsf(distance_cm)) {
            return stop_and_return(MotionResult::DistanceReached);
        }

        const int side_dist = side > 0 ? rpc_bridge_ultrasonic_right_cm() : rpc_bridge_ultrasonic_left_cm();
        const bool side_valid = side_dist > 0 && side_dist < 300;

        float vx_cmd = vx;
        float w = 0.0f;
        if (!side_valid) {
            vx_cmd = vx * 0.5f;
            w = side * CONFIG::M4::WALL_MAX_W * 0.35f;
        } else {
            const float dist_err = static_cast<float>(side_dist) - wall_dist_cm;
            const float dist_der = dist_err - last_dist_err;
            const float w_dist = -side * (CONFIG::M4::WALL_KP * dist_err + CONFIG::M4::WALL_KD * dist_der);
            float w_yaw = 0.0f;

            if (yaw_ref_ready && imu_yaw_ready()) {
                w_yaw = CONFIG::M4::WALL_YAW_KP * wrap_deg(yaw_ref - imu_yaw_deg());
            }

            w = constrain(w_dist + w_yaw, -CONFIG::M4::WALL_MAX_W, CONFIG::M4::WALL_MAX_W);
            last_dist_err = dist_err;
        }

        m4_chassis().set_target(vx_cmd, 0.0f, w);
        wait_control_interval();
    }
}

MotionResult run_drive(StopMode mode, float distance_cm, float speed_cm_s, float front_stop_cm, const RfidStop& rfid_stop) {
    prepare_blocking_motion(rfid_stop);

    const int32_t fl_start = m4_motor_fl().count();
    const int32_t fr_start = m4_motor_fr().count();
    const int32_t rl_start = m4_motor_rl().count();
    const int32_t rr_start = m4_motor_rr().count();

    const float direction_ref = mode == StopMode::Motor ? distance_cm : speed_cm_s;
    const float direction = direction_ref < 0.0f ? -1.0f : 1.0f;
    const float vx = direction * cm_s_to_chassis_vx(fabsf(speed_cm_s));
    const float yaw_ref = imu_yaw_deg();
    const bool yaw_ref_ready = imu_yaw_ready();

    while (true) {
        const MotionResult common = check_common_stop(front_stop_cm, rfid_stop);
        if (common != MotionResult::Timeout) {
            return stop_and_return(common);
        }

        if (mode == StopMode::Motor && motor_distance_cm(fl_start, fr_start, rl_start, rr_start) >= fabsf(distance_cm)) {
            return stop_and_return(MotionResult::DistanceReached);
        }

        float w = 0.0f;
        if (yaw_ref_ready && imu_yaw_ready()) {
            w = constrain(0.02f * wrap_deg(yaw_ref - imu_yaw_deg()), -0.8f, 0.8f);
        }

        m4_chassis().set_target(vx, 0.0f, w);
        wait_control_interval();
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
    Chassis& chassis = m4_chassis();
    chassis.clear_position_control();
    chassis.set_target(0.0f, 0.0f, 0.0f);
    clear_manual_pwm_all();

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
    if (!imu_yaw_ready()) {
        motion_force_stop(false);
        return MotionResult::SensorInvalid;
    }

    prepare_blocking_motion(RfidStop::none());

    const float target_yaw = wrap_deg(imu_yaw_deg() + delta_deg);
    float integral = 0.0f;
    float last_err = wrap_deg(target_yaw - imu_yaw_deg());
    uint8_t stable_count = 0;
    const uint32_t start_ms = millis();
    uint32_t last_ms = start_ms;

    while (millis() - start_ms < timeout_ms) {
        if (running_state == RunningState::STOPPED) {
            return stop_and_return(MotionResult::StopButton);
        }

        if (!imu_yaw_ready()) {
            return stop_and_return(MotionResult::SensorInvalid);
        }

        const uint32_t now_ms = millis();
        float dt_s = (now_ms - last_ms) * 0.001f;
        if (dt_s < 0.001f) {
            dt_s = 0.001f;
        }
        last_ms = now_ms;

        const float err = wrap_deg(target_yaw - imu_yaw_deg());
        integral += err * dt_s;
        integral = constrain(integral, -80.0f, 80.0f);

        const float derivative = (err - last_err) / dt_s;
        last_err = err;

        if (fabsf(err) <= tolerance_deg) {
            if (++stable_count >= CONFIG::M4::TURN_STABLE_COUNT) {
                return stop_and_return(MotionResult::AngleReached);
            }
        } else {
            stable_count = 0;
        }

        float w = CONFIG::M4::TURN_KP * err + CONFIG::M4::TURN_KI * integral + CONFIG::M4::TURN_KD * derivative;
        w = constrain(w, -fabsf(max_w), fabsf(max_w));

        m4_chassis().set_target(0.0f, 0.0f, w);
        wait_control_interval();
    }

    return stop_and_return(MotionResult::Timeout);
}
