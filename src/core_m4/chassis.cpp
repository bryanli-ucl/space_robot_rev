#include "core_m4/chassis.hpp"

#include "core_m4/imu.hpp"
#include "core_m4/serial.hpp"
#include "core_m4/state.hpp"

using namespace ::rtos;

extern Thread task_chassis;

namespace {

Motor motor_fl(CONFIG::M4::MOTOR_FL_EN,
               CONFIG::M4::MOTOR_FL_FORWARD,
               CONFIG::M4::MOTOR_FL_BACKWARD,
               CONFIG::M4::MOTOR_FL_ENC_A,
               CONFIG::M4::MOTOR_FL_ENC_B);
Motor motor_fr(CONFIG::M4::MOTOR_FR_EN,
               CONFIG::M4::MOTOR_FR_FORWARD,
               CONFIG::M4::MOTOR_FR_BACKWARD,
               CONFIG::M4::MOTOR_FR_ENC_A,
               CONFIG::M4::MOTOR_FR_ENC_B);
Motor motor_rl(CONFIG::M4::MOTOR_RL_EN,
               CONFIG::M4::MOTOR_RL_FORWARD,
               CONFIG::M4::MOTOR_RL_BACKWARD,
               CONFIG::M4::MOTOR_RL_ENC_A,
               CONFIG::M4::MOTOR_RL_ENC_B);
Motor motor_rr(CONFIG::M4::MOTOR_RR_EN,
               CONFIG::M4::MOTOR_RR_FORWARD,
               CONFIG::M4::MOTOR_RR_BACKWARD,
               CONFIG::M4::MOTOR_RR_ENC_A,
               CONFIG::M4::MOTOR_RR_ENC_B);
Chassis chassis(motor_fl, motor_fr, motor_rl, motor_rr);
bool chassis_task_started = false;

constexpr auto CHASSIS_DT = std::chrono::microseconds(20000);

float wrap_deg(float angle) {
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

void chassis_task_entry() {
    loggf("[m4-chassis] task ready: L=%.2f W=%.2f R=%.3f\n",
    CONFIG::M4::CHASSIS_L,
    CONFIG::M4::CHASSIS_W,
    CONFIG::M4::CHASSIS_R);

    float yaw_ref = 0.0f;
    bool yaw_ref_ready = false;
    uint32_t last_status_ms = 0;

    while (true) {
        const float vx = chassis.get_target_vx();
        const float vy = chassis.get_target_vy();
        const float w = chassis.get_target_w();

        if (imu_yaw_ready()) {
            const float yaw = imu_yaw_deg();
            const bool moving_xy = fabsf(vx) > CONFIG::M4::CHASSIS_MOVE_EPS ||
                                   fabsf(vy) > CONFIG::M4::CHASSIS_MOVE_EPS;
            const bool rotating = fabsf(w) > CONFIG::M4::CHASSIS_ROTATE_EPS;

            if (!yaw_ref_ready) {
                yaw_ref = yaw;
                yaw_ref_ready = true;
            }

            if (running_state == RunningState::IDLE && moving_xy && !rotating) {
                const float yaw_err = wrap_deg(yaw_ref - yaw);
                const float w_corr = constrain(CONFIG::M4::CHASSIS_YAW_KP * yaw_err,
                                               -CONFIG::M4::CHASSIS_YAW_MAX_W_CORR,
                                               CONFIG::M4::CHASSIS_YAW_MAX_W_CORR);
                chassis.apply_target(vx, vy, w + w_corr);
            } else {
                yaw_ref = yaw;
                chassis.apply_target(vx, vy, w);
            }
        } else {
            yaw_ref_ready = false;
            chassis.apply_target(vx, vy, w);
        }

        chassis.update(CHASSIS_DT);

        const uint32_t now_ms = millis();
        if (last_status_ms == 0 || now_ms - last_status_ms >= CONFIG::M4::CHASSIS_STATUS_INTERVAL_MS) {
            last_status_ms = now_ms;
            loggf("[m4-chassis] state=%s target=%.2f/%.2f/%.2f wheel=%.2f/%.2f/%.2f/%.2f pwm=%d/%d/%d/%d\n",
            running_state_name(running_state),
            vx,
            vy,
            w,
            chassis.get_vfl(),
            chassis.get_vfr(),
            chassis.get_vrl(),
            chassis.get_vrr(),
            motor_fl.get_applied_pwm(),
            motor_fr.get_applied_pwm(),
            motor_rl.get_applied_pwm(),
            motor_rr.get_applied_pwm());
        }

        ThisThread::sleep_for(std::chrono::milliseconds(CONFIG::M4::CHASSIS_TASK_INTERVAL_MS));
    }
}

} // namespace

Motor::Motor(pin_size_t en, pin_size_t forward, pin_size_t backward) : en(en), forward(forward), backward(backward) {
    pinMode(en, OUTPUT);
    pinMode(forward, OUTPUT);
    pinMode(backward, OUTPUT);
}

Motor::Motor(pin_size_t en, pin_size_t forward, pin_size_t backward, pin_size_t enc_a, pin_size_t enc_b) : en(en), forward(forward), backward(backward), enc_a(enc_a), enc_b(enc_b) {
    pinMode(en, OUTPUT);
    pinMode(forward, OUTPUT);
    pinMode(backward, OUTPUT);
    pinMode(enc_a, INPUT_PULLUP);
    pinMode(enc_b, INPUT_PULLUP);

    gpio_init_in_ex(&enc_a_hal, digitalPinToPinName(enc_a), PullUp);
    gpio_init_in_ex(&enc_b_hal, digitalPinToPinName(enc_b), PullUp);

    reset_count();

    attachInterruptParam(digitalPinToInterrupt(enc_a), &Motor::ISR, CHANGE, reinterpret_cast<void*>(this));
}

Motor::~Motor() {
    detachInterrupt(digitalPinToInterrupt(enc_a));
}

int32_t Motor::count() const {
    return enc_cnt;
}

void Motor::reset_count() {
    enc_cnt      = 0;
    prev_enc_cnt = 0;
}

void Motor::set_speed(float v) {
    if (speed_override_enabled || position_control_enabled) {
        return;
    }

    target_speed = v;
}

void Motor::set_position_delta(int32_t delta_count, float max_speed) {
    set_position_target(count() + delta_count, max_speed);
}

void Motor::set_position_target(int32_t target_count, float max_speed) {
    manual_pwm_enabled       = false;
    speed_override_enabled   = false;
    position_control_enabled = true;
    position_target_count    = target_count;
    position_max_speed       = fmaxf(fabsf(max_speed), 1.0f);
    reset_controller();
}

void Motor::clear_position_control() {
    position_control_enabled = false;
    position_max_speed       = 0.0f;
    target_speed             = 0.0f;
    reset_controller();
    write_pwm(0);
}

void Motor::set_test_speed(float v) {
    manual_pwm_enabled    = false;
    speed_override_enabled = true;
    target_speed          = v;
    reset_controller();
}

void Motor::clear_speed_override() {
    speed_override_enabled = false;
    reset_controller();
}

void Motor::update(std::chrono::microseconds dt) {
    if (manual_pwm_enabled) {
        write_pwm(manual_pwm);
        return;
    }

    float dt_s    = dt.count() * 0.000001f;
    int32_t enc_now = count();
    int32_t delta   = enc_now - prev_enc_cnt;
    prev_enc_cnt    = enc_now;
    last_delta       = delta;
    speed_sample_delta += delta;
    speed_sample_dt += dt_s;

    if (position_control_enabled) {
        int32_t position_error = position_target_count - enc_now;

        if (abs(position_error) <= 5) {
            position_control_enabled = false;
            target_speed             = 0.0f;
            current_speed            = 0.0f;
            speed_sample_delta       = 0;
            speed_sample_dt          = 0.0f;
            reset_controller();
            write_pwm(0);
            return;
        }

        target_speed = constrain(position_error * position_kp, -position_max_speed, position_max_speed);
    }

    if (fabsf(target_speed) < 0.001f) {
        current_speed = 0.0f;
        last_error    = 0.0f;
        derivative    = 0.0f;
        speed_sample_delta = 0;
        speed_sample_dt    = 0.0f;
        reset_controller();
        write_pwm(0);
        return;
    }

    constexpr float sample_period_s = 0.02f;
    if (speed_sample_dt < sample_period_s) {
        return;
    }

    float sample_dt = speed_sample_dt;
    raw_speed = speed_sample_delta / sample_dt;
    speed_sample_delta = 0;
    speed_sample_dt    = 0.0f;

    constexpr float r = 0.6f;
    current_speed     = current_speed * r + raw_speed * (1.0f - r);

    float error = target_speed - current_speed;
    last_error  = error;

    integral += error * sample_dt;
    integral = constrain(integral, -300.0f, 300.0f);

    derivative = (error - prev_error) / sample_dt;
    prev_error = error;

    output = kp * error + ki * integral + kd * derivative;
    output = constrain(output, -255, 255);

    write_pwm(output);
}

void Motor::write_pwm(int pwm) {
    if (abs(pwm) < 50) {
        pwm = 0;
    }

    applied_pwm = pwm;

    if (pwm == 0) {
        digitalWrite(forward, LOW);
        digitalWrite(backward, LOW);
    } else if (pwm > 0) {
        digitalWrite(forward, HIGH);
        digitalWrite(backward, LOW);
    } else {
        digitalWrite(forward, LOW);
        digitalWrite(backward, HIGH);
        pwm = -pwm;
    }

    analogWrite(en, pwm);
}

void Motor::set_manual_pwm(int pwm) {
    manual_pwm_enabled = true;
    manual_pwm         = constrain(pwm, -255, 255);
    write_pwm(manual_pwm);
}

void Motor::clear_manual_pwm() {
    manual_pwm_enabled = false;
    clear_speed_override();
    write_pwm(0);
}

void Motor::stop() {
    if (manual_pwm_enabled) {
        write_pwm(manual_pwm);
    } else {
        write_pwm(0);
    }
}

void Motor::set_pid(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
    reset_controller();
}

void Motor::reset_controller() {
    integral   = 0.0f;
    prev_error = 0.0f;
    last_error = 0.0f;
    derivative = 0.0f;
    output     = 0.0f;
}

bool Motor::position_reached(int32_t tolerance) const {
    return !position_control_enabled || abs(position_target_count - count()) <= tolerance;
}

void Motor::ISR(void* ins_ptr) {
    auto ins = reinterpret_cast<Motor*>(ins_ptr);
    if (gpio_read(&ins->enc_b_hal) == gpio_read(&ins->enc_a_hal)) {
        ins->enc_cnt++;
    } else {
        ins->enc_cnt--;
    }
}

Chassis::Chassis(Motor& fl, Motor& fr, Motor& rl, Motor& rr) : fl(fl), fr(fr), rl(rl), rr(rr) {
    fl.reset_count();
    fr.reset_count();
    rl.reset_count();
    rr.reset_count();
}

void Chassis::set_target(float vx, float vy, float w) {
    target_vx = vx;
    target_vy = vy;
    target_w  = w;

    apply_target(vx, vy, w);
}

void Chassis::apply_target(float vx, float vy, float w) {
    vfl = (vx - vy - (L + W) * w) / R;
    vfr = (vx + vy + (L + W) * w) / R;
    vrl = (vx + vy - (L + W) * w) / R;
    vrr = (vx - vy + (L + W) * w) / R;

    auto max    = fmaxf(fmaxf(fabsf(vfl), fabsf(vfr)), fmaxf(fabsf(vrl), fabsf(vrr)));
    float limit = 255.f;

    if (max > 255) {
        vfl *= limit / max;
        vfr *= limit / max;
        vrl *= limit / max;
        vrr *= limit / max;
    }

    fl.set_speed(vfl);
    fr.set_speed(vfr);
    rl.set_speed(vrl);
    rr.set_speed(vrr);
}

void Chassis::set_paras(float l, float w, float r) {
    L = l;
    W = w;
    R = r;
}

void Chassis::move_counts(int32_t counts, float max_speed) {
    target_vx = 0.0f;
    target_vy = 0.0f;
    target_w  = 0.0f;

    vfl = 0.0f;
    vfr = 0.0f;
    vrl = 0.0f;
    vrr = 0.0f;

    fl.set_position_delta(counts, max_speed);
    fr.set_position_delta(counts, max_speed);
    rl.set_position_delta(counts, max_speed);
    rr.set_position_delta(counts, max_speed);
}

void Chassis::clear_position_control() {
    fl.clear_position_control();
    fr.clear_position_control();
    rl.clear_position_control();
    rr.clear_position_control();
}

bool Chassis::position_reached(int32_t tolerance) const {
    return fl.position_reached(tolerance) &&
           fr.position_reached(tolerance) &&
           rl.position_reached(tolerance) &&
           rr.position_reached(tolerance);
}

void Chassis::update(std::chrono::microseconds dt) {
    if (!is_enable) {
        fl.stop();
        fr.stop();
        rl.stop();
        rr.stop();
        return;
    }

    fl.update(dt);
    fr.update(dt);
    rl.update(dt);
    rr.update(dt);
}

Motor& m4_motor_fl() {
    return motor_fl;
}

Motor& m4_motor_fr() {
    return motor_fr;
}

Motor& m4_motor_rl() {
    return motor_rl;
}

Motor& m4_motor_rr() {
    return motor_rr;
}

Chassis& m4_chassis() {
    return chassis;
}

void m4_chassis_begin() {
    chassis.set_paras(CONFIG::M4::CHASSIS_L, CONFIG::M4::CHASSIS_W, CONFIG::M4::CHASSIS_R);
    chassis.set_target(0.0f, 0.0f, 0.0f);
    chassis.disable();
}

void m4_chassis_task_begin() {
    if (chassis_task_started || !CONFIG::M4::ENABLE_CHASSIS_TASK) {
        return;
    }

    const osStatus status = task_chassis.start(chassis_task_entry);
    chassis_task_started = status == osOK;
    if (!chassis_task_started) {
        loggf("[m4-chassis] task start failed status=%d\n", status);
    }
}
