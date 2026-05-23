#include "chassis.hpp"

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
