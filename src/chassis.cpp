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
    target_speed = v;
}

void Motor::update(std::chrono::microseconds dt) {
    if (manual_pwm_enabled) {
        write_pwm(manual_pwm);
        return;
    }

    float dt_s    = dt.count() * 0.000001f;
    int32_t delta = count() - prev_enc_cnt;
    prev_enc_cnt  = count();

    float speed = delta / dt_s;

    constexpr float r = 0.7f;
    current_speed     = current_speed * r + speed * (1 - r);

    float error = target_speed - current_speed;
    prev_error  = error;

    integral = error * dt_s;
    integral = constrain(integral, -1000.f, 1000.f);

    float derivative = (error - prev_error) / dt_s;

    output = kp * (error + ki * integral + kd * derivative);
    output = constrain(output, -255, 255);

    write_pwm(output);
}

void Motor::write_pwm(int pwm) {
    if (abs(pwm) < 50) {
        pwm = 0;
    }

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
    write_pwm(0);
}

void Motor::stop() {
    if (manual_pwm_enabled) {
        write_pwm(manual_pwm);
    } else {
        write_pwm(0);
    }
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
