#include "motor.hpp"

#include "logger.hpp"

#include <math.h>

static Motor motor_fl_instance(MOTOR_FL_EN, MOTOR_FL_FORWARD, MOTOR_FL_BACKWARD, MOTOR_FL_ENC_A, MOTOR_FL_ENC_B);
static Motor motor_fr_instance(MOTOR_FR_EN, MOTOR_FR_FORWARD, MOTOR_FR_BACKWARD, MOTOR_FR_ENC_A, MOTOR_FR_ENC_B);
static Motor motor_rl_instance(MOTOR_RL_EN, MOTOR_RL_FORWARD, MOTOR_RL_BACKWARD, MOTOR_RL_ENC_A, MOTOR_RL_ENC_B);
static Motor motor_rr_instance(MOTOR_RR_EN, MOTOR_RR_FORWARD, MOTOR_RR_BACKWARD, MOTOR_RR_ENC_A, MOTOR_RR_ENC_B);

Motor::Motor(pin_size_t en, pin_size_t forward, pin_size_t backward, pin_size_t enc_a, pin_size_t enc_b)
: en(en), forward(forward), backward(backward), enc_a(enc_a), enc_b(enc_b) {
    pinMode(en, OUTPUT);
    pinMode(forward, OUTPUT);
    pinMode(backward, OUTPUT);
    pinMode(enc_a, INPUT_PULLUP);
    pinMode(enc_b, INPUT_PULLUP);

    gpio_init_in_ex(&enc_a_hal, digitalPinToPinName(enc_a), PullUp);
    gpio_init_in_ex(&enc_b_hal, digitalPinToPinName(enc_b), PullUp);

    stop();
    reset_count();
    attachInterruptParam(digitalPinToInterrupt(enc_a), &Motor::isr, CHANGE, reinterpret_cast<void*>(this));
}

Motor::~Motor() {
    detachInterrupt(digitalPinToInterrupt(enc_a));
}

int32_t Motor::count() const {
    return enc_count;
}

int Motor::applied_pwm() const {
    return pwm_value;
}

void Motor::reset_count() {
    enc_count = 0;
    prev_enc_count = 0;
}

void Motor::set_pwm(int pwm) {
    manual_pwm_enabled = true;
    write_pwm(pwm);
    output_value = pwm_value;
}

void Motor::write_pwm(int pwm) {
    pwm = constrain(pwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    if (abs(pwm) < MOTOR_PWM_DEADBAND) pwm = 0;

    pwm_value = pwm;

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

void Motor::set_speed(float speed) {
    manual_pwm_enabled = false;
    if (fabsf(speed - prev_target_speed_value) > 0.001f) {
        start_boost_time_s = 0.0f;
        prev_target_speed_value = speed;
    }
    target_speed_value = speed;
}

void Motor::clear_manual_pwm() {
    manual_pwm_enabled = false;
}

void Motor::set_pid(float kp, float ki, float kd) {
    kp_value = kp;
    ki_value = ki;
    kd_value = kd;
    reset_controller();
}

void Motor::update(float dt_s) {
    const int32_t now_count = count();
    const int32_t delta = now_count - prev_enc_count;
    prev_enc_count = now_count;

    if (dt_s <= 0.0f) return;

    raw_speed_value = static_cast<float>(delta) / dt_s;
    current_speed_value = current_speed_value * 0.6f + raw_speed_value * 0.4f;

    if (manual_pwm_enabled) {
        output_value = pwm_value;
        return;
    }

    if (fabsf(target_speed_value) < 0.001f) {
        integral = 0.0f;
        prev_error = 0.0f;
        start_boost_time_s = 0.0f;
        pwm_value = 0;
        output_value = 0.0f;
        digitalWrite(forward, LOW);
        digitalWrite(backward, LOW);
        analogWrite(en, 0);
        return;
    }

    const float error = target_speed_value - current_speed_value;
    const bool moving = fabsf(current_speed_value) >= MOTOR_MOVING_SPEED_TH;
    if (moving) {
        integral += error * dt_s;
        integral = constrain(integral, -MOTOR_INTEGRAL_LIMIT, MOTOR_INTEGRAL_LIMIT);
    }

    const float derivative = (error - prev_error) / dt_s;
    prev_error = error;

    const float direction = target_speed_value > 0.0f ? 1.0f : -1.0f;
    if (!moving) {
        start_boost_time_s += dt_s;
        const float ramp = constrain(start_boost_time_s / MOTOR_PWM_START_RAMP_S, 0.0f, 1.0f);
        const float start_pwm = MOTOR_PWM_START + (MOTOR_PWM_START_MAX - MOTOR_PWM_START) * ramp;
        output_value = direction * start_pwm;
    } else {
        start_boost_time_s = 0.0f;
        const float pid = kp_value * error + ki_value * integral + kd_value * derivative;
        const float feedforward = direction * (MOTOR_PWM_RUN + MOTOR_SPEED_KF * fabsf(target_speed_value));
        output_value = feedforward + pid;
    }

    output_value = constrain(output_value, -static_cast<float>(MOTOR_PWM_MAX), static_cast<float>(MOTOR_PWM_MAX));
    write_pwm(static_cast<int>(output_value));
}

void Motor::stop() {
    target_speed_value = 0.0f;
    prev_target_speed_value = 0.0f;
    current_speed_value = 0.0f;
    raw_speed_value = 0.0f;
    reset_controller();
    manual_pwm_enabled = true;
    write_pwm(0);
}

void Motor::reset_controller() {
    integral = 0.0f;
    prev_error = 0.0f;
    output_value = 0.0f;
    start_boost_time_s = 0.0f;
}

void Motor::isr(void* ptr) {
    auto* motor = reinterpret_cast<Motor*>(ptr);
    if (gpio_read(&motor->enc_b_hal) == gpio_read(&motor->enc_a_hal)) {
        motor->enc_count++;
    } else {
        motor->enc_count--;
    }
}

Motor& motor_fl() {
    return motor_fl_instance;
}

Motor& motor_fr() {
    return motor_fr_instance;
}

Motor& motor_rl() {
    return motor_rl_instance;
}

Motor& motor_rr() {
    return motor_rr_instance;
}

void motors_begin() {
    motors_stop_all();
    motors_reset_all_counts();
    loggf("motors ready\n");
}

void motors_stop_all() {
    motor_fl().stop();
    motor_fr().stop();
    motor_rl().stop();
    motor_rr().stop();
}

void motors_reset_all_counts() {
    motor_fl().reset_count();
    motor_fr().reset_count();
    motor_rl().reset_count();
    motor_rr().reset_count();
}
