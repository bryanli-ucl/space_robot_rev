#pragma once

#include "config.hpp"

#include <Arduino.h>
#include <stdint.h>

#include "hal/gpio_api.h"

class Motor {
    public:
    Motor(pin_size_t en, pin_size_t forward, pin_size_t backward, pin_size_t enc_a, pin_size_t enc_b);
    ~Motor();

    int32_t count() const;
    int applied_pwm() const;
    float target_speed() const { return target_speed_value; }
    float current_speed() const { return current_speed_value; }
    float raw_speed() const { return raw_speed_value; }
    float output() const { return output_value; }
    float integral_value() const { return integral; }
    float kp() const { return kp_value; }
    float ki() const { return ki_value; }
    float kd() const { return kd_value; }

    void reset_count();
    void set_pwm(int pwm);
    void set_speed(float speed);
    void clear_manual_pwm();
    void set_pid(float kp, float ki, float kd);
    void update(float dt_s);
    void stop();

    private:
    static void isr(void* ptr);
    void write_pwm(int pwm);
    void reset_controller();

    pin_size_t en;
    pin_size_t forward;
    pin_size_t backward;
    pin_size_t enc_a;
    pin_size_t enc_b;

    gpio_t enc_a_hal;
    gpio_t enc_b_hal;

    volatile int32_t enc_count = 0;
    int32_t prev_enc_count     = 0;
    int pwm_value              = 0;

    bool manual_pwm_enabled = false;
    float target_speed_value = 0.0f;
    float current_speed_value = 0.0f;
    float raw_speed_value = 0.0f;
    float kp_value = MOTOR_PID_KP;
    float ki_value = MOTOR_PID_KI;
    float kd_value = MOTOR_PID_KD;
    float integral = 0.0f;
    float prev_error = 0.0f;
    float output_value = 0.0f;
    float start_boost_time_s = 0.0f;
    float prev_target_speed_value = 0.0f;
};

Motor& motor_fl();
Motor& motor_fr();
Motor& motor_rl();
Motor& motor_rr();

void motors_begin();
void motors_stop_all();
void motors_reset_all_counts();
