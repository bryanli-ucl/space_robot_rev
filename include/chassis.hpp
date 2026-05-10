#pragma once

#include "main.hpp"
#include "hal/gpio_api.h"

class Motor {
    public:
    Motor(pin_size_t en, pin_size_t forward, pin_size_t backward);
    Motor(pin_size_t en, pin_size_t forward, pin_size_t backward, pin_size_t enc_a, pin_size_t enc_b);
    ~Motor();

    int32_t count() const;
    void reset_count();
    void set_speed(float v);
    void update(std::chrono::microseconds dt);
    void write_pwm(int pwm);

    private:
    static void ISR(void* ins_ptr);

    int32_t prev_enc_cnt;
    int32_t enc_cnt;
    pin_size_t en;
    pin_size_t forward;
    pin_size_t backward;
    pin_size_t enc_a;
    pin_size_t enc_b;

    gpio_t enc_a_hal;
    gpio_t enc_b_hal;

    float target_speed  = 0.0f;
    float current_speed = 0.0f;

    float kp = 0.5f;
    float ki = 0.1f;
    float kd = 0.0f;

    float integral   = 0.0f;
    float prev_error = 0.0f;

    float output = 0.0f;
};

class Chassis {
    public:
    Chassis(Motor&, Motor&, Motor&, Motor&);
    ~Chassis() = default;

    void set_target(float vx, float vy, float w);
    void set_paras(float l, float w, float r);

    float get_vfl() const { return vfl; };
    float get_vfr() const { return vfr; };
    float get_vrl() const { return vrl; };
    float get_vrr() const { return vrr; };

    void enable() { is_enable = true; }
    void disable() { is_enable = false; }

    void update(std::chrono::microseconds dt);

    private:
    bool is_enable;

    float L;
    float W;
    float R;

    float vfl;
    float vfr;
    float vrl;
    float vrr;

    Motor& fl;
    Motor& fr;
    Motor& rl;
    Motor& rr;
};
