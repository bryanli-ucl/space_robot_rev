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
    void set_position_delta(int32_t delta_count, float max_speed);
    void set_position_target(int32_t target_count, float max_speed);
    void clear_position_control();
    void set_test_speed(float v);
    void clear_speed_override();
    void update(std::chrono::microseconds dt);
    void write_pwm(int pwm);
    void set_manual_pwm(int pwm);
    void clear_manual_pwm();
    void stop();
    void set_pid(float p, float i, float d);
    void reset_controller();
    bool position_reached(int32_t tolerance = 5) const;

    float get_target_speed() const { return target_speed; }
    float get_current_speed() const { return current_speed; }
    float get_kp() const { return kp; }
    float get_ki() const { return ki; }
    float get_kd() const { return kd; }
    float get_output() const { return output; }
    int32_t get_last_delta() const { return last_delta; }
    float get_raw_speed() const { return raw_speed; }
    float get_last_error() const { return last_error; }
    float get_integral() const { return integral; }
    float get_derivative() const { return derivative; }
    int get_applied_pwm() const { return applied_pwm; }
    bool is_position_control() const { return position_control_enabled; }
    int32_t get_position_target() const { return position_target_count; }
    int32_t get_position_error() const { return position_target_count - count(); }
    float get_position_max_speed() const { return position_max_speed; }
    bool is_manual_pwm() const { return manual_pwm_enabled; }
    bool is_speed_override() const { return speed_override_enabled; }

    private:
    static void ISR(void* ins_ptr);

    int32_t prev_enc_cnt;
    volatile int32_t enc_cnt;
    pin_size_t en;
    pin_size_t forward;
    pin_size_t backward;
    pin_size_t enc_a;
    pin_size_t enc_b;

    gpio_t enc_a_hal;
    gpio_t enc_b_hal;

    float target_speed  = 0.0f;
    float current_speed = 0.0f;

    float kp = 0.25f;
    float ki = 1.8f;
    float kd = 0.1f;

    float integral   = 0.0f;
    float prev_error = 0.0f;
    float last_error = 0.0f;
    float derivative = 0.0f;

    float output = 0.0f;
    int32_t last_delta = 0;
    float raw_speed    = 0.0f;
    int applied_pwm    = 0;
    int32_t speed_sample_delta = 0;
    float speed_sample_dt      = 0.0f;

    bool manual_pwm_enabled = false;
    int manual_pwm          = 0;
    bool speed_override_enabled = false;

    bool position_control_enabled = false;
    int32_t position_target_count = 0;
    float position_max_speed      = 0.0f;
    float position_kp             = 4.0f;
};

class Chassis {
    public:
    Chassis(Motor&, Motor&, Motor&, Motor&);
    ~Chassis() = default;

    void set_target(float vx, float vy, float w);
    void apply_target(float vx, float vy, float w);
    void set_paras(float l, float w, float r);
    void move_counts(int32_t counts, float max_speed);
    void clear_position_control();
    bool position_reached(int32_t tolerance = 5) const;

    float get_target_vx() const { return target_vx; };
    float get_target_vy() const { return target_vy; };
    float get_target_w() const { return target_w; };

    float get_vfl() const { return vfl; };
    float get_vfr() const { return vfr; };
    float get_vrl() const { return vrl; };
    float get_vrr() const { return vrr; };

    void enable() { is_enable = true; }
    void disable() { is_enable = false; }

    void update(std::chrono::microseconds dt);

    private:
    bool is_enable = false;

    float L = 0.0f;
    float W = 0.0f;
    float R = 1.0f;

    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_w  = 0.0f;

    float vfl = 0.0f;
    float vfr = 0.0f;
    float vrl = 0.0f;
    float vrr = 0.0f;

    Motor& fl;
    Motor& fr;
    Motor& rl;
    Motor& rr;
};
