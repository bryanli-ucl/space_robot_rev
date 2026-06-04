#pragma once

#include "config.hpp"

class FastLineFollower {
    public:
    static FastLineFollower& instance();

    void start();
    void stop();
    void update(float dt_s);
    void set_speed(float next_speed);
    void set_pid(float kp, float ki, float kd);
    void set_recovery_speed_scale(float scale);
    void print_status() const;

    bool is_active() const { return active; }
    float target_speed() const { return speed; }
    float kp() const { return kp_value; }
    float ki() const { return ki_value; }
    float kd() const { return kd_value; }
    float recovery_speed_scale() const { return recovery_speed_scale_value; }

    private:
    enum class RecoveryMode {
        None,
        Left,
        Right,
    };

    FastLineFollower()                                   = default;
    FastLineFollower(const FastLineFollower&)            = delete;
    FastLineFollower& operator=(const FastLineFollower&) = delete;

    bool active = false;
    RecoveryMode recovery = RecoveryMode::None;
    float speed = LINEF_DEFAULT_SPEED;
    float kp_value = LINEF_KP;
    float ki_value = LINEF_KI;
    float kd_value = LINEF_KD;
    float recovery_speed_scale_value = LINEF_RECOVERY_SPEED_SCALE;
    float integral = 0.0f;
    float error = 0.0f;
    float prev_error = 0.0f;
    float vx = 0.0f;
    float w = 0.0f;
    uint8_t center_confirm_count = 0;
    uint32_t last_status_ms = 0;

    bool center_sees_line() const;
    const char* recovery_name() const;
};

extern FastLineFollower& fast_line_follower;

void fast_line_follower_update(float dt_s);
void fast_line_follower_stop();
