#pragma once

#include "config.hpp"

#include <stdint.h>

class WallFollower {
    public:
    enum class Side {
        Left,
        Right,
    };

    static WallFollower& instance();

    void start(Side side, float speed, float target_distance_cm, int16_t target_wall_cm);
    void set_pid(float distance_kp, float yaw_kp, float max_w);
    void stop();
    void update(float dt_s);
    void print_status() const;

    bool is_active() const { return active; }
    float distance_kp() const { return distance_kp_value; }
    float yaw_kp() const { return yaw_kp_value; }
    float max_w() const { return max_w_value; }
    const char* side_name() const;

    private:
    WallFollower()                               = default;
    WallFollower(const WallFollower&)            = delete;
    WallFollower& operator=(const WallFollower&) = delete;

    bool active = false;
    Side side = Side::Left;
    float speed = 0.0f;
    float target_distance_cm = 0.0f;
    float traveled_distance_cm = 0.0f;
    int16_t target_wall_cm = 0;
    int16_t wall_cm = -1;
    int16_t front_cm = -1;
    float wall_error = 0.0f;
    float yaw_start = 0.0f;
    float yaw_error = 0.0f;
    float yaw_term = 0.0f;
    float distance_term = 0.0f;
    float w = 0.0f;
    float distance_kp_value = WALL_DIST_KP;
    float yaw_kp_value = WALL_YAW_KP;
    float max_w_value = WALL_MAX_WHEEL_SPEED;
    uint8_t lost_count = 0;
    int32_t start_fl_count = 0;
    int32_t start_fr_count = 0;
    int32_t start_rl_count = 0;
    int32_t start_rr_count = 0;
    uint32_t last_status_ms = 0;

    void update_traveled_distance();
    bool should_stop();
};

extern WallFollower& wall_follower;

void wall_follower_update(float dt_s);
void wall_follower_stop();
