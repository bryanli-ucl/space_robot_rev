#pragma once

#include <stdint.h>

class WallFollower {
    public:
    enum class Side {
        Left,
        Right,
    };

    static WallFollower& instance();

    void start(Side side, float speed, float target_distance_cm, int16_t target_wall_cm);
    void stop();
    void update(float dt_s);
    void print_status() const;

    bool is_active() const { return active; }
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
