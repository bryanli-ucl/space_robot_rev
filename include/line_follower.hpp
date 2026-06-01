#pragma once

#include "config.hpp"

#include <stdint.h>

class LineFollower {
    public:
    enum class StopMode {
        None,
        Cross,
        Front,
        Distance,
        Rfid,
        LeftCorner,
        RightCorner,
        AnyCorner,
    };

    static LineFollower& instance();

    void start(float speed, StopMode mode = StopMode::None, int16_t front_stop_cm = -1, float target_distance_cm = -1.0f);
    void start_rfid(float speed, uint32_t target_uid, bool any_uid, bool not_same, int16_t front_stop_cm = -1);
    void set_speed(float speed);
    void set_pid(float kp, float kd);
    void stop();
    void update(float dt_s);
    void print_status() const;

    bool is_active() const { return active; }
    float target_speed() const { return speed; }
    float last_error() const { return error; }
    float last_w() const { return w; }
    float last_vx() const { return vx; }
    float kp() const { return kp_value; }
    float kd() const { return kd_value; }
    uint8_t last_black_count() const { return black_count; }
    uint8_t no_line_count_value() const { return no_line_count; }
    StopMode stop_mode() const { return mode; }
    int16_t front_stop_distance_cm() const { return front_stop_cm; }
    float target_distance_cm_value() const { return target_distance_cm; }
    float traveled_distance_cm_value() const { return traveled_distance_cm; }
    uint32_t rfid_target_uid_value() const { return rfid_target_uid; }
    uint32_t rfid_ignore_uid_value() const { return rfid_ignore_uid; }
    uint32_t last_rfid_stop_uid_value() const { return last_rfid_stop_uid; }
    const char* stop_mode_name() const;

    private:
    LineFollower()                               = default;
    LineFollower(const LineFollower&)            = delete;
    LineFollower& operator=(const LineFollower&) = delete;

    bool active = false;
    StopMode mode = StopMode::None;
    int16_t front_stop_cm = -1;
    float target_distance_cm = -1.0f;
    float traveled_distance_cm = 0.0f;
    uint32_t rfid_target_uid = 0;
    uint32_t rfid_ignore_uid = 0;
    uint32_t last_rfid_stop_uid = 0;
    bool rfid_any_uid = false;
    bool rfid_not_same = false;
    int32_t start_fl_count = 0;
    int32_t start_fr_count = 0;
    int32_t start_rl_count = 0;
    int32_t start_rr_count = 0;
    float speed = 0.0f;
    float kp_value = LINE_KP;
    float kd_value = LINE_KD;
    float error = 0.0f;
    float prev_error = 0.0f;
    float w = 0.0f;
    float vx = 0.0f;
    uint8_t black_count = 0;
    uint8_t no_line_count = 0;
    uint8_t cross_count = 0;
    uint8_t left_corner_count = 0;
    uint8_t right_corner_count = 0;
    uint32_t last_status_ms = 0;

    bool should_stop_for_event();
    void update_traveled_distance();
};

extern LineFollower& line_follower;

void line_follower_update(float dt_s);
void line_follower_stop();
