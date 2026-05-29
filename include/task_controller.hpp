#pragma once

#include <stdint.h>

class TaskController {
    public:
    static TaskController& instance();

    void start(uint8_t task_id);
    void stop();
    void update(float dt_s);
    void print_status() const;

    bool is_active() const { return active; }

    private:
    TaskController()                                 = default;
    TaskController(const TaskController&)            = delete;
    TaskController& operator=(const TaskController&) = delete;

    bool active = false;
    uint8_t task_id = 0;
    const char* phase = "idle";
    uint32_t start_ms = 0;
    uint32_t last_status_ms = 0;

    bool drive_active = false;
    float drive_target_cm = 0.0f;
    float drive_speed = 0.0f;
    int16_t drive_front_stop_cm = -1;
    float drive_traveled_cm = 0.0f;
    int32_t start_fl_count = 0;
    int32_t start_fr_count = 0;
    int32_t start_rl_count = 0;
    int32_t start_rr_count = 0;

    void stop_actions();
    void begin_drive(float speed, float target_cm, int16_t front_stop_cm, const char* next_phase);
    void update_drive();
    void update_drive_distance();
};

extern TaskController& task_controller;

void task_controller_update(float dt_s);
void task_controller_stop();
