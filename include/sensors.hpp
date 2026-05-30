#pragma once

#include <stdint.h>

class Sensors {
    public:
    static Sensors& instance();

    void begin();

    // update
    void update_ultrasonic();
    void update_ir();
    void update_buttons();

    // ultrasonic
    int16_t ultrasonic_front_cm() const { return front_cm; }
    int16_t ultrasonic_left_cm() const { return left_cm; }
    int16_t ultrasonic_right_cm() const { return right_cm; }
    int16_t ultrasonic_front_raw_cm() const { return front_raw_cm; }
    int16_t ultrasonic_left_raw_cm() const { return left_raw_cm; }
    int16_t ultrasonic_right_raw_cm() const { return right_raw_cm; }
    uint32_t ultrasonic_last_duration_ms() const { return ultrasonic_duration_ms; }

    // ir sensors
    uint16_t ir_position() const { return ir_pos; }
    const uint16_t* ir_values() const { return ir_vals; }
    uint16_t ir_value(uint8_t index) const { return index < 9 ? ir_vals[index] : 0; }
    uint16_t ir_side_left_value() const { return ir_left_value; }
    uint16_t ir_side_right_value() const { return ir_right_value; }
    bool ir_side_left_detected() const { return ir_left_detected; }
    bool ir_side_right_detected() const { return ir_right_detected; }
    uint32_t ir_last_duration_ms() const { return ir_duration_ms; }

    // buttons
    bool revive_button_pressed() const { return revive_button; }
    bool kill_switch_pressed() const { return kill_switch; }

    private:
    Sensors()                          = default;
    Sensors(const Sensors&)            = delete;
    Sensors& operator=(const Sensors&) = delete;

    int16_t front_cm            = -1;
    int16_t front_raw_cm        = -1;
    float front_filtered_cm     = -1.0f;
    uint8_t front_invalid_count = 0;

    int16_t left_cm            = -1;
    int16_t left_raw_cm        = -1;
    float left_filtered_cm     = -1.0f;
    uint8_t left_invalid_count = 0;

    int16_t right_cm            = -1;
    int16_t right_raw_cm        = -1;
    float right_filtered_cm     = -1.0f;
    uint8_t right_invalid_count = 0;

    uint32_t ultrasonic_duration_ms = 0;
    uint16_t ir_pos                 = 0;
    uint16_t ir_vals[9]             = {};
    uint16_t ir_left_value          = 0;
    uint16_t ir_right_value         = 0;
    bool ir_left_detected           = false;
    bool ir_right_detected          = false;
    bool revive_button              = false;
    bool kill_switch                = false;
    uint32_t ir_duration_ms         = 0;
};

extern Sensors& sensors;

void sensors_begin();
void func_sensors_entry();
