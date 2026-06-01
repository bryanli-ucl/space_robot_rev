#pragma once

#include <stdint.h>

void motion_stop_all();
bool motion_drive_blocking(float speed, float target_cm, int16_t front_stop_cm, uint32_t timeout_ms);
bool motion_turn_blocking(float delta_deg, float max_w, float tolerance_deg, uint32_t timeout_ms);
bool motion_turn_to_line_blocking(float direction_deg, float search_w, uint32_t timeout_ms);
bool motion_turn_imu_then_line_blocking(float delta_deg);
bool motion_wait_line_done(uint32_t timeout_ms);
bool motion_wait_blocking(uint32_t duration_ms);
