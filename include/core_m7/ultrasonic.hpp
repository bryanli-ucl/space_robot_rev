#pragma once

#include <stdint.h>

void ultrasonic_begin();
void ultrasonic_update(uint32_t now_ms);
int16_t ultrasonic_front_cm();
int16_t ultrasonic_left_cm();
int16_t ultrasonic_right_cm();
uint32_t ultrasonic_last_duration_ms();
