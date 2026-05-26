#pragma once

#include <stdint.h>

void ir_begin();
void ir_update(uint32_t now_ms);
uint16_t ir_position();
const uint16_t* ir_values();
bool ir_side_left_detected();
bool ir_side_right_detected();
