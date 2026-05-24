#pragma once

#include <stdint.h>

void sunlight_begin();
void sunlight_update(uint32_t now_ms);
uint16_t sunlight_value();
