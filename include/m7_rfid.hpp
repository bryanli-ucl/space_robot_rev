#pragma once

#include <stdint.h>

void rfid_begin();
void rfid_update(uint32_t now_ms);
uint32_t rfid_last_uid();
bool rfid_is_ready();
