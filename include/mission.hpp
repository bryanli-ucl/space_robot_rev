#pragma once

#include <stdint.h>

bool mission_start_task(uint8_t task_id);
void mission_stop();
void mission_print_status();
void func_mission_entry();
