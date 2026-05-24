#pragma once

#include <stdint.h>

bool rpc_bridge_begin();
void rpc_bridge_update(uint32_t now_ms);
bool rpc_bridge_send_m4_command(const char* command);
