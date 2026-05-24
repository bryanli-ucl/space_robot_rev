#pragma once

#include <Arduino.h>
#include <RPC.h>

void rpc_bridge_begin();
void rpc_bridge_update(uint32_t now_ms);

int rpc_bridge_ir_pos();
int rpc_bridge_ir_raw(uint8_t index);
int rpc_bridge_ultrasonic_front_cm();
int rpc_bridge_ultrasonic_left_cm();
int rpc_bridge_ultrasonic_right_cm();
int rpc_bridge_rfid_uid();
int rpc_bridge_sunlight();
