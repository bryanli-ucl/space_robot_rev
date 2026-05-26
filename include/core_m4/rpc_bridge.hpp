#pragma once

#include <Arduino.h>
#include <RPC.h>

void rpc_bridge_begin();
void rpc_bridge_update(uint32_t now_ms);

int rpc_bridge_fast_seq();
int rpc_bridge_slow_seq();
bool rpc_bridge_fast_ready();
bool rpc_bridge_slow_ready();
int rpc_bridge_ir_pos();
int rpc_bridge_ir_raw(uint8_t index);
bool rpc_bridge_ir_side_left();
bool rpc_bridge_ir_side_right();
int rpc_bridge_ultrasonic_front_cm();
int rpc_bridge_ultrasonic_left_cm();
int rpc_bridge_ultrasonic_right_cm();
int rpc_bridge_rfid_uid();
int rpc_bridge_sunlight();

bool rpc_bridge_send_mqtt_to_server(const char* payload);
void rpc_bridge_clear_door_response();
bool rpc_bridge_door_response_ready();
bool rpc_bridge_door_response_granted();
