#include "core_m4/rpc_bridge.hpp"

#include "core_m4/commands.hpp"
#include "core_m4/serial.hpp"

#include <Arduino.h>
#include <mbed.h>

#include <string>

namespace {

uint32_t counter = 0;

volatile int last_cmd_seq = -1;
volatile int last_cmd_vx  = 0;
volatile int last_cmd_w   = 0;

volatile int fast_seq = -1;
volatile int ir_pos = 0;
volatile int ir_raw[9] = {};
volatile int ir_side_left = 0;
volatile int ir_side_right = 0;
volatile int ultrasonic_front_cm = -1;
volatile int ultrasonic_left_cm = -1;
volatile int ultrasonic_right_cm = -1;

volatile int slow_seq = -1;
volatile int sunlight_value = 0;
volatile int rfid_uid_value = 0;

volatile int door_response_seq = -1;
volatile int door_response_ready = 0;
volatile int door_response_granted = 0;

int rpc_ping(int value) {
    return value + 1;
}

int rpc_get_counter() {
    mbed::CriticalSectionLock lock;
    return static_cast<int>(counter++);
}

int rpc_get_last_seq() {
    return last_cmd_seq;
}

int rpc_sensor_fast_update(int seq,
                           int pos,
                           int ir0,
                           int ir1,
                           int ir2,
                           int ir3,
                           int ir4,
                           int ir5,
                           int ir6,
                           int ir7,
                           int ir8,
                           int side_left,
                           int side_right,
                           int front_cm,
                           int left_cm,
                           int right_cm) {
    mbed::CriticalSectionLock lock;
    fast_seq             = seq;
    ir_pos               = pos;
    ir_raw[0]            = ir0;
    ir_raw[1]            = ir1;
    ir_raw[2]            = ir2;
    ir_raw[3]            = ir3;
    ir_raw[4]            = ir4;
    ir_raw[5]            = ir5;
    ir_raw[6]            = ir6;
    ir_raw[7]            = ir7;
    ir_raw[8]            = ir8;
    ir_side_left         = side_left;
    ir_side_right        = side_right;
    ultrasonic_front_cm  = front_cm;
    ultrasonic_left_cm   = left_cm;
    ultrasonic_right_cm  = right_cm;
    return seq;
}

int rpc_sensor_slow_update(int seq, int sunlight, int rfid_uid, int safety, int wifi, int mqtt) {
    mbed::CriticalSectionLock lock;
    (void)safety;
    (void)wifi;
    (void)mqtt;

    slow_seq        = seq;
    sunlight_value  = sunlight;
    rfid_uid_value  = rfid_uid;
    return seq;
}

int rpc_mqtt_command(std::string command) {
    if (command.empty()) {
        return 0;
    }

    return m4_command_enqueue("mqtt", command.c_str()) ? static_cast<int>(command.length()) : -1;
}

int rpc_door_response_update(int seq, int granted) {
    mbed::CriticalSectionLock lock;
    door_response_seq = seq;
    door_response_ready = 1;
    door_response_granted = granted ? 1 : 0;
    return seq;
}

} // namespace

void rpc_bridge_begin() {
    RPC.begin();
    RPC.bind("m4_ping", rpc_ping);
    RPC.bind("m4_get_counter", rpc_get_counter);
    RPC.bind("m4_get_last_seq", rpc_get_last_seq);
    RPC.bind("m4_sensor_fast_update", rpc_sensor_fast_update);
    RPC.bind("m4_sensor_slow_update", rpc_sensor_slow_update);
    RPC.bind("m4_mqtt_command", rpc_mqtt_command);
    RPC.bind("m4_door_response_update", rpc_door_response_update);
    loggf("RPC Begin\n");
}

int rpc_bridge_cmd_seq() {
    return last_cmd_seq;
}

int rpc_bridge_fast_seq() {
    return fast_seq;
}

int rpc_bridge_slow_seq() {
    return slow_seq;
}

bool rpc_bridge_fast_ready() {
    return fast_seq >= 0;
}

bool rpc_bridge_slow_ready() {
    return slow_seq >= 0;
}

int rpc_bridge_ir_pos() {
    return ir_pos;
}

int rpc_bridge_ir_raw(uint8_t index) {
    if (index >= 9) {
        return 0;
    }
    return ir_raw[index];
}

bool rpc_bridge_ir_side_left() {
    return ir_side_left != 0;
}

bool rpc_bridge_ir_side_right() {
    return ir_side_right != 0;
}

int rpc_bridge_ultrasonic_front_cm() {
    return ultrasonic_front_cm;
}

int rpc_bridge_ultrasonic_left_cm() {
    return ultrasonic_left_cm;
}

int rpc_bridge_ultrasonic_right_cm() {
    return ultrasonic_right_cm;
}

int rpc_bridge_rfid_uid() {
    return rfid_uid_value;
}

int rpc_bridge_sunlight() {
    return sunlight_value;
}

bool rpc_bridge_send_mqtt_to_server(const char* payload) {
    if (payload == nullptr || payload[0] == '\0') {
        return false;
    }

    const int result = RPC.call("m7_mqtt_send_server", std::string(payload)).as<int>();
    if (RPC.timedOut() || result < 0) {
        loggf("[m4-rpc] mqtt send failed rc=%d payload=%s\n", result, payload);
        return false;
    }

    loggf("[m4-rpc] mqtt send ok bytes=%d payload=%s\n", result, payload);
    return true;
}

void rpc_bridge_clear_door_response() {
    mbed::CriticalSectionLock lock;
    door_response_ready = 0;
    door_response_granted = 0;
}

bool rpc_bridge_door_response_ready() {
    return door_response_ready != 0;
}

bool rpc_bridge_door_response_granted() {
    return door_response_granted != 0;
}
