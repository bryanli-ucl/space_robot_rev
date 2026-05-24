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
volatile int ultrasonic_front_cm = -1;
volatile int ultrasonic_left_cm = -1;
volatile int ultrasonic_right_cm = -1;

volatile int slow_seq = -1;
volatile int sunlight_value = 0;
volatile int rfid_uid_value = 0;


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

} // namespace

void rpc_bridge_begin() {
    RPC.begin();
    RPC.bind("m4_ping", rpc_ping);
    RPC.bind("m4_get_counter", rpc_get_counter);
    RPC.bind("m4_get_last_seq", rpc_get_last_seq);
    RPC.bind("m4_sensor_fast_update", rpc_sensor_fast_update);
    RPC.bind("m4_sensor_slow_update", rpc_sensor_slow_update);
    RPC.bind("m4_mqtt_command", rpc_mqtt_command);
    loggf("RPC Begin\n");
}

int rpc_bridge_cmd_seq() {
    return last_cmd_seq;
}

int rpc_bridge_fast_seq() {
    return fast_seq;
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
