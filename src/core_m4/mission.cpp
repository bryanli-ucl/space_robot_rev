#include "core_m4/mission.hpp"

#include "core_m4/rpc_bridge.hpp"
#include "core_m4/serial.hpp"

#include <Arduino.h>
#include <mbed.h>

using namespace ::rtos;
using namespace std::chrono_literals;

extern Thread task_mission;

namespace {

constexpr uint32_t SENSOR_LOG_INTERVAL_MS = 2000;
constexpr auto MISSION_LOOP_INTERVAL      = 20ms;

void log_sensor_snapshot() {
    loggf("[m4-mission] sensors ir_pos=%d ir=%d/%d/%d/%d/%d/%d/%d/%d/%d us=%d/%d/%d rfid=%d sunlight=%d\n",
    rpc_bridge_ir_pos(),
    rpc_bridge_ir_raw(0),
    rpc_bridge_ir_raw(1),
    rpc_bridge_ir_raw(2),
    rpc_bridge_ir_raw(3),
    rpc_bridge_ir_raw(4),
    rpc_bridge_ir_raw(5),
    rpc_bridge_ir_raw(6),
    rpc_bridge_ir_raw(7),
    rpc_bridge_ir_raw(8),
    rpc_bridge_ultrasonic_front_cm(),
    rpc_bridge_ultrasonic_left_cm(),
    rpc_bridge_ultrasonic_right_cm(),
    rpc_bridge_rfid_uid(),
    rpc_bridge_sunlight());
}

void mission_entry() {
    loggf("[m4-mission] task ready\n");

    uint32_t last_sensor_log_ms = 0;
    while (true) {
        const uint32_t now_ms = millis();
        if (last_sensor_log_ms == 0 || now_ms - last_sensor_log_ms >= SENSOR_LOG_INTERVAL_MS) {
            last_sensor_log_ms = now_ms;
            log_sensor_snapshot();
        }

        ThisThread::sleep_for(MISSION_LOOP_INTERVAL);
    }
}

} // namespace


void mission_begin() {

    task_mission.start(mission_entry);
    loggf("[m4-mission] started\n");
}
