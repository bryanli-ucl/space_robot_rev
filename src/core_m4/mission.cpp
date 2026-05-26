#include "core_m4/mission.hpp"

#include "config.hpp"
#include "core_m4/imu.hpp"
#include "core_m4/rpc_bridge.hpp"
#include "core_m4/serial.hpp"

#include <Arduino.h>
#include <mbed.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

extern Thread task_mission;

namespace {

void wait_for_startup_ready() {
    loggf("[m4-mission] waiting startup delay %lums\n",
          static_cast<unsigned long>(CONFIG::M4::MISSION_START_DELAY_MS));
    ThisThread::sleep_for(std::chrono::milliseconds(CONFIG::M4::MISSION_START_DELAY_MS));

    const uint32_t sensor_start_ms = millis();
    while ((!rpc_bridge_fast_ready() || !rpc_bridge_slow_ready()) &&
           millis() - sensor_start_ms < CONFIG::M4::MISSION_SENSOR_READY_TIMEOUT_MS) {
        ThisThread::sleep_for(100ms);
    }

    loggf("[m4-mission] rpc ready fast=%d slow=%d fast_seq=%d slow_seq=%d\n",
          rpc_bridge_fast_ready() ? 1 : 0,
          rpc_bridge_slow_ready() ? 1 : 0,
          rpc_bridge_fast_seq(),
          rpc_bridge_slow_seq());

    const uint32_t imu_start_ms = millis();
    while (!imu_is_ready() &&
           millis() - imu_start_ms < CONFIG::M4::MISSION_IMU_READY_TIMEOUT_MS) {
        ThisThread::sleep_for(100ms);
    }

    loggf("[m4-mission] imu ready=%d yaw_ready=%d yaw=%.2f\n",
          imu_is_ready() ? 1 : 0,
          imu_yaw_ready() ? 1 : 0,
          imu_yaw_deg());
    loggf("[m4-mission] startup gate complete\n");
}

void mission_entry() {
    loggf("[m4-mission] task ready\n");
    wait_for_startup_ready();

    while (true) {
        ThisThread::sleep_for(1000ms);

        // Example later:
        // run_line_follow_until_cross(...);
        // run_turn_deg(...);
        // run_line_follow_until_rfid_uid(...);
    }
}

} // namespace


void mission_begin() {

    task_mission.start(mission_entry);
    loggf("[m4-mission] started\n");
}
