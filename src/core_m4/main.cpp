#include <Arduino.h>
#include <Wire.h>
#include <mbed.h>

#include "config.hpp"
#include "core_m4/chassis.hpp"
#include "core_m4/commands.hpp"
#include "core_m4/heartbeat.hpp"
#include "core_m4/imu.hpp"
#include "core_m4/mission.hpp"
#include "core_m4/rpc_bridge.hpp"
#include "core_m4/serial.hpp"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <map>
#include <memory>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace ::rtos;
using namespace ::std::chrono_literals;

Thread task_chassis;
Thread task_serial_command;
Thread task_heartbeat(osPriorityBelowNormal7);
Thread task_imu;
Thread task_logger;
Thread task_mission;


void setup() {
    serial_begin();
    m4_commands_begin();
    serial_command_begin();
    m4_chassis_begin();
    heartbeat_begin();
    m4_chassis_task_begin();
    rpc_bridge_begin();
    if (CONFIG::M4::ENABLE_IMU_TASK) {
        imu_begin();
    }
    mission_begin();
}

void loop() {
    ThisThread::sleep_for(1000ms);
}
