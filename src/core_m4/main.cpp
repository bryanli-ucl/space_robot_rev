#include "chassis.hpp"
#include "config.hpp"
#include "imu.hpp"
#include "logger.hpp"
#include "mission.hpp"
#include "motor.hpp"
#include "rfid.hpp"
#include "rpc_bridge.hpp"
#include "seed_dropper.hpp"
#include "sensors.hpp"
#include "shell.hpp"
#include "state.hpp"

#include <Arduino.h>
#include <mbed.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

Thread task_shell;
Thread task_logger;
Thread task_chassis;
Thread task_sensors;
Thread task_imu;
Thread task_mission;

void setup() {

    // ====================== Serial and Shell Begin ==========================

    Serial1.begin(SERIAL_BAUD);
    delay(200);

    logger.add_output(Serial1, "serial1");

    shell.add_input(Serial1, "serial1");
    rpc_bridge_begin();
    state_outputs_begin();

    // ========================== Motor and Chassis Begin =====================

    motors_begin();
    chassis_begin();

    // =========================== Sensors Begin =====================

    sensors_begin();
    rfid_begin();
    seed_dropper_begin();
    imu_begin();

    // =========================== Start up Threads ===========================

    task_logger.start(func_logger_entry);
    task_shell.start(shell.func_shell_entry);
    task_chassis.start(func_chassis_entry);
    task_sensors.start(func_sensors_entry);
    task_imu.start(func_imu_entry);
    task_mission.start(func_mission_entry);
}

void loop() {
    ThisThread::sleep_for(1s);
}
