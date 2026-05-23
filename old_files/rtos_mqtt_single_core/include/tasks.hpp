#pragma once

#include "main.hpp"

extern Thread task_heartbeat;
extern Thread task_serial_debug;
extern Thread task_sensors;
extern Thread task_imu;
extern Thread task_rfid;
extern Thread task_mission;
extern Thread task_chassis;
extern Thread task_mqtt;

void func_heartbeat();
void func_serial_debug();
void func_chassis();
void func_mission();
void func_sensors();
void func_imu();
void func_rfid();
void func_mqtt();
