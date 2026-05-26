#pragma once

void imu_begin();

bool imu_is_ready();
bool imu_yaw_ready();
bool imu_is_calibrating();
float imu_yaw_deg();

bool imu_request_calibration(bool gyro, bool mag);
