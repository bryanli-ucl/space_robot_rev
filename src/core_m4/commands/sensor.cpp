#include "logger.hpp"
#include "imu.hpp"
#include "rfid.hpp"
#include "sensors.hpp"
#include "shell.hpp"

#include <Arduino.h>
#include <string.h>

static void print_dist() {
    loggf("dist front=%d/%dcm left=%d/%dcm right=%d/%dcm dt=%lums\n",
          sensors.ultrasonic_front_cm(),
          sensors.ultrasonic_front_raw_cm(),
          sensors.ultrasonic_left_cm(),
          sensors.ultrasonic_left_raw_cm(),
          sensors.ultrasonic_right_cm(),
          sensors.ultrasonic_right_raw_cm(),
          static_cast<unsigned long>(sensors.ultrasonic_last_duration_ms()));
}

static void print_ir() {
    const uint16_t* values = sensors.ir_values();
    loggf("ir pos=%u vals=%u/%u/%u/%u/%u/%u/%u/%u/%u side=%u/%u det=%d/%d dt=%lums\n",
          sensors.ir_position(),
          values[0],
          values[1],
          values[2],
          values[3],
          values[4],
          values[5],
          values[6],
          values[7],
          values[8],
          sensors.ir_side_left_value(),
          sensors.ir_side_right_value(),
          sensors.ir_side_left_detected() ? 1 : 0,
          sensors.ir_side_right_detected() ? 1 : 0,
          static_cast<unsigned long>(sensors.ir_last_duration_ms()));
}

static void print_imu() {
    loggf("imu ready=%d yaw_ready=%d calibrating=%d yaw=%.2f pitch=%.2f roll=%.2f bias=%.4f/%.4f/%.4f\n",
          imu.is_ready() ? 1 : 0,
          imu.yaw_is_ready() ? 1 : 0,
          imu.is_calibrating() ? 1 : 0,
          imu.yaw_deg(),
          imu.pitch_deg(),
          imu.roll_deg(),
          imu.gyro_bias_x(),
          imu.gyro_bias_y(),
          imu.gyro_bias_z());
}

static void print_rfid() {
    loggf("rfid ready=%d uid=%lu size=%u age=%lums dt=%lums\n",
          rfid.is_ready() ? 1 : 0,
          static_cast<unsigned long>(rfid.last_uid()),
          rfid.last_uid_size(),
          rfid.last_seen_ms() == 0 ? 0UL : static_cast<unsigned long>(millis() - rfid.last_seen_ms()),
          static_cast<unsigned long>(rfid.last_duration_ms()));
}

static void sensor_cmd(int argc, char** argv) {
    if (argc == 1) {
        print_dist();
        return;
    }

    if (strcmp(argv[1], "dist") == 0 || strcmp(argv[1], "status") == 0) {
        print_dist();
        return;
    }

    if (strcmp(argv[1], "ir") == 0) {
        print_ir();
        return;
    }

    if (strcmp(argv[1], "imu") == 0) {
        print_imu();
        return;
    }

    if (strcmp(argv[1], "rfid") == 0) {
        print_rfid();
        return;
    }

    loggf("usage: sensor dist|ir|imu|rfid|status\n");
}

SHELL_COMMAND("sensor", sensor_cmd, "print sensor data: sensor dist|ir|imu|rfid")
