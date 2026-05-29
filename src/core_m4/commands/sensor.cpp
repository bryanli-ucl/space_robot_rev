#include "logger.hpp"
#include "imu.hpp"
#include "rfid.hpp"
#include "sensors.hpp"
#include "shell.hpp"

#include <Arduino.h>
#include <string.h>

static bool watch_all = false;
static bool watch_dist = false;
static bool watch_ir = false;
static bool watch_imu = false;
static bool watch_rfid = false;
static uint32_t last_watch_ms = 0;
static constexpr uint32_t WATCH_INTERVAL_MS = 500;

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
    loggf("rfid ready=%d valid=%d uid=%lu cached=%lu size=%u age=%lums dt=%lums watch=%d\n",
          rfid.is_ready() ? 1 : 0,
          rfid.uid_is_valid() ? 1 : 0,
          static_cast<unsigned long>(rfid.last_uid()),
          static_cast<unsigned long>(rfid.cached_uid()),
          rfid.last_uid_size(),
          rfid.last_seen_ms() == 0 ? 0UL : static_cast<unsigned long>(millis() - rfid.last_seen_ms()),
          static_cast<unsigned long>(rfid.last_duration_ms()),
          (watch_all || watch_rfid) ? 1 : 0);
}

static void print_all() {
    print_dist();
    print_ir();
    print_imu();
    print_rfid();
}

static bool* watch_flag(const char* name) {
    if (strcmp(name, "all") == 0 || strcmp(name, "status") == 0) return &watch_all;
    if (strcmp(name, "dist") == 0) return &watch_dist;
    if (strcmp(name, "ir") == 0) return &watch_ir;
    if (strcmp(name, "imu") == 0) return &watch_imu;
    if (strcmp(name, "rfid") == 0) return &watch_rfid;
    return nullptr;
}

static void print_watch_status() {
    loggf("sensor watch all=%d dist=%d ir=%d imu=%d rfid=%d interval=%lums\n",
          watch_all ? 1 : 0,
          watch_dist ? 1 : 0,
          watch_ir ? 1 : 0,
          watch_imu ? 1 : 0,
          watch_rfid ? 1 : 0,
          static_cast<unsigned long>(WATCH_INTERVAL_MS));
}

static bool handle_watch_command(int argc, char** argv, int name_index) {
    if (argc <= name_index + 1 || strcmp(argv[name_index + 1], "watch") != 0) return false;

    bool* flag = watch_flag(argv[name_index]);
    if (flag == nullptr) {
        loggf("sensor watch target must be all, dist, ir, imu, or rfid\n");
        return true;
    }

    if (argc == name_index + 2) {
        print_watch_status();
        return true;
    }

    if (argc != name_index + 3) {
        loggf("usage: sensor [all|dist|ir|imu|rfid] watch on|off\n");
        return true;
    }

    if (strcmp(argv[name_index + 2], "on") == 0) {
        *flag = true;
        print_watch_status();
        return true;
    }

    if (strcmp(argv[name_index + 2], "off") == 0) {
        *flag = false;
        print_watch_status();
        return true;
    }

    loggf("sensor watch state must be on or off\n");
    return true;
}

static void sensor_cmd(int argc, char** argv) {
    if (argc == 1) {
        print_all();
        return;
    }

    if (strcmp(argv[1], "watch") == 0) {
        char* all_argv[] = { argv[0], const_cast<char*>("all"), argv[1], argc >= 3 ? argv[2] : nullptr };
        handle_watch_command(argc + 1, all_argv, 1);
        return;
    }

    if (argc >= 3 && handle_watch_command(argc, argv, 1)) return;

    if (strcmp(argv[1], "all") == 0 || strcmp(argv[1], "status") == 0) {
        print_all();
        return;
    }

    if (strcmp(argv[1], "dist") == 0) {
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

    if (strcmp(argv[1], "rfid") == 0 && argc == 2) {
        print_rfid();
        return;
    }

    loggf("usage: sensor [all|dist|ir|imu|rfid|status] | sensor [all|dist|ir|imu|rfid] watch on|off\n");
}

void sensor_watch_update() {
    const uint32_t now_ms = millis();
    if (last_watch_ms != 0 && now_ms - last_watch_ms < WATCH_INTERVAL_MS) return;
    last_watch_ms = now_ms;

    if (watch_all) {
        print_all();
        return;
    }

    if (watch_dist) print_dist();
    if (watch_ir) print_ir();
    if (watch_imu) print_imu();
    if (watch_rfid) print_rfid();
}

SHELL_COMMAND("sensor", sensor_cmd, "print sensor data: sensor dist|ir|imu|rfid")
