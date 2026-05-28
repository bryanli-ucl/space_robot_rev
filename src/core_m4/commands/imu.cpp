#include "imu.hpp"
#include "logger.hpp"
#include "shell.hpp"

#include <string.h>

static void print_imu_status() {
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

static void print_imu_raw() {
    loggf("imu raw acc=%.3f/%.3f/%.3f gyr=%.3f/%.3f/%.3f mag=%.3f/%.3f/%.3f\n",
          imu.accel_x(),
          imu.accel_y(),
          imu.accel_z(),
          imu.gyro_x(),
          imu.gyro_y(),
          imu.gyro_z(),
          imu.mag_x(),
          imu.mag_y(),
          imu.mag_z());
}

static void imu_cmd(int argc, char** argv) {
    if (argc == 1 || strcmp(argv[1], "status") == 0) {
        print_imu_status();
        return;
    }

    if (strcmp(argv[1], "raw") == 0) {
        print_imu_raw();
        return;
    }

    if (strcmp(argv[1], "zero") == 0) {
        imu.zero_yaw();
        loggf("imu yaw zeroed\n");
        return;
    }

    const char* mode = argv[1];
    if (strcmp(argv[1], "cal") == 0 || strcmp(argv[1], "calibrate") == 0) {
        if (argc < 3) {
            loggf("usage: imu cal gyro|mag|all\n");
            return;
        }
        mode = argv[2];
    }

    bool gyro = false;
    bool mag = false;
    if (strcmp(mode, "gyro") == 0) {
        gyro = true;
    } else if (strcmp(mode, "mag") == 0) {
        mag = true;
    } else if (strcmp(mode, "all") == 0) {
        gyro = true;
        mag = true;
    } else {
        loggf("usage: imu [status|raw|zero|gyro|mag|all|cal gyro|cal mag|cal all]\n");
        return;
    }

    if (!imu.request_calibration(gyro, mag)) {
        loggf("imu calibration rejected\n");
        return;
    }

    loggf("imu calibration queued gyro=%d mag=%d\n", gyro ? 1 : 0, mag ? 1 : 0);
}

SHELL_COMMAND("imu", imu_cmd, "IMU status/calibration: imu [status|raw|zero|cal gyro|cal mag|cal all]")
