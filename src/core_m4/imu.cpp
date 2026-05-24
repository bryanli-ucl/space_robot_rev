#include "core_m4/imu.hpp"

#include "config.hpp"
#include "core_m4/serial.hpp"

#include <Arduino.h>
#include <ICM_20948.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <mbed.h>

using namespace ::rtos;
using namespace std::chrono_literals;

extern Thread task_imu;

namespace {

ICM_20948_I2C imu;
Madgwick ahrs;

volatile bool imu_ready_value = false;
volatile bool yaw_ready_value = false;
volatile float yaw_deg_value  = 0.0f;

struct MagCalibration {
    float offset_x;
    float offset_y;
    float offset_z;
    float scale_x;
    float scale_y;
    float scale_z;
};

void set_yaw(float yaw) {
    mbed::CriticalSectionLock lock;
    yaw_deg_value   = yaw;
    yaw_ready_value = true;
}

void set_imu_ready(bool ready) {
    mbed::CriticalSectionLock lock;
    imu_ready_value = ready;
}

void scan_i2c_bus(TwoWire& bus, const char* name) {
    loggf("[m4-imu] I2C scan %s begin\n", name);

    bool found = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        bus.beginTransmission(addr);
        const uint8_t err = bus.endTransmission();
        if (err == 0) {
            found = true;
            loggf("[m4-imu] I2C %s found 0x%02X\n", name, addr);
        }
    }

    if (!found) {
        loggf("[m4-imu] I2C %s found none\n", name);
    }
}

bool configure_imu() {
    ICM_20948_Status_e imu_stat = imu.setSampleMode(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr,
    ICM_20948_Sample_Mode_Continuous);
    if (imu_stat != ICM_20948_Stat_Ok) {
        loggf("[m4-imu] setSampleMode fail: %s\n", imu.statusString(imu_stat));
    }

    ICM_20948_smplrt_t sample_rate;
    sample_rate.g = CONFIG::M4::IMU_GYRO_SMPLRT_DIV;
    sample_rate.a = CONFIG::M4::IMU_ACC_SMPLRT_DIV;
    imu_stat      = imu.setSampleRate(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, sample_rate);
    if (imu_stat != ICM_20948_Stat_Ok) {
        loggf("[m4-imu] setSampleRate fail: %s\n", imu.statusString(imu_stat));
    } else {
        loggf("[m4-imu] acc/gyr sample rate %.1fHz\n", CONFIG::M4::IMU_SAMPLE_HZ);
    }

    ICM_20948_dlpcfg_t dlpf;
    dlpf.a   = acc_d50bw4_n68bw8;
    dlpf.g   = gyr_d51bw2_n73bw3;
    imu_stat = imu.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlpf);
    if (imu_stat != ICM_20948_Stat_Ok) {
        loggf("[m4-imu] setDLPFcfg fail: %s\n", imu.statusString(imu_stat));
    }

    imu_stat = imu.enableDLPF(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, true);
    if (imu_stat != ICM_20948_Stat_Ok) {
        loggf("[m4-imu] enableDLPF fail: %s\n", imu.statusString(imu_stat));
    }

    return true;
}

void calibrate_gyro_bias(float& bias_x, float& bias_y, float& bias_z) {
    loggf("[m4-imu] gyro bias calibration begin, keep robot still\n");

    uint32_t sample_count   = 0;
    const uint32_t start_ms = millis();
    while (millis() - start_ms < CONFIG::M4::IMU_GYRO_BIAS_CAL_MS) {
        if (imu.dataReady()) {
            imu.getAGMT();
            bias_x += imu.gyrX();
            bias_y += imu.gyrY();
            bias_z += imu.gyrZ();
            sample_count++;
        }
        ThisThread::sleep_for(5ms);
    }

    if (sample_count > 0) {
        bias_x /= sample_count;
        bias_y /= sample_count;
        bias_z /= sample_count;
    }

    loggf("[m4-imu] gyro bias %.4f %.4f %.4f dps samples=%lu\n",
    bias_x,
    bias_y,
    bias_z,
    static_cast<unsigned long>(sample_count));
}

MagCalibration default_mag_calibration() {
    return {
        CONFIG::M4::MAG_OFF_X,
        CONFIG::M4::MAG_OFF_Y,
        CONFIG::M4::MAG_OFF_Z,
        CONFIG::M4::MAG_SCALE_X,
        CONFIG::M4::MAG_SCALE_Y,
        CONFIG::M4::MAG_SCALE_Z,
    };
}

MagCalibration calibrate_mag_soft_hard_iron(MagCalibration fallback) {
    if (!CONFIG::M4::IMU_MAG_CALIBRATE) {
        return fallback;
    }

    loggf("[m4-imu] mag hard/soft iron calibration begin, rotate robot through all axes\n");

    float min_x           = 1000000.0f;
    float min_y           = 1000000.0f;
    float min_z           = 1000000.0f;
    float max_x           = -1000000.0f;
    float max_y           = -1000000.0f;
    float max_z           = -1000000.0f;
    uint32_t sample_count = 0;

    const uint32_t start_ms = millis();
    while (millis() - start_ms < CONFIG::M4::IMU_MAG_CAL_MS) {
        if (imu.dataReady()) {
            imu.getAGMT();

            const float mx = imu.magX();
            const float my = imu.magY();
            const float mz = imu.magZ();

            min_x = fminf(min_x, mx);
            min_y = fminf(min_y, my);
            min_z = fminf(min_z, mz);
            max_x = fmaxf(max_x, mx);
            max_y = fmaxf(max_y, my);
            max_z = fmaxf(max_z, mz);
            sample_count++;
        }
        ThisThread::sleep_for(20ms);
    }

    const float radius_x   = (max_x - min_x) * 0.5f;
    const float radius_y   = (max_y - min_y) * 0.5f;
    const float radius_z   = (max_z - min_z) * 0.5f;
    const float avg_radius = (radius_x + radius_y + radius_z) / 3.0f;

    if (sample_count == 0 || radius_x <= 0.1f || radius_y <= 0.1f || radius_z <= 0.1f || avg_radius <= 0.1f) {
        loggf("[m4-imu] mag calibration invalid samples=%lu, keeping configured values\n",
        static_cast<unsigned long>(sample_count));
        return fallback;
    }

    MagCalibration calibrated = {
        (min_x + max_x) * 0.5f,
        (min_y + max_y) * 0.5f,
        (min_z + max_z) * 0.5f,
        avg_radius / radius_x,
        avg_radius / radius_y,
        avg_radius / radius_z,
    };

    loggf("[m4-imu] mag calibration done samples=%lu\n", static_cast<unsigned long>(sample_count));
    loggf("[m4-imu] hard iron offset %.6f %.6f %.6f\n",
    calibrated.offset_x,
    calibrated.offset_y,
    calibrated.offset_z);
    loggf("[m4-imu] soft iron scale %.6f %.6f %.6f\n",
    calibrated.scale_x,
    calibrated.scale_y,
    calibrated.scale_z);

    return calibrated;
}

void imu_entry() {
    ThisThread::sleep_for(200ms);

    loggf("[m4-imu] I2C init\n");
    Wire1.begin();
    if (CONFIG::M4::IMU_SCAN_I2C) {
        scan_i2c_bus(Wire1, "Wire1");
    }

    loggf("[m4-imu] init\n");
    bool imu_ok = false;
    if (imu.begin(Wire1, CONFIG::M4::IMU_AD0_VAL) != ICM_20948_Stat_Ok) {
        loggf("[m4-imu] init fail\n");
    } else {
        imu_ok = configure_imu();
        loggf("[m4-imu] init ok\n");
    }

    ahrs.begin(CONFIG::M4::IMU_SAMPLE_HZ);

    float gyro_bias_x = 0.0f;
    float gyro_bias_y = 0.0f;
    float gyro_bias_z = 0.0f;

    MagCalibration mag_calibration = default_mag_calibration();

    if (imu_ok) {
        calibrate_gyro_bias(gyro_bias_x, gyro_bias_y, gyro_bias_z);
        mag_calibration = calibrate_mag_soft_hard_iron(mag_calibration);
    } else {
        loggf("[m4-imu] calibration skipped because IMU init failed\n");
    }

    set_imu_ready(imu_ok);
    loggf("[m4-imu] task ready ok=%d yaw_ready=%d\n", imu_ok ? 1 : 0, imu_yaw_ready() ? 1 : 0);

    uint32_t last_status_ms = 0;
    while (true) {
        if (imu_ok && imu.dataReady()) {
            imu.getAGMT();

            const float ax = imu.accX();
            const float ay = imu.accY();
            const float az = imu.accZ();

            const float gx = imu.gyrX() - gyro_bias_x;
            const float gy = imu.gyrY() - gyro_bias_y;
            const float gz = imu.gyrZ() - gyro_bias_z;

            const float mx = (imu.magX() - mag_calibration.offset_x) * mag_calibration.scale_x;
            const float my = (imu.magY() - mag_calibration.offset_y) * mag_calibration.scale_y;
            const float mz = (imu.magZ() - mag_calibration.offset_z) * mag_calibration.scale_z;

            ahrs.update(gx, gy, gz, ax, ay, az, mx, my, mz);
            set_yaw(ahrs.getYaw());
        }

        const uint32_t now_ms = millis();
        if (last_status_ms == 0 || now_ms - last_status_ms >= CONFIG::M4::IMU_STATUS_INTERVAL_MS) {
            last_status_ms = now_ms;
            loggf("[m4-imu] status ok=%d yaw_ready=%d yaw=%.2f\n",
            imu_is_ready() ? 1 : 0,
            imu_yaw_ready() ? 1 : 0,
            imu_yaw_deg());
        }

        ThisThread::sleep_for(1ms);
    }
}

} // namespace

void imu_begin() {
    task_imu.start(imu_entry);
    loggf("[m4-imu] started\n");
}

bool imu_is_ready() {
    mbed::CriticalSectionLock lock;
    return imu_ready_value;
}

bool imu_yaw_ready() {
    mbed::CriticalSectionLock lock;
    return yaw_ready_value;
}

float imu_yaw_deg() {
    mbed::CriticalSectionLock lock;
    return yaw_deg_value;
}
