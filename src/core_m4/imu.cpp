#include "imu.hpp"

#include "config.hpp"
#include "logger.hpp"

#include <Arduino.h>
#include <ICM_20948.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <mbed.h>
#include <math.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

struct MagCalibration {
    float offset_x;
    float offset_y;
    float offset_z;
    float scale_x;
    float scale_y;
    float scale_z;
};

Imu& imu = Imu::instance();

static ICM_20948_I2C imu_sensor;
static Madgwick ahrs;
static float gyro_bias_x_cal = 0.0f;
static float gyro_bias_y_cal = 0.0f;
static float gyro_bias_z_cal = 0.0f;
static float gyro_yaw_deg = 0.0f;
static uint32_t last_imu_update_us = 0;
static MagCalibration mag_calibration = {
    MAG_OFF_X,
    MAG_OFF_Y,
    MAG_OFF_Z,
    MAG_SCALE_X,
    MAG_SCALE_Y,
    MAG_SCALE_Z,
};

static float wrap_deg_360(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}

static uint32_t imu_status_interval_ms() {
    return IMU_STATUS_INTERVAL_MS;
}

static void scan_i2c_bus(TwoWire& bus, const char* name) {
    loggf("imu i2c scan %s begin\n", name);

    bool found = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        bus.beginTransmission(addr);
        const uint8_t err = bus.endTransmission();
        if (err == 0) {
            found = true;
            loggf("imu i2c %s found 0x%02X\n", name, addr);
        }
    }

    if (!found) loggf("imu i2c %s found none\n", name);
}

static bool configure_imu() {
    ICM_20948_Status_e stat = imu_sensor.setSampleMode(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr,
                                                       ICM_20948_Sample_Mode_Continuous);
    if (stat != ICM_20948_Stat_Ok) loggf("imu setSampleMode fail: %s\n", imu_sensor.statusString(stat));

    ICM_20948_smplrt_t sample_rate;
    sample_rate.g = IMU_GYRO_SMPLRT_DIV;
    sample_rate.a = IMU_ACC_SMPLRT_DIV;
    stat = imu_sensor.setSampleRate(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, sample_rate);
    if (stat != ICM_20948_Stat_Ok) {
        loggf("imu setSampleRate fail: %s\n", imu_sensor.statusString(stat));
    } else {
        loggf("imu acc/gyr sample rate %.1fHz\n", IMU_SAMPLE_HZ);
    }

    ICM_20948_dlpcfg_t dlpf;
    dlpf.a = acc_d50bw4_n68bw8;
    dlpf.g = gyr_d51bw2_n73bw3;
    stat = imu_sensor.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlpf);
    if (stat != ICM_20948_Stat_Ok) loggf("imu setDLPFcfg fail: %s\n", imu_sensor.statusString(stat));

    stat = imu_sensor.enableDLPF(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, true);
    if (stat != ICM_20948_Stat_Ok) loggf("imu enableDLPF fail: %s\n", imu_sensor.statusString(stat));

    return true;
}

static void calibrate_gyro_bias() {
    loggf("imu gyro bias calibration begin, keep robot still\n");

    gyro_bias_x_cal = 0.0f;
    gyro_bias_y_cal = 0.0f;
    gyro_bias_z_cal = 0.0f;

    uint32_t count = 0;
    const uint32_t start_ms = millis();
    while (millis() - start_ms < IMU_GYRO_BIAS_CAL_MS) {
        if (imu_sensor.dataReady()) {
            imu_sensor.getAGMT();
            gyro_bias_x_cal += imu_sensor.gyrX();
            gyro_bias_y_cal += imu_sensor.gyrY();
            gyro_bias_z_cal += imu_sensor.gyrZ();
            count++;
        }
        ThisThread::sleep_for(5ms);
    }

    if (count > 0) {
        gyro_bias_x_cal /= count;
        gyro_bias_y_cal /= count;
        gyro_bias_z_cal /= count;
    }

    loggf("imu gyro bias %.4f %.4f %.4f dps samples=%lu\n",
          gyro_bias_x_cal,
          gyro_bias_y_cal,
          gyro_bias_z_cal,
          static_cast<unsigned long>(count));
}

static MagCalibration default_mag_calibration() {
    return {
        MAG_OFF_X,
        MAG_OFF_Y,
        MAG_OFF_Z,
        MAG_SCALE_X,
        MAG_SCALE_Y,
        MAG_SCALE_Z,
    };
}

static MagCalibration calibrate_mag(MagCalibration fallback, bool enabled) {
    if (!enabled) return fallback;

    loggf("imu mag calibration begin, rotate robot through all axes\n");

    float min_x = 1000000.0f;
    float min_y = 1000000.0f;
    float min_z = 1000000.0f;
    float max_x = -1000000.0f;
    float max_y = -1000000.0f;
    float max_z = -1000000.0f;
    uint32_t count = 0;

    const uint32_t start_ms = millis();
    while (millis() - start_ms < IMU_MAG_CAL_MS) {
        if (imu_sensor.dataReady()) {
            imu_sensor.getAGMT();
            const float mx = imu_sensor.magX();
            const float my = imu_sensor.magY();
            const float mz = imu_sensor.magZ();

            min_x = fminf(min_x, mx);
            min_y = fminf(min_y, my);
            min_z = fminf(min_z, mz);
            max_x = fmaxf(max_x, mx);
            max_y = fmaxf(max_y, my);
            max_z = fmaxf(max_z, mz);
            count++;
        }
        ThisThread::sleep_for(20ms);
    }

    const float radius_x = (max_x - min_x) * 0.5f;
    const float radius_y = (max_y - min_y) * 0.5f;
    const float radius_z = (max_z - min_z) * 0.5f;
    const float avg_radius = (radius_x + radius_y + radius_z) / 3.0f;

    if (count == 0 || radius_x <= 0.1f || radius_y <= 0.1f || radius_z <= 0.1f || avg_radius <= 0.1f) {
        loggf("imu mag calibration invalid samples=%lu, keeping configured values\n", static_cast<unsigned long>(count));
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

    loggf("imu mag calibration done samples=%lu\n", static_cast<unsigned long>(count));
    loggf("imu mag offset %.6f %.6f %.6f scale %.6f %.6f %.6f\n",
          calibrated.offset_x,
          calibrated.offset_y,
          calibrated.offset_z,
          calibrated.scale_x,
          calibrated.scale_y,
          calibrated.scale_z);
    loggf("config: MAG_OFF_X=%.6ff MAG_OFF_Y=%.6ff MAG_OFF_Z=%.6ff\n",
          calibrated.offset_x,
          calibrated.offset_y,
          calibrated.offset_z);
    loggf("config: MAG_SCALE_X=%.6ff MAG_SCALE_Y=%.6ff MAG_SCALE_Z=%.6ff\n",
          calibrated.scale_x,
          calibrated.scale_y,
          calibrated.scale_z);
    return calibrated;
}

void Imu::set_orientation(float yaw, float pitch, float roll) {
    mbed::CriticalSectionLock lock;
    yaw_deg_value = yaw;
    pitch_deg_value = pitch;
    roll_deg_value = roll;
    yaw_ready_value = true;
}

void Imu::set_raw(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    mbed::CriticalSectionLock lock;
    accel_x_value = ax;
    accel_y_value = ay;
    accel_z_value = az;
    gyro_x_value = gx;
    gyro_y_value = gy;
    gyro_z_value = gz;
    mag_x_value = mx;
    mag_y_value = my;
    mag_z_value = mz;
}

void Imu::set_gyro_bias(float bx, float by, float bz) {
    mbed::CriticalSectionLock lock;
    gyro_bias_x_value = bx;
    gyro_bias_y_value = by;
    gyro_bias_z_value = bz;
}

void Imu::set_ready(bool ready) {
    mbed::CriticalSectionLock lock;
    ready_value = ready;
}

void Imu::set_calibration_busy(bool busy) {
    mbed::CriticalSectionLock lock;
    calibration_busy_value = busy;
}

void Imu::run_requested_calibration(bool gyro, bool mag) {
    if (!is_ready()) {
        loggf("imu calibration ignored, imu is not ready\n");
        return;
    }

    set_calibration_busy(true);
    if (gyro) {
        calibrate_gyro_bias();
        set_gyro_bias(gyro_bias_x_cal, gyro_bias_y_cal, gyro_bias_z_cal);
        zero_yaw();
    }
    if (mag) mag_calibration = calibrate_mag(mag_calibration, true);

    {
        mbed::CriticalSectionLock lock;
        yaw_ready_value = false;
    }
    ahrs.begin(IMU_SAMPLE_HZ);
    set_calibration_busy(false);
    loggf("imu calibration complete gyro=%d mag=%d\n", gyro ? 1 : 0, mag ? 1 : 0);
}

void Imu::service_calibration_request() {
    bool gyro = false;
    bool mag = false;

    {
        mbed::CriticalSectionLock lock;
        gyro = gyro_calibration_requested;
        mag = mag_calibration_requested;
        gyro_calibration_requested = false;
        mag_calibration_requested = false;
    }

    if (gyro || mag) run_requested_calibration(gyro, mag);
}

Imu& Imu::instance() {
    static Imu instance;
    return instance;
}

void Imu::begin() {
    loggf("imu started\n");
}

void Imu::entry() {
    ThisThread::sleep_for(200ms);

    loggf("imu i2c init\n");
    Wire1.begin();
    if (IMU_SCAN_I2C) scan_i2c_bus(Wire1, "Wire1");

    loggf("imu init\n");
    bool ok = false;
    if (imu_sensor.begin(Wire1, IMU_AD0_VAL) != ICM_20948_Stat_Ok) {
        loggf("imu init fail\n");
    } else {
        ok = configure_imu();
        loggf("imu init ok\n");
    }

    ahrs.begin(IMU_SAMPLE_HZ);
    last_imu_update_us = micros();
    mag_calibration = default_mag_calibration();

    if (ok) {
        calibrate_gyro_bias();
        set_gyro_bias(gyro_bias_x_cal, gyro_bias_y_cal, gyro_bias_z_cal);
        zero_yaw();
        mag_calibration = calibrate_mag(mag_calibration, IMU_MAG_CALIBRATE);
    } else {
        loggf("imu calibration skipped because init failed\n");
    }

    set_ready(ok);
    loggf("imu ready ok=%d yaw_ready=%d\n", ok ? 1 : 0, yaw_is_ready() ? 1 : 0);

    uint32_t last_status_ms = 0;
    while (true) {
        service_calibration_request();

        if (ok && imu_sensor.dataReady()) {
            imu_sensor.getAGMT();

            const float ax = imu_sensor.accX();
            const float ay = imu_sensor.accY();
            const float az = imu_sensor.accZ();
            const float gx = imu_sensor.gyrX() - gyro_bias_x_cal;
            const float gy = imu_sensor.gyrY() - gyro_bias_y_cal;
            const float gz = imu_sensor.gyrZ() - gyro_bias_z_cal;
            const float mx = (imu_sensor.magX() - mag_calibration.offset_x) * mag_calibration.scale_x;
            const float my = (imu_sensor.magY() - mag_calibration.offset_y) * mag_calibration.scale_y;
            const float mz = (imu_sensor.magZ() - mag_calibration.offset_z) * mag_calibration.scale_z;

            set_raw(ax, ay, az, gx, gy, gz, mx, my, mz);
            if (IMU_USE_MAG_AHRS) {
                ahrs.update(gx, gy, gz, ax, ay, az, mx, my, mz);
                gyro_yaw_deg = ahrs.getYaw();
            } else {
                ahrs.updateIMU(gx, gy, gz, ax, ay, az);
                const uint32_t now_us = micros();
                const float dt_s = static_cast<float>(now_us - last_imu_update_us) * 0.000001f;
                last_imu_update_us = now_us;
                if (dt_s > 0.0f && dt_s < 0.1f) gyro_yaw_deg = wrap_deg_360(gyro_yaw_deg + gz * dt_s);
            }
            set_orientation(gyro_yaw_deg, ahrs.getPitch(), ahrs.getRoll());
        }

        const uint32_t status_interval_ms = imu_status_interval_ms();
        const uint32_t now_ms = millis();
        if (status_interval_ms > 0 && (last_status_ms == 0 || now_ms - last_status_ms >= status_interval_ms)) {
            last_status_ms = now_ms;
            loggf("imu status ok=%d yaw_ready=%d yaw=%.2f pitch=%.2f roll=%.2f\n",
                  is_ready() ? 1 : 0,
                  yaw_is_ready() ? 1 : 0,
                  yaw_deg(),
                  pitch_deg(),
                  roll_deg());
        }

        ThisThread::sleep_for(1ms);
    }
}

bool Imu::is_ready() const {
    mbed::CriticalSectionLock lock;
    return ready_value;
}

bool Imu::yaw_is_ready() const {
    mbed::CriticalSectionLock lock;
    return yaw_ready_value;
}

bool Imu::is_calibrating() const {
    mbed::CriticalSectionLock lock;
    return calibration_busy_value;
}

float Imu::yaw_deg() const {
    mbed::CriticalSectionLock lock;
    return yaw_deg_value;
}

float Imu::pitch_deg() const {
    mbed::CriticalSectionLock lock;
    return pitch_deg_value;
}

float Imu::roll_deg() const {
    mbed::CriticalSectionLock lock;
    return roll_deg_value;
}

float Imu::accel_x() const {
    mbed::CriticalSectionLock lock;
    return accel_x_value;
}

float Imu::accel_y() const {
    mbed::CriticalSectionLock lock;
    return accel_y_value;
}

float Imu::accel_z() const {
    mbed::CriticalSectionLock lock;
    return accel_z_value;
}

float Imu::gyro_x() const {
    mbed::CriticalSectionLock lock;
    return gyro_x_value;
}

float Imu::gyro_y() const {
    mbed::CriticalSectionLock lock;
    return gyro_y_value;
}

float Imu::gyro_z() const {
    mbed::CriticalSectionLock lock;
    return gyro_z_value;
}

float Imu::mag_x() const {
    mbed::CriticalSectionLock lock;
    return mag_x_value;
}

float Imu::mag_y() const {
    mbed::CriticalSectionLock lock;
    return mag_y_value;
}

float Imu::mag_z() const {
    mbed::CriticalSectionLock lock;
    return mag_z_value;
}

float Imu::gyro_bias_x() const {
    mbed::CriticalSectionLock lock;
    return gyro_bias_x_value;
}

float Imu::gyro_bias_y() const {
    mbed::CriticalSectionLock lock;
    return gyro_bias_y_value;
}

float Imu::gyro_bias_z() const {
    mbed::CriticalSectionLock lock;
    return gyro_bias_z_value;
}

void Imu::zero_yaw(float yaw) {
    mbed::CriticalSectionLock lock;
    gyro_yaw_deg = wrap_deg_360(yaw);
    yaw_deg_value = gyro_yaw_deg;
    yaw_ready_value = ready_value;
    last_imu_update_us = micros();
}

bool Imu::request_calibration(bool gyro, bool mag) {
    if (!gyro && !mag) return false;

    mbed::CriticalSectionLock lock;
    if (calibration_busy_value || gyro_calibration_requested || mag_calibration_requested) return false;

    gyro_calibration_requested = gyro;
    mag_calibration_requested = mag;
    return true;
}

void imu_begin() {
    imu.begin();
}

void func_imu_entry() {
    imu.entry();
}
