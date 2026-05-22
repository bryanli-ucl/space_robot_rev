#include "tasks.hpp"

namespace {

void scan_i2c_bus(TwoWire& bus, const char* name) {
    serial_tx("I2C scan %s begin\n", name);

    bool found = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        bus.beginTransmission(addr);
        uint8_t err = bus.endTransmission();

        if (err == 0) {
            found = true;
            serial_tx("I2C %s found 0x%02X\n", name, addr);
        }
    }

    if (!found) {
        serial_tx("I2C %s found none\n", name);
    }
}

} // namespace

void func_sensors() {
    if (CONFIG::ENABLE_SENSOR_IR_ARRAY) {
        serial_tx("IR Array Init\n");
        qtr.setTypeRC();
        qtr.setEmitterPins((int)PINS::IR_CTRL_O, (int)PINS::IR_CTRL_E);
        qtr.setSensorPins((const uint8_t[]){
                          (int)PINS::IR_1,
                          (int)PINS::IR_2,
                          (int)PINS::IR_3,
                          (int)PINS::IR_4,
                          (int)PINS::IR_5,
                          (int)PINS::IR_6,
                          (int)PINS::IR_7,
                          (int)PINS::IR_8,
                          (int)PINS::IR_9,
                          },
        9);

        ThisThread::sleep_for(500ms);

        qtr.calibrationOn.minimum = (uint16_t*)malloc(sizeof(uint16_t) * 9);
        qtr.calibrationOn.maximum = (uint16_t*)malloc(sizeof(uint16_t) * 9);
        for (uint8_t i = 0; i < 9; i++) {
            qtr.calibrationOn.minimum[i] = CONFIG::qtr_min[i];
            qtr.calibrationOn.maximum[i] = CONFIG::qtr_max[i];
        }
        qtr.calibrationOn.initialized = true;
    }

    if (CONFIG::ENABLE_SENSOR_ULTRASONIC) {
        serial_tx("Ultrasonic Init\n");
    }

    if (CONFIG::ENABLE_SENSOR_SUN_LIGHT) {
        pinMode((int)PINS::SUN_LIGHT_ADC_PIN, INPUT);
    }

    if (CONFIG::ENABLE_SENSOR_MOUSE) {
        serial_tx("Mouse Init\n");
        mouse.attachButtonEvent([](uint8_t btn) { mbutton = btn; });
        mouse.attachXEvent([](int8_t v) { mx += v; });
        mouse.attachYEvent([](int8_t v) { my -= v; });
        mouse.attachZEvent([](int8_t v) { mz += v; });
    }

    constexpr float us_temperature = 25.f;

    while (1) {
        if (CONFIG::ENABLE_SENSOR_IR_ARRAY) {
            ir_pos = qtr.readLineBlack(ir_vals);
        }

        if (CONFIG::ENABLE_SENSOR_SUN_LIGHT) {
            sun_light = analogRead((int)PINS::SUN_LIGHT_ADC_PIN);
        }

        static int slow_cnt = 0;
        if (slow_cnt-- == 0) {
            slow_cnt = 10;

            if (CONFIG::ENABLE_SENSOR_ULTRASONIC) {
                dist_front = usf.measureDistanceCm(us_temperature);
                dist_left  = usl.measureDistanceCm(us_temperature);
                dist_right = usr.measureDistanceCm(us_temperature);
            }

            if (CONFIG::ENABLE_SENSOR_MOUSE) {
                if (!mouse.connected()) {
                    if (mouse.connect()) {
                        serial_tx("Mouse connected\n");
                    }
                }

                if (mbutton & MOUSE::LEFT) { mx = 0, my = 0, mz = 0; }
                if (mbutton & MOUSE::RIGHT) {}
                if (mbutton & MOUSE::MID) {}
            }
        }

        ThisThread::sleep_for(10ms);
    }
}

void func_imu() {
    ThisThread::sleep_for(200ms);

    serial_tx("IMU I2C Init\n");
    Wire1.begin();
    scan_i2c_bus(Wire1, "Wire1");

    serial_tx("IMU Init\n");
    bool imu_ok = false;
    if (imu.begin(Wire1, 0) != ICM_20948_Stat_Ok) {
        serial_tx("IMU init fail\n");
    } else {
        imu_ok = true;
        serial_tx("IMU init ok\n");

        ICM_20948_Status_e imu_stat = imu.setSampleMode(
        ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr,
        ICM_20948_Sample_Mode_Continuous);
        if (imu_stat != ICM_20948_Stat_Ok) {
            serial_tx("IMU setSampleMode fail: %s\n", imu.statusString(imu_stat));
        }

        ICM_20948_smplrt_t imu_sample_rate;
        imu_sample_rate.g = CONFIG::IMU_GYRO_SMPLRT_DIV;
        imu_sample_rate.a = CONFIG::IMU_ACC_SMPLRT_DIV;
        imu_stat          = imu.setSampleRate(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, imu_sample_rate);
        if (imu_stat != ICM_20948_Stat_Ok) {
            serial_tx("IMU setSampleRate fail: %s\n", imu.statusString(imu_stat));
        } else {
            serial_tx("IMU acc/gyr sample rate set to %.1fHz\n", CONFIG::IMU_SAMPLE_HZ);
        }

        ICM_20948_dlpcfg_t imu_dlpf;
        imu_dlpf.a = acc_d50bw4_n68bw8;
        imu_dlpf.g = gyr_d51bw2_n73bw3;
        imu_stat   = imu.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, imu_dlpf);
        if (imu_stat != ICM_20948_Stat_Ok) {
            serial_tx("IMU setDLPFcfg fail: %s\n", imu.statusString(imu_stat));
        }

        imu_stat = imu.enableDLPF(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, true);
        if (imu_stat != ICM_20948_Stat_Ok) {
            serial_tx("IMU enableDLPF fail: %s\n", imu.statusString(imu_stat));
        }
    }

    ahrs.begin(CONFIG::IMU_SAMPLE_HZ);

    float mag_scale_x = CONFIG::MAG_SCALE_X;
    float mag_scale_y = CONFIG::MAG_SCALE_Y;
    float mag_scale_z = CONFIG::MAG_SCALE_Z;

    float mag_off_x = CONFIG::MAG_OFF_X;
    float mag_off_y = CONFIG::MAG_OFF_Y;
    float mag_off_z = CONFIG::MAG_OFF_Z;

    float gyro_bias_x = 0.0f;
    float gyro_bias_y = 0.0f;
    float gyro_bias_z = 0.0f;

    if (imu_ok) {
        serial_tx("IMU gyro bias calibration begin, keep robot still\n");

        uint32_t gyro_bias_count = 0;
        unsigned long start      = millis();
        while (millis() - start < CONFIG::IMU_GYRO_BIAS_CAL_MS) {
            if (imu.dataReady()) {
                imu.getAGMT();
                gyro_bias_x += imu.gyrX();
                gyro_bias_y += imu.gyrY();
                gyro_bias_z += imu.gyrZ();
                gyro_bias_count++;
            }
            ThisThread::sleep_for(5ms);
        }

        if (gyro_bias_count > 0) {
            gyro_bias_x /= gyro_bias_count;
            gyro_bias_y /= gyro_bias_count;
            gyro_bias_z /= gyro_bias_count;
        }

        serial_tx("IMU gyro bias: %.4f %.4f %.4f dps, samples: %lu\n",
        gyro_bias_x, gyro_bias_y, gyro_bias_z, gyro_bias_count);

        if (CONFIG::IMU_MAG_CALIBRATE) {
            float min_x = INT_MAX, max_x = INT_MIN;
            float min_y = INT_MAX, max_y = INT_MIN;
            float min_z = INT_MAX, max_z = INT_MIN;

            serial_tx("imu_calibrate Begin\n");

            unsigned long start = millis();
            while (millis() - start < CONFIG::IMU_MAG_CAL_MS) {
                if (imu.dataReady()) {
                    imu.getAGMT();

                    float mx = imu.magX();
                    float my = imu.magY();
                    float mz = imu.magZ();

                    if (mx < min_x) min_x = mx;
                    if (mx > max_x) max_x = mx;
                    if (my < min_y) min_y = my;
                    if (my > max_y) max_y = my;
                    if (mz < min_z) min_z = mz;
                    if (mz > max_z) max_z = mz;
                }
                ThisThread::sleep_for(20ms);
            }

            mag_off_x = (min_x + max_x) / 2.0f;
            mag_off_y = (min_y + max_y) / 2.0f;
            mag_off_z = (min_z + max_z) / 2.0f;

            float radius_x = (max_x - min_x) / 2.0f;
            float radius_y = (max_y - min_y) / 2.0f;
            float radius_z = (max_z - min_z) / 2.0f;
            float avg_r    = (radius_x + radius_y + radius_z) / 3.0f;

            if (avg_r > 0.1f && radius_x > 0.1f && radius_y > 0.1f && radius_z > 0.1f) {
                mag_scale_x = avg_r / radius_x;
                mag_scale_y = avg_r / radius_y;
                mag_scale_z = avg_r / radius_z;
            }

            serial_tx("imu_calibrate Done\n");
            serial_tx("Offset (uT): %f %f %f\n", mag_off_x, mag_off_y, mag_off_z);
            serial_tx("Scale  (uT): %f %f %f\n", mag_scale_x, mag_scale_y, mag_scale_z);
        }
    } else {
        serial_tx("imu_calibrate skipped because IMU init failed\n");
    }

    while (1) {
        if (imu_ok && imu.dataReady()) {
            imu.getAGMT();

            float ax = imu.accX();
            float ay = imu.accY();
            float az = imu.accZ();

            float gx = imu.gyrX() - gyro_bias_x;
            float gy = imu.gyrY() - gyro_bias_y;
            float gz = imu.gyrZ() - gyro_bias_z;

            float mx = (imu.magX() - mag_off_x) * mag_scale_x;
            float my = (imu.magY() - mag_off_y) * mag_scale_y;
            float mz = (imu.magZ() - mag_off_z) * mag_scale_z;

            ahrs.update(gx, gy, gz, ax, ay, az, mx, my, mz);
            imu_yaw_deg   = ahrs.getYaw();
            imu_yaw_ready = true;
        }

        ThisThread::sleep_for(1ms);
    }
}

void func_rfid() {
    ThisThread::sleep_for(200ms);

    serial_tx("RFID I2C Init\n");
    Wire.begin();
    scan_i2c_bus(Wire, "Wire");

    serial_tx("RFID Init\n");
    rfid.PCD_Init();
    byte rfid_version = rfid.PCD_ReadRegister(rfid.VersionReg);
    serial_tx("RFID VersionReg: 0x%02X\n", rfid_version);
    if (!rfid.PCD_PerformSelfTest()) {
        serial_tx("RFID self test warning\n");
    } else {
        serial_tx("RFID self test ok\n");
    }
    rfid.PCD_Init();

    uint32_t last_uid                     = 0;
    unsigned long last_ms                 = 0;
    constexpr uint32_t repeat_cooldown_ms = 1000;

    while (1) {
        if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
            uint32_t uid      = reinterpret_cast<uint32_t*>(rfid.uid.uidByte)[0];
            unsigned long now = millis();

            if (uid != last_uid || now - last_ms > repeat_cooldown_ms) {
                detected_uid = uid;
                last_uid     = uid;
                last_ms      = now;

                serial_tx("New Tag Detected, size: %d, uid: %lu\n", rfid.uid.size, static_cast<unsigned long>(uid));
                wifi_tx("New Tag Detected, size: %d, uid: %lu\n", rfid.uid.size, static_cast<unsigned long>(uid));
            }
        }

        ThisThread::sleep_for(50ms);
    }
}

