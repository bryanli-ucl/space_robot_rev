#include "main.hpp"

#include <ctype.h>

Thread task_heartbeat(osPriorityBelowNormal7);
Thread task_serial_debug(osPriorityBelowNormal4);
Thread task_sensors(osPriorityBelowNormal3);
Thread task_imu(osPriorityAboveNormal5);
Thread task_rfid(osPriorityAboveNormal1);
Thread task_mission(osPriorityAboveNormal2);
Thread task_chassis(osPriorityAboveNormal4);

// Devices
USBHostMouse mouse;

Motor mfl((int)PINS::MOTOR_FL_EN, (int)PINS::MOTOR_FL_FORWARD, (int)PINS::MOTOR_FL_BACKWARD, (int)PINS::MOTOR_FL_ENC_A, (int)PINS::MOTOR_FL_ENC_B);
Motor mfr((int)PINS::MOTOR_FR_EN, (int)PINS::MOTOR_FR_FORWARD, (int)PINS::MOTOR_FR_BACKWARD, (int)PINS::MOTOR_FR_ENC_A, (int)PINS::MOTOR_FR_ENC_B);
Motor mrl((int)PINS::MOTOR_RL_EN, (int)PINS::MOTOR_RL_FORWARD, (int)PINS::MOTOR_RL_BACKWARD, (int)PINS::MOTOR_RL_ENC_A, (int)PINS::MOTOR_RL_ENC_B);
Motor mrr((int)PINS::MOTOR_RR_EN, (int)PINS::MOTOR_RR_FORWARD, (int)PINS::MOTOR_RR_BACKWARD, (int)PINS::MOTOR_RR_ENC_A, (int)PINS::MOTOR_RR_ENC_B);
Chassis chassis(mfl, mfr, mrl, mrr);

UltraSonicDistanceSensor usf((int)PINS::US_FRONT_TRIG, (int)PINS::US_FRONT_ECHO);
UltraSonicDistanceSensor usl((int)PINS::US_LEFT_TRIG, (int)PINS::US_LEFT_ECHO);
UltraSonicDistanceSensor usr((int)PINS::US_RIGHT_TRIG, (int)PINS::US_RIGHT_ECHO);

QTRSensors qtr;

MFRC522_I2C rfid(static_cast<byte>(I2C_ADDR::RFID), static_cast<byte>(-1), &Wire);

ICM_20948_I2C imu;

MiniMessenger messenger;

// Function Prototypes

void func_heartbeat();
void func_serial_debug();
void func_mqtt_messenger_tick();
void func_chassis();
void func_mission();
void func_sensors();
void func_imu();
void func_rfid();

// State Machine Define
ButtonState button_state = ButtonState::STOPPED;
MotionState motion_state = MotionState::IDLE;

void setup() {
    task_heartbeat.start(func_heartbeat);
    task_serial_debug.start(func_serial_debug);
    task_chassis.start(func_chassis);
    task_mission.start(func_mission);
    task_sensors.start(func_sensors);
    task_imu.start(func_imu);
    task_rfid.start(func_rfid);
}

void loop() {
    func_mqtt_messenger_tick();
    ThisThread::sleep_for(10ms);
}

// Field
struct FieldUnit {
    uint32_t uid;
    bool fertile;
    float dist_of_front;
    float dist_of_left;
    float dist_of_right;
};

std::array<std::array<FieldUnit, 9>, 9> field;

// Mouse
volatile uint8_t mbutton = 0;
volatile int32_t mx      = 0;
volatile int32_t my      = 0;
volatile int8_t mz       = 0;

// US
volatile int16_t dist_front = 0;
volatile int16_t dist_left  = 0;
volatile int16_t dist_right = 0;

// Wall Follow
volatile int8_t wall_follow_side     = 1;
volatile float wall_follow_target_cm = 20.0f;

// IR
uint16_t ir_vals[9];
volatile uint16_t ir_pos;

// RFID
volatile uint32_t detected_uid = 0;

// IMU
Madgwick ahrs;
volatile float imu_yaw_deg  = 0.0f;
volatile bool imu_yaw_ready = false;

// LED
LEDStatus led_red   = LEDStatus::OFF;
LEDStatus led_green = LEDStatus::OFF;
LEDStatus led_blue  = LEDStatus::BLINK;

// Sun Light
uint16_t sun_light;

static void scan_i2c_bus(TwoWire& bus, const char* name) {
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

void func_sensors() {

    // IR Sensors
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

    // Ultrasonic
    serial_tx("Ultrasonic Init\n");
    constexpr float us_temperature = 25.f;

    // Sun Light
    pinMode((int)PINS::SUN_LIGHT_ADC_PIN, INPUT);

    // Mouse
    serial_tx("Mouse Init\n");
    mouse.attachButtonEvent([](uint8_t btn) { mbutton = btn; });
    mouse.attachXEvent([](int8_t v) { mx += v; });
    mouse.attachYEvent([](int8_t v) { my -= v; });
    mouse.attachZEvent([](int8_t v) { mz += v; });

    while (1) {
        // IR Array
        ir_pos = qtr.readLineBlack(ir_vals);

        // Sun Light
        sun_light = analogRead((int)PINS::SUN_LIGHT_ADC_PIN);

        static int slow_cnt = 0;
        if (slow_cnt-- == 0) {
            slow_cnt = 10;

            // Ultrasonic
            dist_front = usf.measureDistanceCm(us_temperature);
            dist_left  = usl.measureDistanceCm(us_temperature);
            dist_right = usr.measureDistanceCm(us_temperature);

            // Mouse
            if (!mouse.connected()) {
                if (mouse.connect()) {
                    serial_tx("Mouse connected\n");
                }
            }

            if (mbutton & MOUSE::LEFT) { mx = 0, my = 0, mz = 0; }
            if (mbutton & MOUSE::RIGHT) {}
            if (mbutton & MOUSE::MID) {}
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

void func_chassis() {
    chassis.set_paras(1.f, 1.f, 0.05f);
    chassis.set_target(0, 0, 0);

    constexpr float yaw_kp     = 0.03f;
    constexpr float max_w_corr = 0.4f;
    constexpr float move_eps   = 0.01f;
    constexpr float rotate_eps = 0.01f;

    float yaw_ref      = 0.0f;
    bool yaw_ref_ready = false;

    auto wrap_deg = [](float angle) -> float {
        while (angle > 180.0f) {
            angle -= 360.0f;
        }
        while (angle < -180.0f) {
            angle += 360.0f;
        }
        return angle;
    };

    while (1) {
        float vx = chassis.get_target_vx();
        float vy = chassis.get_target_vy();
        float w  = chassis.get_target_w();

        if (imu_yaw_ready) {
            float yaw      = imu_yaw_deg;
            bool moving_xy = fabsf(vx) > move_eps || fabsf(vy) > move_eps;
            bool rotating  = fabsf(w) > rotate_eps;

            if (!yaw_ref_ready) {
                yaw_ref       = yaw;
                yaw_ref_ready = true;
            }

            if (button_state == ButtonState::IDLE && moving_xy && !rotating) {
                float yaw_err = wrap_deg(yaw_ref - yaw);
                float w_corr  = constrain(yaw_kp * yaw_err, -max_w_corr, max_w_corr);
                chassis.apply_target(vx, vy, w + w_corr);
            } else {
                yaw_ref = yaw;
                chassis.apply_target(vx, vy, w);
            }
        } else {
            chassis.apply_target(vx, vy, w);
        }

        chassis.update(20ms);
        ThisThread::sleep_for(20ms);
    }
}

// ============================================================
// ================== About Debug and Display =================
// ============================================================

Mail<std::array<char, 256>, 64> mail_serial_debug;
void serial_tx(const char* fmt, ...) {
    std::array<char, 256>* mail = mail_serial_debug.try_alloc();
    if (mail == nullptr) {
        return;
    }

    va_list args;
    va_start(args, fmt);
    vsnprintf(mail->data(), mail->size() * sizeof(std::remove_pointer<decltype(mail->data())>::type), fmt, args);
    va_end(args);

    mail_serial_debug.put(mail);
}

Mail<std::array<char, 256>, 64> mail_wifi_tx;
void wifi_tx(const char* fmt, ...) {
    std::array<char, 256>* mail = mail_wifi_tx.try_alloc();
    if (mail == nullptr) {
        return;
    }

    va_list args;
    va_start(args, fmt);
    vsnprintf(mail->data(), mail->size() * sizeof(std::remove_pointer<decltype(mail->data())>::type), fmt, args);
    va_end(args);

    mail_wifi_tx.put(mail);
}

void command_tx(const char* fmt, ...) {
    char buf[256];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    serial_tx("%s", buf);
    wifi_tx("%s", buf);
}

Mail<std::array<char, 256>, 64> mail_mqtt_cmd;

static volatile bool mqtt_safety_enabled = false;

static void stop_robot_for_mqtt_safety() {
    mqtt_safety_enabled = false;
    button_state        = ButtonState::STOPPED;
    motion_state        = MotionState::IDLE;
    chassis.set_target(0.0f, 0.0f, 0.0f);
}

static bool payload_starts_with_type(const char* msg) {
    while (*msg != '\0' && isspace(static_cast<unsigned char>(*msg))) {
        msg++;
    }
    return strncmp(msg, "type=", 5) == 0;
}

static const char* wifi_status_name(uint8_t status) {
    switch (status) {
    case WL_IDLE_STATUS:
        return "idle";
    case WL_NO_SSID_AVAIL:
        return "ssid unavailable";
    case WL_SCAN_COMPLETED:
        return "scan completed";
    case WL_CONNECTED:
        return "connected";
    case WL_CONNECT_FAILED:
        return "connect failed";
    case WL_CONNECTION_LOST:
        return "connection lost";
    case WL_DISCONNECTED:
        return "disconnected";
    default:
        return "unknown";
    }
}

static void on_mqtt_message(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
    char msg[256];
    size_t copy_len = (length < sizeof(msg) - 1) ? length : sizeof(msg) - 1;
    memcpy(msg, payload, copy_len);
    msg[copy_len] = '\0';

    serial_tx("MQTT RX [%s -> %s]: %s\n", metadata.fromBoardId, metadata.target, msg);

    if (strstr(msg, "type=heartbeat enable=1")) {
        if (!mqtt_safety_enabled) {
            serial_tx("MQTT safety: heartbeat enabled\n");
        }
        mqtt_safety_enabled = true;
        if (button_state == ButtonState::STOPPED) {
            button_state = ButtonState::IDLE;
        }
        return;
    }

    if (strstr(msg, "type=heartbeat enable=0")) {
        stop_robot_for_mqtt_safety();
        serial_tx("MQTT safety: heartbeat disabled\n");
        return;
    }

    if (strstr(msg, "type=emergency enabled=true") || strstr(msg, "type=disable enabled=false")) {
        stop_robot_for_mqtt_safety();
        serial_tx("MQTT safety: emergency/disable active\n");
        return;
    }

    if (payload_starts_with_type(msg)) {
        return;
    }

    std::array<char, 256>* mail = mail_mqtt_cmd.try_alloc();
    if (mail == nullptr) {
        serial_tx("MQTT RX queue full, command dropped\n");
        return;
    }

    strncpy(mail->data(), msg, mail->size() - 1);
    mail->data()[mail->size() - 1] = '\0';
    mail_mqtt_cmd.put(mail);
}

void func_mqtt_messenger_tick() {
    static bool initialized               = false;
    static bool missing_config_logged     = false;
    static unsigned long last_register_ms = 0;
    static unsigned long last_status_ms   = 0;
    static bool last_connected            = false;

    if (CONFIG::SSID == nullptr) {
        if (!missing_config_logged) {
            missing_config_logged = true;
            serial_tx("Please set CONFIG::SSID in main.hpp\n");
        }
        return;
    }

    if (!initialized) {
        if (WiFi.status() == WL_CONNECTED) {
            serial_tx("WiFi already connected, IP: %s\n", WiFi.localIP().toString().c_str());
        } else {
            ThisThread::sleep_for(100ms);
            serial_tx("WiFi connecting to %s\n", CONFIG::SSID);
            WiFi.begin(CONFIG::SSID, CONFIG::PWD);

            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 1000) {
                ThisThread::sleep_for(100ms);
                attempts++;
                if (attempts % 10 == 0) {
                    serial_tx("WiFi connect attempts: %d\n", attempts);
                }
            }

            if (WiFi.status() == WL_CONNECTED) {
                serial_tx("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
            } else {
                serial_tx("WiFi initial connect failed, status: %s\n", wifi_status_name(WiFi.status()));
            }
        }

        messenger.onMessage(on_mqtt_message);
        bool connected = messenger.begin(CONFIG::SSID,
        CONFIG::PWD,
        CONFIG::MQTT_BROKER_HOST,
        CONFIG::MQTT_BROKER_PORT,
        CONFIG::GROUP_ID,
        CONFIG::BOARD_ID);

        initialized    = true;
        last_connected = connected;

        serial_tx("MiniMessenger started on Arduino loop, broker: %s:%u, group: %s, board: %s, client: %s, connected: %d, error: %d\n",
        CONFIG::MQTT_BROKER_HOST,
        CONFIG::MQTT_BROKER_PORT,
        CONFIG::GROUP_ID,
        CONFIG::BOARD_ID,
        messenger.clientId(),
        connected,
        messenger.connectError());
    }

    messenger.loop();

    bool now_connected = messenger.isConnected();
    if (now_connected != last_connected) {
        last_connected = now_connected;
        serial_tx("MiniMessenger %s, IP: %s\n",
        now_connected ? "connected" : "disconnected",
        WiFi.localIP().toString().c_str());
    }

    if (millis() - last_status_ms > 5000 || last_status_ms == 0) {
        last_status_ms = millis();
        serial_tx("MiniMessenger status: wifi=%s mqtt=%d error=%d ip=%s broker=%s:%u client=%s\n",
        wifi_status_name(WiFi.status()),
        now_connected,
        messenger.connectError(),
        WiFi.localIP().toString().c_str(),
        CONFIG::MQTT_BROKER_HOST,
        CONFIG::MQTT_BROKER_PORT,
        messenger.clientId());
    }

    if (now_connected && (last_register_ms == 0 || millis() - last_register_ms > 10000)) {
        last_register_ms = millis();

        char reg[96];
        snprintf(reg,
        sizeof(reg),
        "type=register team_id=%s board_id=%s",
        CONFIG::GROUP_ID,
        CONFIG::BOARD_ID);
        bool sent = messenger.sendToBoard(CONFIG::SERVER_BOARD_ID, reg);
        serial_tx("MiniMessenger register %s: %s\n", sent ? "sent" : "failed", reg);
    }

    // Send queued wifi_tx messages to the challenge server.
    while (!mail_wifi_tx.empty()) {
        std::array<char, 256>* msg = mail_wifi_tx.try_get();
        if (msg == nullptr) break;
        bool sent = messenger.sendToBoard(CONFIG::SERVER_BOARD_ID, msg->data());
        if (!sent) {
            serial_tx("MiniMessenger TX failed: %s\n", msg->data());
        }
        mail_wifi_tx.free(msg);
    }
}

void func_serial_debug() {
    Serial1.begin(115200);
    ThisThread::sleep_for(100ms);

    std::array<char, 256> serial_cmd;
    size_t serial_cmd_len = 0;

    while (1) {
        while (!mail_serial_debug.empty()) {
            std::array<char, 256>* msg = mail_serial_debug.try_get();
            if (msg == nullptr) break;
            Serial1.write(msg->data());
            mail_serial_debug.free(msg);
        }

        while (Serial1.available() > 0) {
            char c = static_cast<char>(Serial1.read());

            if (c == '\r' || c == '\n') {
                if (serial_cmd_len == 0) {
                    continue;
                }

                serial_cmd[serial_cmd_len] = '\0';
                serial_tx("SERIAL RX: %s\n", serial_cmd.data());
                bash.execute(serial_cmd.data());
                serial_cmd_len = 0;
            } else if (c == '\b' || c == 0x7f) {
                if (serial_cmd_len > 0) {
                    serial_cmd_len--;
                }
            } else if (serial_cmd_len < serial_cmd.size() - 1) {
                serial_cmd[serial_cmd_len++] = c;
            } else {
                serial_cmd_len = 0;
                serial_tx("SERIAL RX overflow, command cleared.\n");
            }
        }

        ThisThread::sleep_for(10ms);
    }
}

void func_heartbeat() {

    // led signals

    GPIO_TypeDef* RED_LED_GPIOX   = GPIOG;
    GPIO_TypeDef* GREEN_LED_GPIOX = GPIOC;
    GPIO_TypeDef* BLUE_LED_GPIOX  = GPIOE;

    constexpr int RED_LED_GPIO_PIN   = GPIO_PIN_14;
    constexpr int GREEN_LED_GPIO_PIN = GPIO_PIN_7;
    constexpr int BLUE_LED_GPIO_PIN  = GPIO_PIN_3;

    pinMode((int)PINS::RED_LED_PIN, OUTPUT);
    pinMode((int)PINS::GREED_LED_PIN, OUTPUT);
    pinMode((int)PINS::BLUE_LED_PIN, OUTPUT);

    digitalWrite((int)PINS::RED_LED_PIN, HIGH);
    digitalWrite((int)PINS::GREED_LED_PIN, HIGH);
    digitalWrite((int)PINS::BLUE_LED_PIN, HIGH);

    volatile bool toggle = false;

    // Button Detect

    pinMode((int)PINS::REVIVING_BUTTON_PIN, INPUT_PULLUP);
    pinMode((int)PINS::KILLSWITCH_BUTTON_PIN, INPUT_PULLUP);

    int8_t kill_btn_stable = 0;

    while (1) {

        // Serial Heart beat
        static int heart_beat_cnt = 0;
        if (heart_beat_cnt-- == 0) {
            serial_tx("Heart Beat: %ums\n", millis());
            heart_beat_cnt = 500;
        }

        // led signals
        static int blink_cnt = 0;
        if (blink_cnt-- == 0) {
            toggle    = !toggle;
            blink_cnt = 50;
        }

        if (led_red == LEDStatus::OFF)
            HAL_GPIO_WritePin(RED_LED_GPIOX, RED_LED_GPIO_PIN, GPIO_PIN_SET);
        else if (led_red == LEDStatus::ON)
            HAL_GPIO_WritePin(RED_LED_GPIOX, RED_LED_GPIO_PIN, GPIO_PIN_RESET);
        else if (led_red == LEDStatus::BLINK)
            HAL_GPIO_WritePin(RED_LED_GPIOX, RED_LED_GPIO_PIN, toggle ? GPIO_PIN_SET : GPIO_PIN_RESET);

        if (led_green == LEDStatus::OFF)
            HAL_GPIO_WritePin(GREEN_LED_GPIOX, GREEN_LED_GPIO_PIN, GPIO_PIN_SET);
        else if (led_green == LEDStatus::ON)
            HAL_GPIO_WritePin(GREEN_LED_GPIOX, GREEN_LED_GPIO_PIN, GPIO_PIN_RESET);
        else if (led_green == LEDStatus::BLINK)
            HAL_GPIO_WritePin(GREEN_LED_GPIOX, GREEN_LED_GPIO_PIN, toggle ? GPIO_PIN_SET : GPIO_PIN_RESET);

        if (led_blue == LEDStatus::OFF)
            HAL_GPIO_WritePin(BLUE_LED_GPIOX, BLUE_LED_GPIO_PIN, GPIO_PIN_SET);
        else if (led_blue == LEDStatus::ON)
            HAL_GPIO_WritePin(BLUE_LED_GPIOX, BLUE_LED_GPIO_PIN, GPIO_PIN_RESET);
        else if (led_blue == LEDStatus::BLINK)
            HAL_GPIO_WritePin(BLUE_LED_GPIOX, BLUE_LED_GPIO_PIN, toggle ? GPIO_PIN_SET : GPIO_PIN_RESET);

        // buttons detection
        if (digitalRead((int)PINS::KILLSWITCH_BUTTON_PIN) == LOW) {
            // Kill Switch Pressed
            if (kill_btn_stable++ == 20) {
                if (button_state == ButtonState::STOPPED) {
                    button_state = ButtonState::IDLE;
                } else {
                    button_state = ButtonState::STOPPED;
                }
            }
        } else {
            // Kill Switch unPressed
            kill_btn_stable = 0;
            if (digitalRead((int)PINS::REVIVING_BUTTON_PIN) == LOW) {
                // REVIVING_BUTTON Pressed
                if (button_state == ButtonState::IDLE) {
                    button_state = ButtonState::REVIVING;
                }
            } else {
                // REVIVING_BUTTON unPressed
                if (button_state == ButtonState::REVIVING) {
                    button_state = ButtonState::IDLE;
                }
            }
        }

        ThisThread::sleep_for(10ms);
    }
}
