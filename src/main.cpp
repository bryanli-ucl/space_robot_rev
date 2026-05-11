#include "main.hpp"

Thread task_heartbeat(osPriorityBelowNormal7);
Thread task_serial_debug(osPriorityBelowNormal4);
Thread task_sensors(osPriorityBelowNormal3);
Thread task_mission(osPriorityAboveNormal2);
Thread task_wifi_server(osPriorityAboveNormal3);
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

MFRC522_I2C rfid(static_cast<byte>(I2C_ADDR::RFID), static_cast<byte>(-1), &Wire1);

ICM_20948_I2C imu;
Madgwick ahrs;

WiFiUDP udp;

// Function Prototypes
void serial_tx(const char* fmt, ...);

void func_heartbeat();
void func_serial_debug();
void func_wifi_server();
void func_chassis();
void func_mission();
void func_sensors();

// State Machine Define
ButtonState button_state = ButtonState::STOPPED;
MotionState motion_state = MotionState::IDLE;

void setup() {
    task_heartbeat.start(func_heartbeat);
    task_serial_debug.start(func_serial_debug);
    task_wifi_server.start(func_wifi_server);
    task_chassis.start(func_chassis);
    task_mission.start(func_mission);
    task_sensors.start(func_sensors);
}

void loop() {
    ThisThread::sleep_for(10s);
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

// IR
uint16_t ir_vals[9];
volatile uint16_t ir_pos;

// RFID
volatile uint32_t detected_uid = 0;

// IMU
volatile float yaw = 0;

// LED
LEDStatus led_red   = LEDStatus::OFF;
LEDStatus led_green = LEDStatus::OFF;
LEDStatus led_blue  = LEDStatus::BLINK;


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

    // RFID
    serial_tx("RFID Init\n");
    Wire1.begin();

    // rfid.PCD_Init();
    // if (!rfid.PCD_PerformSelfTest()) {
    //     serial_tx("RFID Cannot Pass Self Test\n");
    // }

    // IMU
    serial_tx("IMU Init\n");
    if (imu.begin(Wire1) != ICM_20948_Stat_Ok) {
        serial_tx("IMU init fail\n");
    }

    ahrs.begin(100.0f);

    // Load pre-calibrated magnetometer params (update these in main.hpp after running calibration)
    float mag_scale_x = CONFIG::MAG_SCALE_X;
    float mag_scale_y = CONFIG::MAG_SCALE_Y;
    float mag_scale_z = CONFIG::MAG_SCALE_Z;

    float mag_off_x = CONFIG::MAG_OFF_X;
    float mag_off_y = CONFIG::MAG_OFF_Y;
    float mag_off_z = CONFIG::MAG_OFF_Z;

    char imu_calibrate = 1;
    if (imu_calibrate) {
        float min_x = INT_MAX, max_x = INT_MIN;
        float min_y = INT_MAX, max_y = INT_MIN;
        float min_z = INT_MAX, max_z = INT_MIN;

        serial_tx("imu_calibrate Begin\n");

        unsigned long start = millis();
        while (millis() - start < 10000) {
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

        if (avg_r > 0.1f) {
            mag_scale_x = avg_r / radius_x;
            mag_scale_y = avg_r / radius_y;
            mag_scale_z = avg_r / radius_z;
        }

        serial_tx("imu_calibrate Done\n");
        serial_tx("Offset (uT): %f %f %f\n", mag_off_x, mag_off_y, mag_off_z);
        serial_tx("Scale  (uT): %f %f %f\n", mag_scale_x, mag_scale_y, mag_scale_z);
    }

    // Mouse
    serial_tx("Mouse Init\n");
    mouse.attachButtonEvent([](uint8_t btn) { mbutton = btn; });
    mouse.attachXEvent([](int8_t v) { mx += v; });
    mouse.attachYEvent([](int8_t v) { my -= v; });
    mouse.attachZEvent([](int8_t v) { mz += v; });


    while (1) {

        // IMU
        if (imu.dataReady()) {
            imu.getAGMT();

            float ax = imu.accX();
            float ay = imu.accY();
            float az = imu.accZ();

            float gx = imu.gyrX();
            float gy = imu.gyrY();
            float gz = imu.gyrZ();

            float mx = (imu.magX() - mag_off_x) * mag_scale_x;
            float my = (imu.magY() - mag_off_y) * mag_scale_y;
            float mz = (imu.magZ() - mag_off_z) * mag_scale_z;

            ahrs.update(gx, gy, gz, ax, ay, az, mx, my, mz);
            yaw = ahrs.getYaw();
        }

        static int slow_cnt = 0;
        if (slow_cnt-- == 0) {
            slow_cnt = 10;

            // IR Array
            ir_pos = qtr.readLineBlack(ir_vals);

            // Ultrasonic
            dist_front = usf.measureDistanceCm(us_temperature);
            dist_left  = usl.measureDistanceCm(us_temperature);
            dist_right = usr.measureDistanceCm(us_temperature);

            // RFID
            // if (rfid.PICC_IsNewCardPresent()) {
            //     if (rfid.PICC_ReadCardSerial()) {
            //         detected_uid = reinterpret_cast<uint32_t*>(rfid.uid.uidByte)[0];
            //         serial_tx("New Tag Detected, size: %d, uid: %d\n", rfid.uid.size, detected_uid);
            //     }
            // }

            // Mouse
            if (!mouse.connected()) {
                if (mouse.connect()) {
                    serial_tx("Mouse connected\n");
                }
            }

            if (mbutton & MOUSE::LEFT) {
                mx = 0, my = 0, mz = 0;
            }
            if (mbutton & MOUSE::RIGHT) {}
            if (mbutton & MOUSE::MID) {}
        }

        ThisThread::sleep_for(10ms);
    }
}

void func_chassis() {
    chassis.set_paras(1.f, 1.f, 0.05f);
    chassis.set_target(0, 0, 0);

    int cnt = 200;
    while (1) {
        chassis.update(5ms);
        if (--cnt == 0) {
            cnt = 10;
            // serial_tx("chassis: %f %f %f %f, mouse: %d %d\n", chassis.get_vfl(), chassis.get_vfr(), chassis.get_vrl(), chassis.get_vrr(), mx, my);
        }
        ThisThread::sleep_for(5ms);
    }
}

// ============================================================
// ================== About Debug and Display =================
// ============================================================

Mail<std::array<char, 256>, 64> mail_udp_cmd;
void func_wifi_server() {
    if (CONFIG::SSID == nullptr) {
        serial_tx("Please set CONFIG::SSID in main.hpp\n");
        return;
    }

    // Connect WiFi
    if (WiFi.status() == WL_CONNECTED) {
        serial_tx("WiFi already connected, IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        WiFi.begin(CONFIG::SSID, CONFIG::PWD);
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 100) {
            ThisThread::sleep_for(100ms);
            attempts++;
        }
        if (WiFi.status() != WL_CONNECTED) {
            serial_tx("WiFi connection failed\n");
            return;
        }
        serial_tx("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
    }

    // Start UDP
    udp.begin(CONFIG::SERVER_PORT);
    serial_tx("UDP started on port %d, target PC: %s:%d\n",
    CONFIG::SERVER_PORT, CONFIG::SERVER_IP, CONFIG::SERVER_PORT);

    while (1) {
        // Send queued wifi_tx messages
        while (!mail_wifi_tx.empty()) {
            std::array<char, 256>* msg = mail_wifi_tx.try_get();
            if (msg == nullptr) break;
            udp.beginPacket(CONFIG::SERVER_IP, CONFIG::SERVER_PORT);
            udp.write((const uint8_t*)msg->data(), strlen(msg->data()));
            udp.endPacket();
            mail_wifi_tx.free(msg);
        }

        // Receive data from PC
        int packet_size = udp.parsePacket();
        if (packet_size) {
            std::array<char, 256>* mail = mail_udp_cmd.try_alloc();

            if (mail == nullptr) {
                udp.readString();
                continue;
            }

            udp.read(mail->data(), mail->max_size() * sizeof(std::remove_pointer<decltype(mail->data())>::type));
            serial_tx("UDP RX [%s:%d]: %s\n", udp.remoteIP().toString().c_str(), udp.remotePort(), mail->data());
            mail_udp_cmd.put(mail);
        }

        ThisThread::sleep_for(100ms);
    }
}

Mail<std::array<char, 256>, 64> mail_serial_debug;
Mail<std::array<char, 256>, 64> mail_wifi_tx;

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

void func_serial_debug() {
    Serial1.begin(115200);
    ThisThread::sleep_for(100ms);

    while (1) {
        while (!mail_serial_debug.empty()) {
            std::array<char, 256>* msg = mail_serial_debug.try_get();
            if (msg == nullptr) break;
            Serial1.write(msg->data());
            mail_serial_debug.free(msg);
        }
        ThisThread::sleep_for(100ms);
    }
}

void func_heartbeat() {

    // led signals

    GPIO_TypeDef* RED_LED_GPIOX   = GPIOI;
    GPIO_TypeDef* GREEN_LED_GPIOX = GPIOJ;
    GPIO_TypeDef* BLUE_LED_GPIOX  = GPIOE;

    constexpr int RED_LED_GPIO_PIN   = GPIO_PIN_12;
    constexpr int GREEN_LED_GPIO_PIN = GPIO_PIN_13;
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
