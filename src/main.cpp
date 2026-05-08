#include "main.hpp"

Thread task_heartbeat(osPriorityBelowNormal7);
Thread task_serial_debug(osPriorityBelowNormal4);
Thread task_sensors(osPriorityBelowNormal3);
Thread task_mouse(osPriorityBelowNormal2);
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

// Function Prototypes
void serial_tx(const char* fmt, ...);

void func_heartbeat();
void func_serial_debug();
void func_wifi_server();
void func_mouse();
void func_chassis();
void func_mission();
void func_sensors();

void setup() {
    task_heartbeat.start(func_heartbeat);
    task_serial_debug.start(func_serial_debug);
    task_wifi_server.start(func_wifi_server);
    task_mouse.start(func_mouse);
    task_chassis.start(func_chassis);
    task_mission.start(func_mission);
    task_sensors.start(func_sensors);
}

void loop() {
    ThisThread::sleep_for(10s);
}

void func_mission() {
    ThisThread::sleep_for(1000ms);
    while (1) {
        chassis.set_target(mx / 1000.f, my / 1000.f, 0);
        ThisThread::sleep_for(10ms);
    }
}

void func_sensors() {
    qtr.setTypeRC();
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

    qtr.setEmitterPins((int)PINS::IR_CTRL_O, (int)PINS::IR_CTRL_E);

    ThisThread::sleep_for(1s);

    qtr.calibrationOn.minimum = (uint16_t*)malloc(sizeof(uint16_t) * 9);
    qtr.calibrationOn.maximum = (uint16_t*)malloc(sizeof(uint16_t) * 9);
    for (uint8_t i = 0; i < 9; i++) {
        qtr.calibrationOn.minimum[i] = CONFIG::qtr_min[i];
        qtr.calibrationOn.maximum[i] = CONFIG::qtr_max[i];
    }
    qtr.calibrationOn.initialized = true;

    while (1) {
        static uint16_t sensorValues[9];
        uint16_t position = qtr.readLineBlack(sensorValues);

        // print the sensor values as numbers from 0 to 1000, where 0 means maximum
        // reflectance and 1000 means minimum reflectance, followed by the line
        // position
        // for (uint8_t i = 0; i < 9; i++) {
        //     Serial.print(sensorValues[i]);
        //     Serial.print('\t');
        // }
        Serial.println(position);

        ThisThread::sleep_for(10ms);
    }
}

void func_chassis() {
    chassis.set_paras(1.f, 1.f, 0.05f);

    chassis.set_target(1, 0, 0);

    int cnt = 200;
    while (1) {
        chassis.update(5ms);
        if (--cnt == 0) {
            cnt = 10;
            serial_tx("chassis: %f %f %f %f, mouse: %d %d\n", chassis.get_vfl(), chassis.get_vfr(), chassis.get_vrl(), chassis.get_vrr(), mx, my);
        }
        ThisThread::sleep_for(5ms);
    }
}

void func_wifi_server() {
    if (CONFIG::SSID == nullptr) {
        serial_tx("Please set CONFIG::SSID in main.hpp\n");
        return;
    }

    if (WiFi.status() == WL_CONNECTED) {
        serial_tx("WiFi was connected, IP: %s\n", WiFi.localIP().toString().c_str());
        return;
    }

    WiFi.begin(CONFIG::SSID, CONFIG::PWD);
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

void func_serial_debug() {
    Serial1.begin(115200);
    delay(100);

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

volatile uint8_t mbutton = 0;
volatile int32_t mx      = 0;
volatile int32_t my      = 0;
volatile int8_t mz       = 0;

void func_mouse() {
    mouse.attachButtonEvent([](uint8_t btn) { mbutton = btn; });
    mouse.attachXEvent([](int8_t v) { mx += v; });
    mouse.attachYEvent([](int8_t v) { my -= v; });
    mouse.attachZEvent([](int8_t v) { mz += v; });

    while (1) {
        while (!mouse.connected()) {
            bool ret = mouse.connect();
            if (ret == true) {
                serial_tx("mouse connected");
            }
            ThisThread::sleep_for(100ms);
        }

        // serial_tx("mouse: x: %d, y: %d, btn: %d, z: %d\n", (int)mx, (int)my, (int)mbutton, (int)mz);

        ThisThread::sleep_for(1000ms);
    }
}

void func_heartbeat() {
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_BLUE, HIGH);

    while (1) {
        ThisThread::sleep_for(1s);
        digitalWrite(LED_BLUE, LOW);
        ThisThread::sleep_for(1s);
        digitalWrite(LED_BLUE, HIGH);
        serial_tx("Heart Beat: %ums\n", millis());
    }
}
