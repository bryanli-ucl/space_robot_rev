#include "main.hpp"
#include "tasks.hpp"

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

// State Machine Define
ButtonState button_state = ButtonState::STOPPED;
MotionState motion_state = MotionState::IDLE;

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

namespace {

void setup_main_loop_led() {
    if (!CONFIG::ENABLE_MAIN_LOOP_LED) {
        return;
    }

    pinMode((int)PINS::BLUE_LED_PIN, OUTPUT);
    digitalWrite((int)PINS::BLUE_LED_PIN, HIGH);
}

void loop_main_loop_led() {
    if (!CONFIG::ENABLE_MAIN_LOOP_LED) {
        return;
    }

    static unsigned long last_toggle_ms = 0;
    static bool led_on = false;

    if (millis() - last_toggle_ms >= 500 || last_toggle_ms == 0) {
        last_toggle_ms = millis();
        led_on = !led_on;
        digitalWrite((int)PINS::BLUE_LED_PIN, led_on ? LOW : HIGH);
    }
}

void start_configured_tasks() {
    if (CONFIG::ENABLE_TASK_HEARTBEAT) {
        task_heartbeat.start(func_heartbeat);
    }

    if (CONFIG::ENABLE_TASK_CHASSIS) {
        task_chassis.start(func_chassis);
    }

    if (CONFIG::ENABLE_TASK_MISSION) {
        task_mission.start(func_mission);
    }

    if (CONFIG::ENABLE_TASK_SENSORS) {
        task_sensors.start(func_sensors);
    }

    if (CONFIG::ENABLE_TASK_IMU) {
        task_imu.start(func_imu);
    }

    if (CONFIG::ENABLE_TASK_RFID) {
        task_rfid.start(func_rfid);
    }
}

} // namespace

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    delay(200);

    setup_main_loop_led();

    if (CONFIG::ENABLE_TASK_SERIAL_DEBUG) {
        task_serial_debug.start(func_serial_debug);
    }

    setup_mqtt_messenger();
    start_configured_tasks();
}

void loop() {
    loop_main_loop_led();
    loop_mqtt_messenger();
    delay(10);
}
