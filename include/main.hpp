#pragma once

// Includes

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <mbed.h>

#include "USBHostHID/USBHostMouse.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <map>
#include <memory>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// My peripherals

#include "MadgwickAHRS.h"
#include "bash.hpp"
#include "chassis.hpp"
#include <HCSR04.h>
#include <ICM_20948.h>
#include <MFRC522_I2C.h>
#include <QTRSensors.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

// Constants

enum class I2C_ADDR : uint8_t {
    RFID = 0x28,
    IMU  = 0x02,
};

enum class PINS : uint8_t {
    // Motors
    MOTOR_RL_EN       = D2,
    MOTOR_RL_FORWARD  = D25,
    MOTOR_RL_BACKWARD = D27,
    MOTOR_RL_ENC_A    = D33,
    MOTOR_RL_ENC_B    = D32,

    MOTOR_FL_EN       = D3,
    MOTOR_FL_FORWARD  = D31,
    MOTOR_FL_BACKWARD = D29,
    MOTOR_FL_ENC_A    = D35,
    MOTOR_FL_ENC_B    = D34,

    MOTOR_FR_EN       = D4,
    MOTOR_FR_FORWARD  = D24,
    MOTOR_FR_BACKWARD = D26,
    MOTOR_FR_ENC_A    = D36,
    MOTOR_FR_ENC_B    = D37,

    MOTOR_RR_EN       = D5,
    MOTOR_RR_FORWARD  = D30,
    MOTOR_RR_BACKWARD = D28,
    MOTOR_RR_ENC_A    = D38,
    MOTOR_RR_ENC_B    = D39,

    // Ultrasonics

    US_FRONT_TRIG = D40,
    US_FRONT_ECHO = D41,
    US_LEFT_TRIG  = D42,
    US_LEFT_ECHO  = D43,
    US_RIGHT_TRIG = D22,
    US_RIGHT_ECHO = D23,

    // Buttons and LEDs

    // RED_LED_PIN   = LEDR,
    // GREED_LED_PIN = LEDG,
    // BLUE_LED_PIN  = LEDB,

    RED_LED_PIN   = D14,
    GREED_LED_PIN = D15,
    BLUE_LED_PIN  = LEDB,

    REVIVING_BUTTON_PIN   = A2,
    KILLSWITCH_BUTTON_PIN = A3,

    // Sun Light
    SUN_LIGHT_ADC_PIN = A1,

    // IR Arrays
    IR_CTRL_O = D18,
    IR_CTRL_E = D19,
    IR_1      = D45,
    IR_2      = D46,
    IR_3      = D47,
    IR_4      = D48,
    IR_5      = D49,
    IR_6      = D50,
    IR_7      = D51,
    IR_8      = D52,
    IR_9      = D53,
};

namespace CONFIG {

// Pre-calibrated values for 9 sensors
constexpr uint16_t qtr_min[9] = { 37, 24, 26, 29, 27, 29, 27, 28, 26 };
constexpr uint16_t qtr_max[9] = { 564, 355, 356, 405, 383, 415, 432, 488, 490 };

// Robot info
constexpr int ROBOT_ID = 12;

// Server info
// constexpr const char* SSID      = "BD4B Hyperoptic 1Gb Fibre 2.4Ghz";
constexpr const char* SSID = "PhaseSpaceNetwork_2.4G";
// constexpr const char* PWD       = "3R9gfN4up9ar";
constexpr const char* PWD = "8igMacNet";
// constexpr const char* SERVER_IP = "192.168.1.120";
constexpr const char* SERVER_IP = "192.168.0.211";
constexpr int SERVER_PORT       = 8080;

// IMU Magnetometer Calibration (hard-iron / soft-iron)
// Run calibration once, read the printed values from serial, then update these constants.
constexpr float IMU_SAMPLE_HZ           = 100.0f;
constexpr uint8_t IMU_GYRO_SMPLRT_DIV   = 10; // 1.1kHz / (1 + 10) = 100Hz
constexpr uint16_t IMU_ACC_SMPLRT_DIV   = 10; // 1.125kHz / (1 + 10) ~= 102.27Hz
constexpr uint32_t IMU_GYRO_BIAS_CAL_MS = 2000;
constexpr bool IMU_MAG_CALIBRATE        = false;
constexpr uint32_t IMU_MAG_CAL_MS       = 10000;
constexpr float LINE_KP                 = 0.0100f;
constexpr float LINE_KD                 = 0.0040f;
constexpr float LINE_BASE_VX            = 12.0f;
constexpr float LINE_MIN_VX             = 5.0f;
constexpr float LINE_MAX_W              = 3.0f;
constexpr float WALL_KP                 = 0.12f;
constexpr float WALL_KD                 = 0.08f;
constexpr float WALL_BASE_VX            = 9.0f;
constexpr float WALL_MAX_W              = 2.5f;
constexpr int16_t WALL_FRONT_STOP_CM    = 12;
constexpr float MAG_OFF_X               = 457.575012f;
constexpr float MAG_OFF_Y               = 457.575012f;
constexpr float MAG_OFF_Z               = -33.150002f;
constexpr float MAG_SCALE_X             = 1.576635f;
constexpr float MAG_SCALE_Y             = 0.789238f;
constexpr float MAG_SCALE_Z             = 1.021528f;

}; // namespace CONFIG

namespace MOUSE {
constexpr int LEFT  = 0b0001;
constexpr int RIGHT = 0b0010;
constexpr int MID   = 0b0100;
}; // namespace MOUSE

// Global Vars

extern Mail<std::array<char, 256>, 64> mail_udp_cmd;
extern Mail<std::array<char, 256>, 64> mail_serial_debug;
extern Mail<std::array<char, 256>, 64> mail_wifi_tx;

enum class LEDStatus {
    ON,
    OFF,
    BLINK,
};

// Mouse
extern volatile uint8_t mbutton;
extern volatile int32_t mx;
extern volatile int32_t my;
extern volatile int8_t mz;

// US
extern volatile int16_t dist_front;
extern volatile int16_t dist_left;
extern volatile int16_t dist_right;

// Wall Follow
extern volatile int8_t wall_follow_side;
extern volatile float wall_follow_target_cm;

// IR
extern uint16_t ir_vals[9];
extern volatile uint16_t ir_pos;

// RFID
extern volatile uint32_t detected_uid;

// IMU
extern Madgwick ahrs;
extern volatile float imu_yaw_deg;
extern volatile bool imu_yaw_ready;

// LED
extern LEDStatus led_red;
extern LEDStatus led_green;
extern LEDStatus led_blue;

// Sun Light
extern uint16_t sun_light;

// State Machine Define
enum class ButtonState {
    STOPPED,
    REVIVING,
    IDLE,
};
extern ButtonState button_state;

enum class MotionState {
    IDLE,
    LINE_FOLLOW,
    WALL_FOLLOW,
    MOUSE_FOLLOW,
};
extern MotionState motion_state;


// Devices
extern USBHostMouse mouse;

extern class Motor mfl;
extern class Motor mfr;
extern class Motor mrl;
extern class Motor mrr;
extern class Chassis chassis;

extern UltraSonicDistanceSensor usf;
extern UltraSonicDistanceSensor usl;
extern UltraSonicDistanceSensor usr;

extern QTRSensors qtr;

extern MFRC522_I2C rfid;

extern ICM_20948_I2C imu;

extern WiFiUDP udp;


// Function Prototype
void serial_tx(const char* fmt, ...);
void wifi_tx(const char* fmt, ...);
void command_tx(const char* fmt, ...);
