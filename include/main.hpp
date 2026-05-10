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
#include <tuple>
#include <type_traits>

// My peripherals

#include "MadgwickAHRS.h"
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
    MOTOR_FL_EN       = D2,
    MOTOR_FL_FORWARD  = D25,
    MOTOR_FL_BACKWARD = D27,
    MOTOR_FL_ENC_A    = D32,
    MOTOR_FL_ENC_B    = D33,

    MOTOR_FR_EN       = D3,
    MOTOR_FR_FORWARD  = D29,
    MOTOR_FR_BACKWARD = D31,
    MOTOR_FR_ENC_A    = D34,
    MOTOR_FR_ENC_B    = D35,

    MOTOR_RL_EN       = D4,
    MOTOR_RL_FORWARD  = D24,
    MOTOR_RL_BACKWARD = D26,
    MOTOR_RL_ENC_A    = D36,
    MOTOR_RL_ENC_B    = D37,

    MOTOR_RR_EN       = D5,
    MOTOR_RR_FORWARD  = D28,
    MOTOR_RR_BACKWARD = D30,
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

    RED_LED_PIN   = LEDR,
    GREED_LED_PIN = LEDG,
    BLUE_LED_PIN  = LEDB,

    REVIVING_BUTTON_PIN   = D22,
    KILLSWITCH_BUTTON_PIN = D23,

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
constexpr const char* SSID      = "BD4B Hyperoptic 1Gb Fibre 2.4Ghz";
constexpr const char* PWD       = "3R9gfN4up9ar";
constexpr const char* SERVER_IP = "192.168.1.120";
constexpr int SERVER_PORT       = 8080;

// IMU Magnetometer Calibration (hard-iron / soft-iron)
// Run calibration once, read the printed values from serial, then update these constants.
constexpr float MAG_OFF_X   = 0.0f;
constexpr float MAG_OFF_Y   = 0.0f;
constexpr float MAG_OFF_Z   = 0.0f;
constexpr float MAG_SCALE_X = 1.0f;
constexpr float MAG_SCALE_Y = 1.0f;
constexpr float MAG_SCALE_Z = 1.0f;

}; // namespace CONFIG

namespace MOUSE {
constexpr int LEFT  = 0b0001;
constexpr int RIGHT = 0b0010;
constexpr int MID   = 0b0100;
}; // namespace MOUSE

// Global Vars

extern Mail<std::array<char, 256>, 64> mail_udp_cmd;
extern Mail<std::array<char, 256>, 64> mail_serial_debug;

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

// IR
extern uint16_t ir_vals[9];
extern volatile uint16_t ir_pos;

// RFID
extern volatile uint32_t detected_uid;

// IMU

// LED
extern LEDStatus led_red;
extern LEDStatus led_green;
extern LEDStatus led_blue;

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
extern Madgwick ahrs;

extern WiFiUDP udp;


// Function Prototype
void serial_tx(const char* fmt, ...);
