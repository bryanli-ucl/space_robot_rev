#pragma once

#include <Arduino.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// =========================== Serial and Shell =============================
constexpr int SERIAL_BAUD = 115200;

// ============================== Wireless ==================================
// SSID and Password Setting
constexpr const char* WIFI_SSID = "PhaseSpaceNetwork_2.4G";
constexpr const char* WIFI_PASS = "8igMacNet";

// constexpr const char* WIFI_SSID = "BD4B Hyperoptic 1Gb Fibre 2.4Ghz";
// constexpr const char* WIFI_PASS = "3R9gfN4up9ar";

// constexpr const char* WIFI_SSID = "李彦伯’s iPhone";
// constexpr const char* WIFI_PASS = "lyb20061127";

// Wifi Config
constexpr uint16_t WIRELESS_DEBUG_PORT            = 7777;
constexpr size_t WIRELESS_LINE_SIZE               = 256;
constexpr uint32_t WIFI_CONNECT_RETRY_MS          = 5000;
constexpr uint32_t WIRELESS_LOG_POLL_MS           = 20;
constexpr uint32_t WIRELESS_TELEMETRY_INTERVAL_MS = 1000;

// ================================ RFID ====================================
constexpr uint8_t RFID_I2C_ADDR            = 0x28;
constexpr uint32_t RFID_SAMPLE_INTERVAL_MS = 100;
constexpr uint32_t RFID_REPEAT_COOLDOWN_MS = 200;
constexpr uint32_t RFID_VALID_MS           = 1000;

// ================================ Motors ==================================
// Motor Pins
constexpr pin_size_t MOTOR_RL_EN       = D2;
constexpr pin_size_t MOTOR_RL_FORWARD  = D31;
constexpr pin_size_t MOTOR_RL_BACKWARD = D25;
constexpr pin_size_t MOTOR_RL_ENC_A    = D39;
constexpr pin_size_t MOTOR_RL_ENC_B    = D38;

constexpr pin_size_t MOTOR_FL_EN       = D3;
constexpr pin_size_t MOTOR_FL_FORWARD  = D27;
constexpr pin_size_t MOTOR_FL_BACKWARD = D29;
constexpr pin_size_t MOTOR_FL_ENC_A    = D36;
constexpr pin_size_t MOTOR_FL_ENC_B    = D37;

constexpr pin_size_t MOTOR_RR_EN       = D4;
constexpr pin_size_t MOTOR_RR_FORWARD  = D24;
constexpr pin_size_t MOTOR_RR_BACKWARD = D28;
constexpr pin_size_t MOTOR_RR_ENC_A    = D33;
constexpr pin_size_t MOTOR_RR_ENC_B    = D32;

constexpr pin_size_t MOTOR_FR_EN       = D5;
constexpr pin_size_t MOTOR_FR_FORWARD  = D26;
constexpr pin_size_t MOTOR_FR_BACKWARD = D30;
constexpr pin_size_t MOTOR_FR_ENC_A    = D35;
constexpr pin_size_t MOTOR_FR_ENC_B    = D34;

// Velocity Controller
constexpr int MOTOR_PWM_DEADBAND       = 30;
constexpr int MOTOR_PWM_START          = 220;
constexpr int MOTOR_PWM_START_MAX      = 255;
constexpr float MOTOR_PWM_START_RAMP_S = 0.20f;
constexpr int MOTOR_PWM_RUN            = 40;
constexpr float MOTOR_MOVING_SPEED_TH  = 8.0f;
constexpr float MOTOR_PID_KP           = 0.50f;
constexpr float MOTOR_PID_KI           = 2.80f;
constexpr float MOTOR_PID_KD           = 0.00f;
constexpr float MOTOR_SPEED_KF         = 0.18f;
constexpr float MOTOR_INTEGRAL_LIMIT   = 1000.0f;

// Chassis and Wheel Config
constexpr uint32_t CHASSIS_TASK_INTERVAL_MS    = 20;
constexpr float CHASSIS_MAX_WHEEL_SPEED        = 600.0f;
constexpr float WHEEL_DIAMETER_CM              = 6.0f;
constexpr float CHASSIS_ENCODER_COUNTS_PER_REV = 224.0f;
constexpr float CHASSIS_ENCODER_COUNTS_PER_CM  = CHASSIS_ENCODER_COUNTS_PER_REV / (3.1415926f * WHEEL_DIAMETER_CM);

// Turn Controller
constexpr float TURN_DIRECTION       = -1.0f;
constexpr float TURN_KP              = 5.0f;
constexpr float TURN_KD              = 0.20f;
constexpr float TURN_MIN_WHEEL_SPEED = 180.0f;
constexpr float TURN_MAX_WHEEL_SPEED = 560.0f;
constexpr float TURN_SLOW_ZONE_DEG   = 8.0f;
constexpr float TURN_TOLERANCE_DEG   = 5.0f;
constexpr float TURN_STOP_SPEED_DPS  = 12.0f;
constexpr uint8_t TURN_CONFIRM_COUNT = 5;
constexpr uint32_t TURN_TIMEOUT_MS   = 8000;

// Line Follower
constexpr uint16_t LINE_CENTER_POS           = 4000;
constexpr uint16_t LINE_BLACK_THRESHOLD      = 700;
constexpr uint8_t LINE_NO_LINE_MAX_BLACK     = 0;
constexpr uint8_t LINE_NO_LINE_CONFIRM       = 15;
constexpr uint8_t LINE_CROSS_CONFIRM         = 2;
constexpr uint8_t LINE_CORNER_CONFIRM        = 3;
constexpr int16_t LINE_DEFAULT_FRONT_STOP_CM = 12;
constexpr float LINE_DIRECTION               = 1.0f;
constexpr float LINE_KP                      = 0.330f;
constexpr float LINE_KD                      = 0.180f;
constexpr float LINE_MAX_WHEEL_SPEED         = 560.0f;
constexpr float LINE_MIN_SPEED_SCALE         = 0.30f;
constexpr uint32_t LINE_STATUS_INTERVAL_MS   = 500;

// Wall Follower
constexpr int16_t WALL_DEFAULT_TARGET_CM   = -1;
constexpr int16_t WALL_FRONT_STOP_CM       = 12;
constexpr float WALL_DIST_KP               = 3.0f;
constexpr float WALL_YAW_KP                = 10.0f;
constexpr float WALL_MAX_WHEEL_SPEED       = 220.0f;
constexpr float WALL_LEFT_DIRECTION        = -1.0f;
constexpr float WALL_RIGHT_DIRECTION       = 1.0f;
constexpr uint8_t WALL_LOST_CONFIRM        = 50;
constexpr uint32_t WALL_STATUS_INTERVAL_MS = 500;

// Trial Task Defaults
constexpr float TASK_LINE_SPEED             = 150.0f;
constexpr float TASK_DRIVE_SPEED            = 140.0f;
constexpr float TASK_RAMP_SPEED             = 180.0f;
constexpr float TASK_WALL_SPEED             = 100.0f;
constexpr float TASK_REVIVE_SPEED           = 70.0f;
constexpr float TASK_OPEN_FIELD_DISTANCE_CM = 125.0f;
constexpr float TASK_RAMP_DISTANCE_CM       = 120.0f;
constexpr float TASK6_WALL_SPEED            = 150.0f;
constexpr float TASK6_WALL_DISTANCE_CM      = 120.0f;
constexpr int16_t TASK6_WALL_TARGET_CM      = WALL_DEFAULT_TARGET_CM;
constexpr float TASK_REVIVE_MAX_DISTANCE_CM = 80.0f;
constexpr int16_t TASK_OBSTACLE_FRONT_CM    = 18;
constexpr int16_t TASK_REVIVE_FRONT_CM      = 7;

// Task 1
constexpr float TASK1_LINE_FOLLOW_SPEED        = 150.0f;
constexpr int32_t TASK1_LINE_FOLLOW_TIMEOUT_MS = 90 * 1000;

// Task 4
constexpr float TASK4_NODE_CM             = 30.0f;
constexpr float TASK4_DRIVE_SPEED         = 150.0f;
constexpr float TASK4_RIGHT_TURN_DEG      = 90.0f;
constexpr float TASK4_LEFT_TURN_DEG       = -90.0f;
constexpr uint32_t TASK4_ACTION_SETTLE_MS = 300;

// Task 2
constexpr float TASK2_CENTER_DRIVE_CM = 10.0f;
constexpr float TASK2_LEFT_TURN_DEG   = -90.0f;
constexpr float TASK2_RIGHT_TURN_DEG  = 90.0f;
constexpr int16_t TASK2_DOOR_FRONT_CM = 10;
constexpr uint32_t TASK2_RFID_WAIT_MS = 2000;

// Task 3
constexpr uint8_t TASK3_FIRST_STRAIGHT_NODES  = 2;
constexpr uint8_t TASK3_MIDDLE_STRAIGHT_NODES = 1;
constexpr uint8_t TASK3_LAST_STRAIGHT_NODES   = 2;

// Task 4
// Task 5
// Task 6

// Task 7
constexpr float TASK7_CROSS_CLEAR_CM     = 8.0f;
constexpr uint8_t TASK7_DETECT_MAX_NODES = 2;
constexpr uint8_t TASK7_SIDE_NODES       = 1;
constexpr uint8_t TASK7_PASS_NODES       = 2;
constexpr uint8_t TASK7_FINISH_NODES     = 3;

// Task 8
constexpr float TASK8_FAST_SPEED         = 150.0f;
constexpr float TASK8_MID_SPEED          = 100.0f;
constexpr float TASK8_CONTACT_SPEED      = 80.0f;
constexpr int16_t TASK8_MID_FRONT_CM     = 20;
constexpr int16_t TASK8_CONTACT_FRONT_CM = 10;
constexpr float TASK8_MAX_DISTANCE_CM    = 120.0f;
constexpr uint32_t TASK8_TIMEOUT_MS      = 60000;

// ================================ Mission =================================
constexpr float MISSION_TURN_IMU_RATIO            = 0.50f;
constexpr float MISSION_LINE_SEARCH_W             = 300.0f;
constexpr uint32_t MISSION_LINE_SEARCH_IGNORE_MS  = 300;
constexpr uint32_t MISSION_LINE_SEARCH_TIMEOUT_MS = 5000;
constexpr uint8_t MISSION_LINE_SEARCH_BLACK_COUNT = 2;
constexpr uint8_t MISSION_LINE_SEARCH_CONFIRM     = 3;
constexpr uint32_t MISSION_LINE_TIMEOUT_MS        = 20000;
constexpr uint32_t MISSION_DRIVE_TIMEOUT_MS       = 8000;
constexpr uint32_t MISSION_TURN_TIMEOUT_MS        = 8000;

// ================================ Sensors =================================
constexpr pin_size_t REVIVE_BUTTON_PIN      = D44;
constexpr pin_size_t KILL_SWITCH_PIN        = D10;
constexpr pin_size_t STATE_LED_RED_PIN      = D17;
constexpr pin_size_t STATE_LED_GREEN_PIN    = D16;
constexpr uint32_t STATE_STOPPED_BLINK_MS   = 500;
constexpr uint8_t KILL_SWITCH_CONFIRM_COUNT = 5;

// Ultrasonic
constexpr pin_size_t ULTRASONIC_FRONT_TRIG_PIN   = D40;
constexpr pin_size_t ULTRASONIC_FRONT_ECHO_PIN   = D41;
constexpr pin_size_t ULTRASONIC_LEFT_TRIG_PIN    = D43;
constexpr pin_size_t ULTRASONIC_LEFT_ECHO_PIN    = D42;
constexpr pin_size_t ULTRASONIC_RIGHT_TRIG_PIN   = D9;
constexpr pin_size_t ULTRASONIC_RIGHT_ECHO_PIN   = D8;
constexpr uint16_t ULTRASONIC_MAX_DISTANCE_CM    = 250;
constexpr uint32_t ULTRASONIC_TIMEOUT_US         = 15000;
constexpr uint32_t ULTRASONIC_SAMPLE_INTERVAL_MS = 150;
constexpr float ULTRASONIC_LOW_PASS_ALPHA        = 0.75f;
constexpr uint8_t ULTRASONIC_INVALID_HOLD_COUNT  = 5;

// IR Sensors
constexpr pin_size_t IR_CTRL_O_PIN                     = D18;
constexpr pin_size_t IR_CTRL_E_PIN                     = D19;
constexpr uint8_t IR_SENSOR_COUNT                      = 9;
constexpr uint32_t IR_SAMPLE_INTERVAL_MS               = 50;
constexpr uint16_t IR_RC_TIMEOUT_US                    = 1000;
constexpr uint16_t IR_CALIBRATION_MIN[IR_SENSOR_COUNT] = { 37, 24, 26, 29, 27, 29, 27, 28, 26 };
constexpr uint16_t IR_CALIBRATION_MAX[IR_SENSOR_COUNT] = { 564, 355, 356, 405, 383, 415, 432, 488, 490 };
constexpr uint8_t IR_SENSOR_PINS[IR_SENSOR_COUNT]      = { D45, D46, D47, D48, D49, D50, D51, D52, D53 };
constexpr pin_size_t IR_SIDE_LEFT_CTRL_PIN             = A7;
constexpr uint8_t IR_SIDE_LEFT_SENSOR_PIN              = A6;
constexpr pin_size_t IR_SIDE_RIGHT_CTRL_PIN            = A4;
constexpr uint8_t IR_SIDE_RIGHT_SENSOR_PIN             = A5;
constexpr uint16_t IR_SIDE_LEFT_BLACK_THRESHOLD        = 750;
constexpr uint16_t IR_SIDE_RIGHT_BLACK_THRESHOLD       = 150;
constexpr uint32_t SENSOR_TASK_INTERVAL_MS             = 10;

// IMU / AHRS
constexpr bool IMU_SCAN_I2C               = true;
constexpr bool IMU_AD0_VAL                = false;
constexpr float IMU_SAMPLE_HZ             = 100.0f;
constexpr uint8_t IMU_GYRO_SMPLRT_DIV     = 10;
constexpr uint16_t IMU_ACC_SMPLRT_DIV     = 10;
constexpr uint32_t IMU_GYRO_BIAS_CAL_MS   = 5000;
constexpr uint32_t IMU_STATUS_INTERVAL_MS = 0;
constexpr bool IMU_MAG_CALIBRATE          = false;
constexpr uint32_t IMU_MAG_CAL_MS         = 15000;
constexpr bool IMU_USE_MAG_AHRS           = false;
constexpr float MAG_OFF_X                 = -17.100000f;
constexpr float MAG_OFF_Y                 = -158.100006f;
constexpr float MAG_OFF_Z                 = -41.700001f;
constexpr float MAG_SCALE_X               = 1.133848f;
constexpr float MAG_SCALE_Y               = 0.932275f;
constexpr float MAG_SCALE_Z               = 0.956569f;
