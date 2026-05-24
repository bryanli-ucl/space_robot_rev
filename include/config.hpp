#pragma once

#include <Arduino.h>

namespace CONFIG {

namespace M4 {

// Serial logging
constexpr uint32_t SERIAL_BAUD = 115200;

constexpr uint32_t LED_INTERVAL_MS = 500;

// RTOS tasks
constexpr bool ENABLE_HEARTBEAT_TASK             = true;
constexpr bool ENABLE_SERIAL_DEBUG_TASK          = true;
constexpr bool ENABLE_CHASSIS_TASK               = true;
constexpr uint32_t HEARTBEAT_TASK_INTERVAL_MS    = 10;
constexpr uint32_t SERIAL_DEBUG_TASK_INTERVAL_MS = 50;
constexpr uint32_t CHASSIS_TASK_INTERVAL_MS      = 20;
constexpr uint32_t CHASSIS_STATUS_INTERVAL_MS    = 5000;

// Safety inputs and status LEDs. M4 owns these so the stop path stays close to the motor controller.
constexpr pin_size_t REVIVING_BUTTON_PIN     = A2;
constexpr pin_size_t KILLSWITCH_BUTTON_PIN   = A3;
constexpr pin_size_t STATUS_RED_LED_PIN      = LEDR;
constexpr pin_size_t STATUS_GREEN_LED_PIN    = LEDG;
constexpr pin_size_t STATUS_BLUE_LED_PIN     = LEDB;
constexpr uint8_t KILLSWITCH_DEBOUNCE_TICKS  = 20;
constexpr uint8_t HEARTBEAT_BLINK_TICKS      = 50;
constexpr uint32_t HEARTBEAT_LOG_INTERVAL_MS = 5000;

// Motor pins
constexpr pin_size_t MOTOR_RL_EN       = D2;
constexpr pin_size_t MOTOR_RL_FORWARD  = D25;
constexpr pin_size_t MOTOR_RL_BACKWARD = D27;
constexpr pin_size_t MOTOR_RL_ENC_A    = D33;
constexpr pin_size_t MOTOR_RL_ENC_B    = D32;

constexpr pin_size_t MOTOR_FL_EN       = D3;
constexpr pin_size_t MOTOR_FL_FORWARD  = D31;
constexpr pin_size_t MOTOR_FL_BACKWARD = D29;
constexpr pin_size_t MOTOR_FL_ENC_A    = D35;
constexpr pin_size_t MOTOR_FL_ENC_B    = D34;

constexpr pin_size_t MOTOR_FR_EN       = D4;
constexpr pin_size_t MOTOR_FR_FORWARD  = D24;
constexpr pin_size_t MOTOR_FR_BACKWARD = D26;
constexpr pin_size_t MOTOR_FR_ENC_A    = D36;
constexpr pin_size_t MOTOR_FR_ENC_B    = D37;

constexpr pin_size_t MOTOR_RR_EN       = D5;
constexpr pin_size_t MOTOR_RR_FORWARD  = D30;
constexpr pin_size_t MOTOR_RR_BACKWARD = D28;
constexpr pin_size_t MOTOR_RR_ENC_A    = D38;
constexpr pin_size_t MOTOR_RR_ENC_B    = D39;

// Chassis geometry and encoder conversion placeholders.
constexpr float CHASSIS_L              = 1.0f;
constexpr float CHASSIS_W              = 1.0f;
constexpr float CHASSIS_R              = 0.05f;
constexpr float COUNTS_PER_CM          = 120.0f;
constexpr float CHASSIS_YAW_KP         = 0.03f;
constexpr float CHASSIS_YAW_MAX_W_CORR = 0.4f;
constexpr float CHASSIS_MOVE_EPS       = 0.01f;
constexpr float CHASSIS_ROTATE_EPS     = 0.01f;

// Movement Control
constexpr float TURN_MAX_W              = 1.f;
constexpr float TURN_TOLERANCE_DEG      = 1.f;
constexpr uint32_t TURN_TIMEOUT_MS      = 5000;
constexpr float LINE_KP                 = 0.0100f;
constexpr float LINE_KD                 = 0.0040f;
constexpr float LINE_MAX_W              = 3.0f;
constexpr uint8_t LINE_CROSS_MIN_BLACK  = 7;
constexpr uint16_t LINE_BLACK_THRESHOLD = 700;
constexpr uint8_t LINE_CROSS_CONFIRM    = 2;
constexpr float WALL_KP                 = 0.12f;
constexpr float WALL_KD                 = 0.08f;
constexpr float WALL_YAW_KP             = 0.015f;
constexpr float WALL_MAX_W              = 2.5f;
constexpr float TURN_KP                 = 0.035f;
constexpr float TURN_KI                 = 0.0f;
constexpr float TURN_KD                 = 0.006f;
constexpr uint8_t TURN_STABLE_COUNT     = 8;

// IMU / AHRS
constexpr bool IMU_SCAN_I2C               = true;
constexpr bool IMU_AD0_VAL                = false;
constexpr float IMU_SAMPLE_HZ             = 100.0f;
constexpr uint8_t IMU_GYRO_SMPLRT_DIV     = 10;
constexpr uint16_t IMU_ACC_SMPLRT_DIV     = 10;
constexpr uint32_t IMU_GYRO_BIAS_CAL_MS   = 2000;
constexpr uint32_t IMU_STATUS_INTERVAL_MS = 5000;

// Set true to sample magnetometer min/max at boot and compute hard/soft iron correction.
constexpr bool IMU_MAG_CALIBRATE  = false;
constexpr uint32_t IMU_MAG_CAL_MS = 10000;
constexpr float MAG_OFF_X         = 457.575012f;
constexpr float MAG_OFF_Y         = 457.575012f;
constexpr float MAG_OFF_Z         = -33.150002f;
constexpr float MAG_SCALE_X       = 1.576635f;
constexpr float MAG_SCALE_Y       = 0.789238f;
constexpr float MAG_SCALE_Z       = 1.021528f;

} // namespace M4

namespace M7 {

// Serial logging
constexpr uint32_t SERIAL_BAUD = 115200;

// WiFi
constexpr const char* WIFI_SSID            = "BD4B Hyperoptic 1Gb Fibre 2.4Ghz";
constexpr const char* WIFI_PASS            = "3R9gfN4up9ar";
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
constexpr uint32_t WIFI_CONNECT_POLL_MS    = 200;

// MQTT / MiniMessenger
constexpr const char* MQTT_HOST = "192.168.1.120";
constexpr uint16_t MQTT_PORT    = 1883;
constexpr const char* GROUP_ID  = "12";
constexpr const char* BOARD_ID  = "12";
constexpr const char* SERVER_ID = "server";

// Loop intervals
constexpr uint32_t WIFI_SCAN_INTERVAL_MS     = 30000;
constexpr uint32_t STATUS_INTERVAL_MS        = 5000;
constexpr uint32_t LED_INTERVAL_MS           = 500;
constexpr uint32_t HEARTBEAT_LOG_INTERVAL_MS = 5000;

// M7 <-> M4 RPC benchmark
constexpr bool ENABLE_RPC_BENCHMARK           = true;
constexpr uint16_t RPC_BENCHMARK_CALLS        = 50;
constexpr uint32_t RPC_BENCHMARK_BOOT_WAIT_MS = 1000;
constexpr uint32_t RPC_FAST_PUSH_INTERVAL_MS  = 20;
constexpr uint32_t RPC_SLOW_PUSH_INTERVAL_MS  = 100;

// Sensors
constexpr bool ENABLE_IR_UPDATE         = true;
constexpr bool ENABLE_ULTRASONIC_UPDATE = true;
constexpr bool ENABLE_RFID_UPDATE       = true;
constexpr bool ENABLE_RPC_UPDATE        = true;
constexpr bool ENABLE_SUNLIGHT_UPDATE   = true;
constexpr bool ENABLE_WIFI_UPDATE       = true;

constexpr pin_size_t SUNLIGHT_PIN              = A1;
constexpr uint32_t SUNLIGHT_SAMPLE_INTERVAL_MS = 100;

constexpr pin_size_t IR_CTRL_O_PIN                     = D14;
constexpr pin_size_t IR_CTRL_E_PIN                     = D15;
constexpr uint8_t IR_SENSOR_COUNT                      = 9;
constexpr uint32_t IR_SAMPLE_INTERVAL_MS               = 50;
constexpr uint16_t IR_RC_TIMEOUT_US                    = 1000;
constexpr uint32_t IR_WIFI_SETTLE_MS                   = 3000;
constexpr uint16_t IR_CALIBRATION_MIN[IR_SENSOR_COUNT] = { 37, 24, 26, 29, 27, 29, 27, 28, 26 };
constexpr uint16_t IR_CALIBRATION_MAX[IR_SENSOR_COUNT] = { 564, 355, 356, 405, 383, 415, 432, 488, 490 };
constexpr uint8_t IR_SENSOR_PINS[IR_SENSOR_COUNT]      = { D45, D46, D47, D48, D49, D50, D51, D52, D53 };

constexpr pin_size_t ULTRASONIC_FRONT_TRIG_PIN   = D40;
constexpr pin_size_t ULTRASONIC_FRONT_ECHO_PIN   = D41;
constexpr pin_size_t ULTRASONIC_LEFT_TRIG_PIN    = D42;
constexpr pin_size_t ULTRASONIC_LEFT_ECHO_PIN    = D43;
constexpr pin_size_t ULTRASONIC_RIGHT_TRIG_PIN   = D22;
constexpr pin_size_t ULTRASONIC_RIGHT_ECHO_PIN   = D23;
constexpr uint16_t ULTRASONIC_MAX_DISTANCE_CM    = 250;
constexpr uint32_t ULTRASONIC_TIMEOUT_US         = 15000;
constexpr uint32_t ULTRASONIC_SAMPLE_INTERVAL_MS = 60;
constexpr float ULTRASONIC_TEMPERATURE_C         = 25.0f;

constexpr uint8_t RFID_I2C_ADDR            = 0x28;
constexpr uint32_t RFID_SAMPLE_INTERVAL_MS = 100;
constexpr uint32_t RFID_REPEAT_COOLDOWN_MS = 200;

} // namespace M7

} // namespace CONFIG
