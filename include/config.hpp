#pragma once

#include <Arduino.h>

namespace CONFIG {

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

// Sensors
constexpr bool ENABLE_IR_UPDATE         = true;
constexpr bool ENABLE_ULTRASONIC_UPDATE = true;
constexpr bool ENABLE_RFID_UPDATE       = true;
constexpr uint32_t LOOP_STEP_WARN_MS    = 20;

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
constexpr uint32_t RFID_REPEAT_COOLDOWN_MS = 1000;

} // namespace M7

} // namespace CONFIG
