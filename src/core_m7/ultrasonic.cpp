#include "core_m7/ultrasonic.hpp"

#include "config.hpp"
#include "core_m7/serial.hpp"

#include <Arduino.h>
#include <HCSR04.h>

namespace {

UltraSonicDistanceSensor front_sensor(CONFIG::M7::ULTRASONIC_FRONT_TRIG_PIN,
CONFIG::M7::ULTRASONIC_FRONT_ECHO_PIN,
CONFIG::M7::ULTRASONIC_MAX_DISTANCE_CM,
CONFIG::M7::ULTRASONIC_TIMEOUT_US);
UltraSonicDistanceSensor left_sensor(CONFIG::M7::ULTRASONIC_LEFT_TRIG_PIN,
CONFIG::M7::ULTRASONIC_LEFT_ECHO_PIN,
CONFIG::M7::ULTRASONIC_MAX_DISTANCE_CM,
CONFIG::M7::ULTRASONIC_TIMEOUT_US);
UltraSonicDistanceSensor right_sensor(CONFIG::M7::ULTRASONIC_RIGHT_TRIG_PIN,
CONFIG::M7::ULTRASONIC_RIGHT_ECHO_PIN,
CONFIG::M7::ULTRASONIC_MAX_DISTANCE_CM,
CONFIG::M7::ULTRASONIC_TIMEOUT_US);

uint32_t last_sample_ms   = 0;
uint32_t last_duration_ms = 0;
uint8_t sensor_index      = 0;
int16_t front_cm          = -1;
int16_t left_cm           = -1;
int16_t right_cm          = -1;

int16_t normalize_distance_cm(float value) {
    if (value <= 0.0f || value > CONFIG::M7::ULTRASONIC_MAX_DISTANCE_CM) {
        return -1;
    }
    return static_cast<int16_t>(value);
}

} // namespace

void ultrasonic_begin() {
    loggf("[m7-us] ready\n");
}

void ultrasonic_update(uint32_t now_ms) {
    if (now_ms - last_sample_ms < CONFIG::M7::ULTRASONIC_SAMPLE_INTERVAL_MS && last_sample_ms != 0) {
        return;
    }

    last_sample_ms            = now_ms;
    const uint32_t started_ms = millis();

    if (sensor_index == 0) {
        front_cm = normalize_distance_cm(front_sensor.measureDistanceCm(CONFIG::M7::ULTRASONIC_TEMPERATURE_C));
    } else if (sensor_index == 1) {
        left_cm = normalize_distance_cm(left_sensor.measureDistanceCm(CONFIG::M7::ULTRASONIC_TEMPERATURE_C));
    } else {
        right_cm = normalize_distance_cm(right_sensor.measureDistanceCm(CONFIG::M7::ULTRASONIC_TEMPERATURE_C));
    }

    last_duration_ms = millis() - started_ms;
    sensor_index     = (sensor_index + 1) % 3;
}

int16_t ultrasonic_front_cm() {
    return front_cm;
}

int16_t ultrasonic_left_cm() {
    return left_cm;
}

int16_t ultrasonic_right_cm() {
    return right_cm;
}

uint32_t ultrasonic_last_duration_ms() {
    return last_duration_ms;
}
