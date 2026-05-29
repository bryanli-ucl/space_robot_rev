#include "sensors.hpp"

#include "config.hpp"
#include "logger.hpp"
#include "rfid.hpp"

#include <Arduino.h>
#include <QTRSensors.h>
#include <mbed.h>
#include <stdlib.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

Sensors& sensors = Sensors::instance();

extern void sensor_watch_update();

static QTRSensors qtr;
static QTRSensors qtr_side_left;
static QTRSensors qtr_side_right;
static uint8_t ir_side_left_pin[1] = { IR_SIDE_LEFT_SENSOR_PIN };
static uint8_t ir_side_right_pin[1] = { IR_SIDE_RIGHT_SENSOR_PIN };

static int16_t measure_ultrasonic_cm(pin_size_t trig, pin_size_t echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    const unsigned long duration_us = pulseIn(echo, HIGH, ULTRASONIC_TIMEOUT_US);
    if (duration_us == 0) return -1;

    const int16_t cm = static_cast<int16_t>(duration_us / 58UL);
    if (cm <= 0 || cm > ULTRASONIC_MAX_DISTANCE_CM) return -1;
    return cm;
}

static int16_t low_pass_ultrasonic_cm(int16_t raw_cm, float& filtered_cm, uint8_t& invalid_count) {
    if (raw_cm < 0) {
        if (filtered_cm >= 0.0f && invalid_count < ULTRASONIC_INVALID_HOLD_COUNT) {
            invalid_count++;
            return static_cast<int16_t>(filtered_cm + 0.5f);
        }

        filtered_cm = -1.0f;
        return -1;
    }

    invalid_count = 0;

    if (filtered_cm < 0.0f) {
        filtered_cm = static_cast<float>(raw_cm);
    } else if (fabsf(static_cast<float>(raw_cm) - filtered_cm) > ULTRASONIC_MAX_VALID_JUMP_CM) {
        return static_cast<int16_t>(filtered_cm + 0.5f);
    } else {
        filtered_cm += ULTRASONIC_LOW_PASS_ALPHA * (static_cast<float>(raw_cm) - filtered_cm);
    }

    return static_cast<int16_t>(filtered_cm + 0.5f);
}

Sensors& Sensors::instance() {
    static Sensors instance;
    return instance;
}

void Sensors::begin() {
    qtr.setTypeRC();
    qtr.setTimeout(IR_RC_TIMEOUT_US);
    qtr.setEmitterPins(IR_CTRL_O_PIN, IR_CTRL_E_PIN);
    qtr.setSensorPins(IR_SENSOR_PINS, IR_SENSOR_COUNT);

    qtr_side_left.setTypeRC();
    qtr_side_left.setTimeout(IR_RC_TIMEOUT_US);
    qtr_side_left.setEmitterPin(IR_SIDE_LEFT_CTRL_PIN);
    qtr_side_left.setSensorPins(ir_side_left_pin, 1);

    qtr_side_right.setTypeRC();
    qtr_side_right.setTimeout(IR_RC_TIMEOUT_US);
    qtr_side_right.setEmitterPin(IR_SIDE_RIGHT_CTRL_PIN);
    qtr_side_right.setSensorPins(ir_side_right_pin, 1);

    qtr.calibrationOn.minimum = static_cast<uint16_t*>(malloc(sizeof(uint16_t) * IR_SENSOR_COUNT));
    qtr.calibrationOn.maximum = static_cast<uint16_t*>(malloc(sizeof(uint16_t) * IR_SENSOR_COUNT));
    if (qtr.calibrationOn.minimum == nullptr || qtr.calibrationOn.maximum == nullptr) {
        loggf("ir calibration allocation failed\n");
    } else {
        for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
            qtr.calibrationOn.minimum[i] = IR_CALIBRATION_MIN[i];
            qtr.calibrationOn.maximum[i] = IR_CALIBRATION_MAX[i];
        }
        qtr.calibrationOn.initialized = true;
    }

    pinMode(ULTRASONIC_FRONT_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_FRONT_ECHO_PIN, INPUT);
    pinMode(ULTRASONIC_LEFT_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_LEFT_ECHO_PIN, INPUT);
    pinMode(ULTRASONIC_RIGHT_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_RIGHT_ECHO_PIN, INPUT);

    digitalWrite(ULTRASONIC_FRONT_TRIG_PIN, LOW);
    digitalWrite(ULTRASONIC_LEFT_TRIG_PIN, LOW);
    digitalWrite(ULTRASONIC_RIGHT_TRIG_PIN, LOW);

    loggf("sensors ready ultrasonic ir\n");
}

void Sensors::update_ultrasonic() {
    const uint32_t now_ms = millis();
    if (last_ultrasonic_sample_ms != 0 && now_ms - last_ultrasonic_sample_ms < ULTRASONIC_SAMPLE_INTERVAL_MS) return;
    last_ultrasonic_sample_ms = now_ms;

    const uint32_t started_ms = millis();

    if (ultrasonic_index == 0) {
        front_raw_cm = measure_ultrasonic_cm(ULTRASONIC_FRONT_TRIG_PIN, ULTRASONIC_FRONT_ECHO_PIN);
        front_cm = low_pass_ultrasonic_cm(front_raw_cm, front_filtered_cm, front_invalid_count);
    } else if (ultrasonic_index == 1) {
        left_raw_cm = measure_ultrasonic_cm(ULTRASONIC_LEFT_TRIG_PIN, ULTRASONIC_LEFT_ECHO_PIN);
        left_cm = low_pass_ultrasonic_cm(left_raw_cm, left_filtered_cm, left_invalid_count);
    } else {
        right_raw_cm = measure_ultrasonic_cm(ULTRASONIC_RIGHT_TRIG_PIN, ULTRASONIC_RIGHT_ECHO_PIN);
        right_cm = low_pass_ultrasonic_cm(right_raw_cm, right_filtered_cm, right_invalid_count);
    }

    ultrasonic_duration_ms = millis() - started_ms;
    ultrasonic_index = (ultrasonic_index + 1) % 3;
}

void Sensors::update_ir() {
    const uint32_t now_ms = millis();
    if (last_ir_sample_ms != 0 && now_ms - last_ir_sample_ms < IR_SAMPLE_INTERVAL_MS) return;
    last_ir_sample_ms = now_ms;

    const uint32_t started_ms = millis();
    ir_pos = qtr.readLineBlack(ir_vals);

    uint16_t side_left_values[1] = {};
    uint16_t side_right_values[1] = {};
    qtr_side_left.read(side_left_values);
    qtr_side_right.read(side_right_values);

    ir_left_value = side_left_values[0];
    ir_right_value = side_right_values[0];
    ir_left_detected = ir_left_value >= IR_SIDE_LEFT_BLACK_THRESHOLD;
    ir_right_detected = ir_right_value >= IR_SIDE_RIGHT_BLACK_THRESHOLD;
    ir_duration_ms = millis() - started_ms;
}

void sensors_begin() {
    sensors.begin();
}

void func_sensors_entry() {
    while (true) {
        sensors.update_ir();
        sensors.update_ultrasonic();
        rfid_update();
        sensor_watch_update();
        ThisThread::sleep_for(std::chrono::milliseconds(SENSOR_TASK_INTERVAL_MS));
    }
}
