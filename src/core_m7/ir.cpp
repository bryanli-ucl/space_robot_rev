#include "core_m7/ir.hpp"

#include "config.hpp"
#include "core_m7/serial.hpp"

#include <Arduino.h>
#include <QTRSensors.h>
#include <stdlib.h>

namespace {

QTRSensors qtr;
uint32_t last_sample_ms                      = 0;
uint16_t position                            = 0;
uint16_t values[CONFIG::M7::IR_SENSOR_COUNT] = {};
bool side_left_detected                      = false;
bool side_right_detected                     = false;

bool side_sensor_detected(pin_size_t pin) {
    const int raw = digitalRead(pin);
    return CONFIG::M7::IR_SIDE_ACTIVE_LOW ? raw == LOW : raw == HIGH;
}

} // namespace

void ir_begin() {
    loggf("[m7-ir] init\n");
    qtr.setTypeRC();
    qtr.setTimeout(CONFIG::M7::IR_RC_TIMEOUT_US);
    qtr.setEmitterPins(CONFIG::M7::IR_CTRL_O_PIN, CONFIG::M7::IR_CTRL_E_PIN);
    loggf("[m7-ir] emitter control enabled pins=%d/%d\n",
    CONFIG::M7::IR_CTRL_O_PIN,
    CONFIG::M7::IR_CTRL_E_PIN);
    qtr.setSensorPins(CONFIG::M7::IR_SENSOR_PINS, CONFIG::M7::IR_SENSOR_COUNT);
    pinMode(CONFIG::M7::IR_SIDE_LEFT_PIN, INPUT_PULLUP);
    pinMode(CONFIG::M7::IR_SIDE_RIGHT_PIN, INPUT_PULLUP);
    loggf("[m7-ir] side turn sensors left/right pins=%d/%d active_low=%d\n",
    CONFIG::M7::IR_SIDE_LEFT_PIN,
    CONFIG::M7::IR_SIDE_RIGHT_PIN,
    CONFIG::M7::IR_SIDE_ACTIVE_LOW ? 1 : 0);

    qtr.calibrationOn.minimum = static_cast<uint16_t*>(malloc(sizeof(uint16_t) * CONFIG::M7::IR_SENSOR_COUNT));
    qtr.calibrationOn.maximum = static_cast<uint16_t*>(malloc(sizeof(uint16_t) * CONFIG::M7::IR_SENSOR_COUNT));
    if (qtr.calibrationOn.minimum == nullptr || qtr.calibrationOn.maximum == nullptr) {
        loggf("[m7-ir] calibration allocation failed\n");
        return;
    }

    for (uint8_t i = 0; i < CONFIG::M7::IR_SENSOR_COUNT; i++) {
        qtr.calibrationOn.minimum[i] = CONFIG::M7::IR_CALIBRATION_MIN[i];
        qtr.calibrationOn.maximum[i] = CONFIG::M7::IR_CALIBRATION_MAX[i];
    }
    qtr.calibrationOn.initialized = true;
    loggf("[m7-ir] ready\n");
}

void ir_update(uint32_t now_ms) {
    if (now_ms - last_sample_ms < CONFIG::M7::IR_SAMPLE_INTERVAL_MS && last_sample_ms != 0) {
        return;
    }

    last_sample_ms             = now_ms;
    const uint32_t started_ms  = millis();
    position                   = qtr.readLineBlack(values);
    side_left_detected         = side_sensor_detected(CONFIG::M7::IR_SIDE_LEFT_PIN);
    side_right_detected        = side_sensor_detected(CONFIG::M7::IR_SIDE_RIGHT_PIN);
    const uint32_t duration_ms = millis() - started_ms;
    if (duration_ms > 10) {
        loggf("[m7-ir] read took %lums pos=%u\n",
        static_cast<unsigned long>(duration_ms),
        position);
    }
}

uint16_t ir_position() {
    return position;
}

const uint16_t* ir_values() {
    return values;
}

bool ir_side_left_detected() {
    return side_left_detected;
}

bool ir_side_right_detected() {
    return side_right_detected;
}
