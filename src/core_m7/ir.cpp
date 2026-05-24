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
