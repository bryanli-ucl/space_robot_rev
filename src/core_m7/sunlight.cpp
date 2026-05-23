#include "m7_sunlight.hpp"

#include "config.hpp"

#include <Arduino.h>

namespace {

uint32_t last_sample_ms = 0;
uint16_t current_value = 0;

} // namespace

void sunlight_begin() {
    pinMode(CONFIG::M7::SUNLIGHT_PIN, INPUT);
    current_value = analogRead(CONFIG::M7::SUNLIGHT_PIN);
}

void sunlight_update(uint32_t now_ms) {
    if (now_ms - last_sample_ms < CONFIG::M7::SUNLIGHT_SAMPLE_INTERVAL_MS &&
        last_sample_ms != 0) {
        return;
    }

    last_sample_ms = now_ms;
    current_value = analogRead(CONFIG::M7::SUNLIGHT_PIN);
}

uint16_t sunlight_value() {
    return current_value;
}
