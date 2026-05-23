#include "m7_led.hpp"

#include <Arduino.h>

#include "config.hpp"


void led_begin() {
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDB, HIGH);
}

void led_update(uint32_t now_ms) {

    static uint32_t last_led_ms = 0;
    static bool led_on          = false;

    if (now_ms - last_led_ms < CONFIG::M7::LED_INTERVAL_MS && last_led_ms != 0) {
        return;
    }

    last_led_ms = now_ms;
    led_on      = !led_on;
    digitalWrite(LEDB, led_on ? LOW : HIGH);
}
