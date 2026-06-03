#pragma once

#include "config.hpp"

#include <Arduino.h>
#include <stdint.h>

#include "hal/gpio_api.h"

class SeedDropper {
    public:
    static SeedDropper& instance();

    void begin();
    void start(uint8_t pwm = SEED_DROPPER_PWM_HIGH);
    void stop();
    void reset_count();
    bool drop_one(int32_t counts = SEED_DROPPER_DROP_COUNTS, uint32_t timeout_ms = SEED_DROPPER_DROP_TIMEOUT_MS);

    int32_t count() const { return enc_count; }
    uint8_t pwm() const { return pwm_value; }
    bool is_running() const { return running; }

    private:
    SeedDropper()                                  = default;
    SeedDropper(const SeedDropper&)                = delete;
    SeedDropper& operator=(const SeedDropper&)     = delete;

    static void isr(void* ptr);

    gpio_t enc_a_hal;
    gpio_t enc_b_hal;
    volatile int32_t enc_count = 0;
    uint8_t pwm_value          = 0;
    bool running              = false;
    bool initialized          = false;
};

extern SeedDropper& seed_dropper;

void seed_dropper_begin();
