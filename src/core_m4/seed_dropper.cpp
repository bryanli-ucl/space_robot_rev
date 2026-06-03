#include "seed_dropper.hpp"

#include "logger.hpp"
#include "state.hpp"

#include <mbed.h>

using namespace ::std::chrono_literals;
using namespace ::rtos;

SeedDropper& seed_dropper = SeedDropper::instance();

SeedDropper& SeedDropper::instance() {
    static SeedDropper instance;
    return instance;
}

void SeedDropper::begin() {
    if (initialized) return;

    pinMode(SEED_DROPPER_PWM_PIN, OUTPUT);
    pinMode(SEED_DROPPER_ENC_A_PIN, INPUT_PULLUP);
    pinMode(SEED_DROPPER_ENC_B_PIN, INPUT_PULLUP);

    gpio_init_in_ex(&enc_a_hal, digitalPinToPinName(SEED_DROPPER_ENC_A_PIN), PullUp);
    gpio_init_in_ex(&enc_b_hal, digitalPinToPinName(SEED_DROPPER_ENC_B_PIN), PullUp);

    analogWrite(SEED_DROPPER_PWM_PIN, 0);
    attachInterruptParam(digitalPinToInterrupt(SEED_DROPPER_ENC_A_PIN), &SeedDropper::isr, CHANGE, this);

    initialized = true;
    loggf("seed dropper ready pwm=%u enc=%u/%u\n",
    static_cast<unsigned>(SEED_DROPPER_PWM_PIN),
    static_cast<unsigned>(SEED_DROPPER_ENC_A_PIN),
    static_cast<unsigned>(SEED_DROPPER_ENC_B_PIN));
}

void SeedDropper::start(uint8_t pwm) {
    if (!initialized) begin();

    pwm_value = pwm;
    running   = pwm_value > 0;
    analogWrite(SEED_DROPPER_PWM_PIN, pwm_value);
}

void SeedDropper::stop() {
    pwm_value = 0;
    running   = false;
    analogWrite(SEED_DROPPER_PWM_PIN, 0);
}

void SeedDropper::reset_count() {
    enc_count = 0;
}

bool SeedDropper::drop_one(int32_t counts, uint32_t timeout_ms) {
    if (counts <= 0) counts = SEED_DROPPER_DROP_COUNTS;
    if (timeout_ms == 0) timeout_ms = SEED_DROPPER_DROP_TIMEOUT_MS;

    reset_count();
    start(SEED_DROPPER_PWM_HIGH);

    const uint32_t start_ms = millis();
    while (millis() - start_ms < timeout_ms) {
        if (running_state == RunningState::STOPPED) {
            stop();
            loggf("seed drop stopped by state count=%ld target=%ld\n",
            static_cast<long>(enc_count),
            static_cast<long>(counts));
            return false;
        }

        if (labs(enc_count) >= counts) {
            stop();
            loggf("seed drop done count=%ld target=%ld elapsed=%lums\n",
            static_cast<long>(enc_count),
            static_cast<long>(counts),
            static_cast<unsigned long>(millis() - start_ms));
            return true;
        }

        ThisThread::sleep_for(5ms);
    }

    stop();
    loggf("seed drop timeout count=%ld target=%ld timeout=%lums\n",
    static_cast<long>(enc_count),
    static_cast<long>(counts),
    static_cast<unsigned long>(timeout_ms));
    return false;
}

void SeedDropper::isr(void* ptr) {
    auto* dropper = reinterpret_cast<SeedDropper*>(ptr);
    if (gpio_read(&dropper->enc_b_hal) == gpio_read(&dropper->enc_a_hal)) {
        dropper->enc_count++;
    } else {
        dropper->enc_count--;
    }
}

void seed_dropper_begin() {
    seed_dropper.begin();
}
