#include "chassis.hpp"

#include "logger.hpp"
#include "motor.hpp"

#include <Arduino.h>
#include <mbed.h>
#include <math.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

Chassis& chassis = Chassis::instance();

Chassis& Chassis::instance() {
    static Chassis instance;
    return instance;
}

void Chassis::begin() {
    stop();
    loggf("chassis ready pid\n");
}

void Chassis::set_target(float next_vx, float next_vy, float next_w) {
    vx = next_vx;
    vy = next_vy;
    w  = next_w;

    fl = vx - vy - w;
    fr = vx + vy + w;
    rl = vx + vy - w;
    rr = vx - vy + w;

    const float max_speed = fmaxf(fmaxf(fabsf(fl), fabsf(fr)), fmaxf(fabsf(rl), fabsf(rr)));
    if (max_speed > CHASSIS_MAX_WHEEL_SPEED) {
        fl *= CHASSIS_MAX_WHEEL_SPEED / max_speed;
        fr *= CHASSIS_MAX_WHEEL_SPEED / max_speed;
        rl *= CHASSIS_MAX_WHEEL_SPEED / max_speed;
        rr *= CHASSIS_MAX_WHEEL_SPEED / max_speed;
    }

    motor_fl().set_speed(fl);
    motor_fr().set_speed(fr);
    motor_rl().set_speed(rl);
    motor_rr().set_speed(rr);
}

void Chassis::update(float dt_s) {
    motor_fl().update(dt_s);
    motor_fr().update(dt_s);
    motor_rl().update(dt_s);
    motor_rr().update(dt_s);
}

void Chassis::stop() {
    vx = 0.0f;
    vy = 0.0f;
    w  = 0.0f;
    fl = 0.0f;
    fr = 0.0f;
    rl = 0.0f;
    rr = 0.0f;
    motors_stop_all();
}

void chassis_begin() {
    chassis.begin();
}

void chassis_stop() {
    chassis.stop();
}

void func_chassis_entry() {
    constexpr float dt_s = CHASSIS_TASK_INTERVAL_MS * 0.001f;

    while (true) {
        chassis.update(dt_s);
        ThisThread::sleep_for(std::chrono::milliseconds(CHASSIS_TASK_INTERVAL_MS));
    }
}
