#include "wall_follower.hpp"

#include "chassis.hpp"
#include "config.hpp"
#include "imu.hpp"
#include "logger.hpp"
#include "motor.hpp"
#include "sensors.hpp"
#include "state.hpp"

#include <Arduino.h>
#include <math.h>

WallFollower& wall_follower = WallFollower::instance();

static float wrap_deg_180(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

WallFollower& WallFollower::instance() {
    static WallFollower instance;
    return instance;
}

const char* WallFollower::side_name() const {
    return side == Side::Left ? "left" : "right";
}

void WallFollower::start(Side next_side, float next_speed, float next_target_distance_cm, int16_t next_target_wall_cm) {
    side                 = next_side;
    speed                = next_speed;
    target_distance_cm   = next_target_distance_cm;
    target_wall_cm       = next_target_wall_cm;
    traveled_distance_cm = 0.0f;
    wall_cm              = -1;
    front_cm             = -1;
    wall_error           = 0.0f;
    yaw_start            = imu.yaw_deg();
    yaw_target           = yaw_start;
    yaw_error            = 0.0f;
    w                    = 0.0f;
    lost_count           = 0;
    start_fl_count       = motor_fl().count();
    start_fr_count       = motor_fr().count();
    start_rl_count       = motor_rl().count();
    start_rr_count       = motor_rr().count();
    last_status_ms       = 0;
    active               = true;

    loggf("wall start side=%s speed=%.1f dist=%.1f target=%d yaw=%.2f\n",
    side_name(),
    speed,
    target_distance_cm,
    target_wall_cm,
    yaw_start);
}

void WallFollower::stop() {
    if (active) loggf("wall stop\n");
    active               = false;
    speed                = 0.0f;
    target_distance_cm   = 0.0f;
    traveled_distance_cm = 0.0f;
    wall_cm              = -1;
    front_cm             = -1;
    wall_error           = 0.0f;
    yaw_target           = yaw_start;
    yaw_error            = 0.0f;
    w                    = 0.0f;
    lost_count           = 0;
    chassis_stop();
}

void WallFollower::update_traveled_distance() {
    const float fl             = static_cast<float>(motor_fl().count() - start_fl_count);
    const float fr             = static_cast<float>(motor_fr().count() - start_fr_count);
    const float rl             = static_cast<float>(motor_rl().count() - start_rl_count);
    const float rr             = static_cast<float>(motor_rr().count() - start_rr_count);
    const float forward_counts = fabsf((fl + fr + rl + rr) * 0.25f);
    traveled_distance_cm       = forward_counts / CHASSIS_ENCODER_COUNTS_PER_CM;
}

bool WallFollower::should_stop() {
    if (running_state == RunningState::STOPPED) {
        loggf("wall stopped by state\n");
        return true;
    }

    if (front_cm > 0 && front_cm <= WALL_FRONT_STOP_CM) {
        loggf("wall front stop front=%d threshold=%d traveled=%.1f\n",
        front_cm,
        WALL_FRONT_STOP_CM,
        traveled_distance_cm);
        return true;
    }

    if (target_distance_cm > 0.0f && traveled_distance_cm >= target_distance_cm) {
        loggf("wall distance stop traveled=%.1f target=%.1f\n", traveled_distance_cm, target_distance_cm);
        return true;
    }

    if (lost_count >= WALL_LOST_CONFIRM) {
        loggf("wall lost side=%s traveled=%.1f\n", side_name(), traveled_distance_cm);
        return true;
    }

    return false;
}

void WallFollower::update(float dt_s) {
    if (!active) return;

    update_traveled_distance();

    wall_cm  = side == Side::Left ? sensors.ultrasonic_left_cm() : sensors.ultrasonic_right_cm();
    front_cm = sensors.ultrasonic_front_cm();

    if (wall_cm <= 0) {
        if (lost_count < WALL_LOST_CONFIRM) lost_count++;
    } else {
        lost_count = 0;
    }

    if (should_stop()) {
        stop();
        return;
    }

    const float side_direction = side == Side::Left ? WALL_LEFT_DIRECTION : WALL_RIGHT_DIRECTION;
    if (wall_cm > 0) {
        wall_error = static_cast<float>(wall_cm - target_wall_cm);
    }

    const float yaw_offset = constrain(side_direction * WALL_DIST_TO_YAW_KP * wall_error,
    -WALL_MAX_YAW_OFFSET_DEG,
    WALL_MAX_YAW_OFFSET_DEG);
    yaw_target = wrap_deg_180(yaw_start + yaw_offset);
    yaw_error  = wrap_deg_180(yaw_target - imu.yaw_deg());
    w         = TURN_DIRECTION * WALL_YAW_KP * yaw_error;
    w         = constrain(w, -WALL_MAX_WHEEL_SPEED, WALL_MAX_WHEEL_SPEED);

    chassis.set_target(speed, 0.0f, w);

    const uint32_t now_ms = millis();
    if (last_status_ms == 0 || now_ms - last_status_ms >= WALL_STATUS_INTERVAL_MS) {
        last_status_ms = now_ms;
        print_status();
    }
}

void WallFollower::print_status() const {
    loggf("wall active=%d side=%s speed=%.1f dist=%.1f/%.1f wall=%d/%d err=%.1f yaw=%.2f start=%.2f target=%.2f yawerr=%.2f front=%d lost=%u cross=%d w=%.1f\n",
    active ? 1 : 0,
    side_name(),
    speed,
    traveled_distance_cm,
    target_distance_cm,
    wall_cm,
    target_wall_cm,
    wall_error,
    imu.yaw_deg(),
    yaw_start,
    yaw_target,
    yaw_error,
    front_cm,
    lost_count,
    sensors.ir_side_left_detected() && sensors.ir_side_right_detected() ? 1 : 0,
    w);
}

void wall_follower_update(float dt_s) {
    wall_follower.update(dt_s);
}

void wall_follower_stop() {
    wall_follower.stop();
}
