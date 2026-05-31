#include "movement.h"

#include "chassis.hpp"
#include "config.hpp"
#include "imu.hpp"
#include "line_follower.hpp"
#include "mission.hpp"
#include "motor.hpp"
#include "rfid.hpp"
#include "sensors.hpp"
#include "state.hpp"
#include "wall_follower.hpp"

#include <Arduino.h>
#include <math.h>
#include <stdio.h>

static constexpr float CELL_DISTANCE_CM = 30.0f;
static constexpr float DRIVE_SPEED = TASK_DRIVE_SPEED;
static constexpr float LINE_SPEED = TASK_LINE_SPEED;
static constexpr int16_t FRONT_BLOCKED_CM = TASK_OBSTACLE_FRONT_CM;
static constexpr unsigned long MOVE_TIMEOUT_MS = MISSION_DRIVE_TIMEOUT_MS;
static constexpr unsigned long LINE_TIMEOUT_MS = MISSION_LINE_TIMEOUT_MS;
static constexpr unsigned long RFID_TIMEOUT_MS = 2500;
static constexpr unsigned long RFID_CELL_TIMEOUT_MS = 12000;

static float wrap_deg_180(float deg)
{
    while(deg > 180.0f)
    {
        deg -= 360.0f;
    }

    while(deg <= -180.0f)
    {
        deg += 360.0f;
    }

    return deg;
}

static float wrap_deg_360(float deg)
{
    while(deg >= 360.0f)
    {
        deg -= 360.0f;
    }

    while(deg < 0.0f)
    {
        deg += 360.0f;
    }

    return deg;
}

static bool should_stop(void)
{
    return running_state == RunningState::STOPPED || sensors.kill_switch_pressed();
}

static void stop_motion(void)
{
    mission_stop();
    wall_follower_stop();
    line_follower_stop();
    chassis_stop();
}

static void reset_counts(int32_t* fl, int32_t* fr, int32_t* rl, int32_t* rr)
{
    *fl = motor_fl().count();
    *fr = motor_fr().count();
    *rl = motor_rl().count();
    *rr = motor_rr().count();
}

static float traveled_cm(int32_t fl0, int32_t fr0, int32_t rl0, int32_t rr0)
{
    const float fl = (float)(motor_fl().count() - fl0);
    const float fr = (float)(motor_fr().count() - fr0);
    const float rl = (float)(motor_rl().count() - rl0);
    const float rr = (float)(motor_rr().count() - rr0);
    const float forward_counts = fabsf((fl + fr + rl + rr) * 0.25f);

    return forward_counts / CHASSIS_ENCODER_COUNTS_PER_CM;
}

static void drive_distance(float distance_cm, float speed)
{
    int32_t fl0 = 0;
    int32_t fr0 = 0;
    int32_t rl0 = 0;
    int32_t rr0 = 0;

    stop_motion();
    reset_counts(&fl0, &fr0, &rl0, &rr0);
    chassis.set_target(speed, 0.0f, 0.0f);

    const unsigned long start_ms = millis();
    while(millis() - start_ms < MOVE_TIMEOUT_MS)
    {
        if(should_stop())
        {
            break;
        }

        const int16_t front_cm = sensors.ultrasonic_front_cm();
        if(speed > 0.0f && front_cm > 0 && front_cm <= FRONT_BLOCKED_CM)
        {
            break;
        }

        if(traveled_cm(fl0, fr0, rl0, rr0) >= fabsf(distance_cm))
        {
            break;
        }

        delay(20);
    }

    chassis_stop();
}

static void follow_line_one_cell(void)
{
    stop_motion();
    line_follower.start(
        LINE_SPEED,
        LineFollower::StopMode::Distance,
        LINE_DEFAULT_FRONT_STOP_CM,
        CELL_DISTANCE_CM
    );

    const unsigned long start_ms = millis();
    while(line_follower.is_active() && millis() - start_ms < LINE_TIMEOUT_MS)
    {
        if(should_stop())
        {
            break;
        }

        delay(20);
    }

    line_follower_stop();
}

static bool follow_line_to_next_rfid(uint32_t* detected_uid)
{
    stop_motion();
    rfid_update();

    line_follower.start_rfid(
        LINE_SPEED,
        0,
        true,
        true,
        LINE_DEFAULT_FRONT_STOP_CM
    );

    const unsigned long start_ms = millis();
    while(line_follower.is_active() && millis() - start_ms < RFID_CELL_TIMEOUT_MS)
    {
        if(should_stop())
        {
            line_follower_stop();
            return false;
        }

        delay(20);
    }

    line_follower_stop();

    const uint32_t uid = rfid.last_uid();
    if(detected_uid != NULL)
    {
        *detected_uid = uid;
    }

    return uid != 0;
}

static void turn_degrees(float delta_deg)
{
    if(!imu.yaw_is_ready())
    {
        return;
    }

    stop_motion();

    const float start_yaw = imu.yaw_deg();
    const float target_yaw = wrap_deg_360(start_yaw + delta_deg);
    float prev_err = wrap_deg_180(target_yaw - start_yaw);
    uint8_t confirm = 0;
    unsigned long last_ms = millis();
    const unsigned long start_ms = last_ms;

    while(millis() - start_ms < TURN_TIMEOUT_MS)
    {
        if(should_stop())
        {
            break;
        }

        const unsigned long now_ms = millis();
        const float dt_s = fmaxf(0.001f, (float)(now_ms - last_ms) * 0.001f);
        last_ms = now_ms;

        const float yaw = imu.yaw_deg();
        const float err = wrap_deg_180(target_yaw - yaw);
        const float yaw_rate_dps = -wrap_deg_180(err - prev_err) / dt_s;
        prev_err = err;

        if(fabsf(err) <= TURN_TOLERANCE_DEG && fabsf(yaw_rate_dps) <= TURN_STOP_SPEED_DPS)
        {
            chassis.set_target(0.0f, 0.0f, 0.0f);
            confirm++;
            if(confirm >= TURN_CONFIRM_COUNT)
            {
                break;
            }
        }
        else
        {
            confirm = 0;
            float w = TURN_KP * err + TURN_KD * -yaw_rate_dps;
            const float slow_scale = constrain(fabsf(err) / TURN_SLOW_ZONE_DEG, 0.25f, 1.0f);
            const float limited_max_w = fmaxf(TURN_MIN_WHEEL_SPEED, TURN_MAX_WHEEL_SPEED * slow_scale);

            w = constrain(w, -limited_max_w, limited_max_w);
            if(fabsf(w) > 0.001f && fabsf(w) < TURN_MIN_WHEEL_SPEED)
            {
                w = w >= 0.0f ? TURN_MIN_WHEEL_SPEED : -TURN_MIN_WHEEL_SPEED;
            }

            chassis.set_target(0.0f, 0.0f, TURN_DIRECTION * w);
        }

        delay(20);
    }

    chassis_stop();
}

static void observe_cell_if_valid(
    OccupancyMap* occ,
    int x,
    int y,
    bool blocked
)
{
    if(x < 0 || x >= GRID_W || y < 0 || y >= GRID_H)
    {
        return;
    }

    occupancy_observe_dynamic(occ, x, y, blocked, false);
}

void move_forward_one_cell(void)
{
    move_forward_rfid_cells(1);
}

bool move_forward_rfid_cells(int cell_count)
{
    if(cell_count <= 0)
    {
        return false;
    }

    for(int i = 0; i < cell_count; i++)
    {
        uint32_t uid = 0;
        if(!follow_line_to_next_rfid(&uid))
        {
            return false;
        }
    }

    return true;
}

void move_backward_one_cell(void)
{
    drive_distance(CELL_DISTANCE_CM, -DRIVE_SPEED);
}

void strafe_left_one_cell(void)
{
    turn_left_90();
    follow_line_one_cell();
}

void strafe_right_one_cell(void)
{
    turn_right_90();
    follow_line_one_cell();
}

void align_to_hole_midpoint(void)
{
    drive_distance(8.0f, DRIVE_SPEED * 0.5f);
}

void drop_seed(void)
{
    wait_for_rfid_confirmation();
    stop_motion();
}

bool wait_for_rfid_confirmation(void)
{
    const unsigned long start_ms = millis();
    while(millis() - start_ms < RFID_TIMEOUT_MS)
    {
        rfid_update();
        if(rfid.last_uid() != 0)
        {
            return true;
        }

        if(should_stop())
        {
            return false;
        }

        delay(20);
    }

    return false;
}

bool read_rfid_tag(
    char* tag_id,
    size_t tag_id_size
)
{
    if(tag_id == NULL || tag_id_size == 0)
    {
        return false;
    }

    tag_id[0] = '\0';

    const unsigned long start_ms = millis();
    while(millis() - start_ms < RFID_TIMEOUT_MS)
    {
        rfid_update();
        const uint32_t uid = rfid.last_uid();
        if(uid != 0)
        {
            snprintf(tag_id, tag_id_size, "%08lX", (unsigned long)uid);
            return true;
        }

        if(should_stop())
        {
            return false;
        }

        delay(20);
    }

    return false;
}

void movement_observe_adjacent_cells(
    OccupancyMap* occ,
    Cell robot_pos,
    GridHeading heading
)
{
    int front_dx = 0;
    int front_dy = 1;

    switch(heading)
    {
        case HEADING_POS_X:
            front_dx = 1;
            front_dy = 0;
            break;

        case HEADING_NEG_Y:
            front_dx = 0;
            front_dy = -1;
            break;

        case HEADING_NEG_X:
            front_dx = -1;
            front_dy = 0;
            break;

        default:
            break;
    }

    const int left_dx = -front_dy;
    const int left_dy = front_dx;
    const int right_dx = front_dy;
    const int right_dy = -front_dx;

    const int16_t front_cm = sensors.ultrasonic_front_cm();
    const int16_t left_cm = sensors.ultrasonic_left_cm();
    const int16_t right_cm = sensors.ultrasonic_right_cm();

    observe_cell_if_valid(occ, robot_pos.x + front_dx, robot_pos.y + front_dy, front_cm > 0 && front_cm <= FRONT_BLOCKED_CM);
    observe_cell_if_valid(occ, robot_pos.x + left_dx, robot_pos.y + left_dy, left_cm > 0 && left_cm <= FRONT_BLOCKED_CM);
    observe_cell_if_valid(occ, robot_pos.x + right_dx, robot_pos.y + right_dy, right_cm > 0 && right_cm <= FRONT_BLOCKED_CM);
}

void execute_move(MoveDirection dir)
{
    switch(dir)
    {
        case MOVE_FORWARD:
            move_forward_one_cell();
            break;

        case MOVE_BACKWARD:
            move_backward_one_cell();
            break;

        case MOVE_LEFT:
            strafe_left_one_cell();
            break;

        case MOVE_RIGHT:
            strafe_right_one_cell();
            break;

        default:
            stop_motion();
            break;
    }
}

void turn_left_90(void)
{
    turn_degrees(90.0f);
}

void turn_right_90(void)
{
    turn_degrees(-90.0f);
}

void movement_stop_all(void)
{
    stop_motion();
}

bool movement_start_button_pressed(void)
{
    return sensors.revive_button_pressed();
}

bool movement_killswitch_pressed(void)
{
    return should_stop();
}

void execute_move_to_heading(
    GridHeading* heading,
    GridHeading desired_heading
)
{
    if(heading == NULL)
    {
        return;
    }

    const int turns = ((int)desired_heading - (int)(*heading) + 4) % 4;

    if(turns == 1)
    {
        turn_right_90();
    }
    else if(turns == 2)
    {
        turn_right_90();
        turn_right_90();
    }
    else if(turns == 3)
    {
        turn_left_90();
    }

    *heading = desired_heading;
    move_forward_one_cell();
}
