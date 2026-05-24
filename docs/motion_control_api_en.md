# Motion Control Blocking API

This document is for the mission/application layer. The API is declared in `include/motion_control.hpp` and implemented in `src/motion_control.cpp`.

All functions are blocking calls. Once called, the robot keeps moving until one of the stop conditions is reached, the stop button is pressed, a required sensor is invalid, or a timeout occurs. Each function returns a `MotionResult` so the mission layer can decide what to do next.

## Requirements

Enable the required tasks/sensors before using each feature:

```cpp
CONFIG::ENABLE_TASK_CHASSIS      = true;
CONFIG::ENABLE_TASK_HEARTBEAT    = true;
CONFIG::ENABLE_TASK_SENSORS      = true;  // updates IR and ultrasonic data
CONFIG::ENABLE_SENSOR_ULTRASONIC = true;  // front/left/right ultrasonic sensors
CONFIG::ENABLE_SENSOR_IR_ARRAY   = true;  // line following and cross-line detection
CONFIG::ENABLE_TASK_RFID         = true;  // RFID detection
CONFIG::ENABLE_TASK_IMU          = true;  // wall yaw hold and turn control
```

If a feature does not need a sensor, that sensor can stay disabled. For example, `drive motor` does not need IR or RFID. However, if the default `front_stop_cm = 10` safety stop is kept, the ultrasonic task should be running.

## Defaults

The main defaults are defined in `include/main.hpp` and `include/motion_control.hpp`:

```cpp
CONFIG::COUNTS_PER_CM = 120.0f;       // temporary value, must be calibrated
MOTION_DEFAULT_SPEED_CM_S = 10.0f;
MOTION_DEFAULT_FRONT_STOP_CM = 10.0f;
CONFIG::LINE_CROSS_MIN_BLACK = 7;
CONFIG::LINE_BLACK_THRESHOLD = 700;
CONFIG::LINE_CROSS_CONFIRM = 2;
CONFIG::TURN_MAX_W = 2.0f;
CONFIG::TURN_TOLERANCE_DEG = 2.0f;
CONFIG::TURN_TIMEOUT_MS = 8000;
```

## Return Values

All motion functions return:

```cpp
enum class MotionResult {
    DistanceReached,      // target motor distance reached
    FrontObstacle,        // front ultrasonic distance <= front_stop_cm
    RfidDetected,         // target RFID detected
    CrossLineDetected,    // cross line detected by IR array
    AngleReached,         // turn target angle reached
    StopButton,           // stop button / button_state STOPPED
    SensorInvalid,        // required sensor is not ready
    Timeout,              // timeout
};
```

Use `motion_result_name(result)` to convert a result to a string:

```cpp
MotionResult result = run_turn_deg(90.0f);
serial_tx("turn result=%s\n", motion_result_name(result));
```

## Force Stop

```cpp
void motion_force_stop(bool disable_chassis = true);
```

This function:
- clears position control
- calls `chassis.set_target(0, 0, 0)`
- clears manual PWM on all four motors
- calls `chassis.disable()` when `disable_chassis == true`

The heartbeat thread already calls this when the stop button is triggered.

## Line Following

Line following uses the 9-channel QTR sensor array:
- `ir_pos` center is around `4000`
- the controller outputs angular velocity `w`
- forward speed is in `cm/s`
- all line-following functions check the front ultrasonic sensor and stop at `10cm` by default

### Follow Line Until Front Obstacle

```cpp
MotionResult run_line_follow_until_front_cm(
    float front_stop_cm = 10.0f,
    float speed_cm_s = 10.0f);
```

Examples:

```cpp
MotionResult r = run_line_follow_until_front_cm();
MotionResult r = run_line_follow_until_front_cm(15.0f, 8.0f);
```

Common return values:
- `FrontObstacle`
- `StopButton`

### Follow Line Until Motor Distance

```cpp
MotionResult run_line_follow_until_motor_cm(
    float distance_cm,
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

Examples:

```cpp
MotionResult r = run_line_follow_until_motor_cm(80.0f);
MotionResult r = run_line_follow_until_motor_cm(80.0f, 10.0f, 12.0f);
```

Possible return values:
- `DistanceReached`
- `FrontObstacle`
- `StopButton`

### Follow Line Until Any RFID

```cpp
MotionResult run_line_follow_until_rfid(
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

Example:

```cpp
MotionResult r = run_line_follow_until_rfid();
```

Possible return values:
- `RfidDetected`
- `FrontObstacle`
- `StopButton`

### Follow Line Until Specific RFID UID

```cpp
MotionResult run_line_follow_until_rfid_uid(
    uint32_t uid,
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

Examples:

```cpp
MotionResult r = run_line_follow_until_rfid_uid(0x12345678);
MotionResult r = run_line_follow_until_rfid_uid(0x12345678, 8.0f, 10.0f);
```

### Follow Line Until Cross Line

```cpp
MotionResult run_line_follow_until_cross(
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

Cross-line detection:
- at least `CONFIG::LINE_CROSS_MIN_BLACK` out of 9 IR sensors are above `CONFIG::LINE_BLACK_THRESHOLD`
- the condition must be confirmed for `CONFIG::LINE_CROSS_CONFIRM` consecutive checks

Examples:

```cpp
MotionResult r = run_line_follow_until_cross();
MotionResult r = run_line_follow_until_cross(8.0f, 10.0f);
```

## Wall Following

Wall following treats the robot as a normal wheeled robot, without mecanum strafing. Control behaviour:
- left/right ultrasonic sensor keeps the robot at `wall_dist_cm`
- IMU yaw is used as a weak stabilising correction
- the robot stops when the front ultrasonic distance is below `front_stop_cm`

`WallSide`:

```cpp
enum class WallSide {
    Left,
    Right,
};
```

### Follow Wall Until Front Obstacle

```cpp
MotionResult run_wall_follow_until_front_cm(
    WallSide side,
    float wall_dist_cm,
    float front_stop_cm = 10.0f,
    float speed_cm_s = 10.0f);
```

Examples:

```cpp
MotionResult r = run_wall_follow_until_front_cm(WallSide::Right, 20.0f);
MotionResult r = run_wall_follow_until_front_cm(WallSide::Left, 15.0f, 10.0f, 8.0f);
```

### Follow Wall Until Motor Distance

```cpp
MotionResult run_wall_follow_until_motor_cm(
    WallSide side,
    float wall_dist_cm,
    float distance_cm,
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

Example:

```cpp
MotionResult r = run_wall_follow_until_motor_cm(WallSide::Right, 20.0f, 100.0f);
```

Possible return values:
- `DistanceReached`
- `FrontObstacle`
- `StopButton`

### Follow Wall Until Any RFID

```cpp
MotionResult run_wall_follow_until_rfid(
    WallSide side,
    float wall_dist_cm,
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

Example:

```cpp
MotionResult r = run_wall_follow_until_rfid(WallSide::Right, 20.0f);
```

### Follow Wall Until Specific RFID UID

```cpp
MotionResult run_wall_follow_until_rfid_uid(
    WallSide side,
    float wall_dist_cm,
    uint32_t uid,
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

Example:

```cpp
MotionResult r = run_wall_follow_until_rfid_uid(WallSide::Left, 20.0f, 0x12345678);
```

## Straight Driving

Straight driving treats the robot as a normal wheeled robot:
- motor encoders are used to estimate distance
- IMU yaw is used as a weak heading hold
- speed is in `cm/s`
- `distance_cm` may be negative; negative distance means driving backwards

### Drive Until Front Obstacle

```cpp
MotionResult run_drive_until_front_cm(
    float front_stop_cm = 10.0f,
    float speed_cm_s = 10.0f);
```

Examples:

```cpp
MotionResult r = run_drive_until_front_cm();
MotionResult r = run_drive_until_front_cm(12.0f, 8.0f);
MotionResult r = run_drive_until_front_cm(12.0f, -8.0f); // backwards, usually not recommended with front stop
```

### Drive Until Motor Distance

```cpp
MotionResult run_drive_until_motor_cm(
    float distance_cm,
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

Examples:

```cpp
MotionResult r = run_drive_until_motor_cm(50.0f);
MotionResult r = run_drive_until_motor_cm(-30.0f, 8.0f); // drive backwards 30cm
```

### Drive Until Any RFID

```cpp
MotionResult run_drive_until_rfid(
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

Example:

```cpp
MotionResult r = run_drive_until_rfid();
```

### Drive Until Specific RFID UID

```cpp
MotionResult run_drive_until_rfid_uid(
    uint32_t uid,
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

Example:

```cpp
MotionResult r = run_drive_until_rfid_uid(0x12345678);
```

## Turning

Turning uses IMU yaw PID control. The angle unit is degrees:
- positive value: turn in the positive chassis `w` direction
- negative value: turn in the opposite direction
- yaw wrap is handled internally, for example from `179` to `-179`

```cpp
MotionResult run_turn_deg(
    float delta_deg,
    float max_w = CONFIG::TURN_MAX_W,
    float tolerance_deg = CONFIG::TURN_TOLERANCE_DEG,
    uint32_t timeout_ms = CONFIG::TURN_TIMEOUT_MS);
```

Examples:

```cpp
MotionResult r = run_turn_deg(90.0f);
MotionResult r = run_turn_deg(-90.0f);
MotionResult r = run_turn_deg(180.0f, 2.0f, 2.0f, 10000);
```

Common return values:
- `AngleReached`
- `SensorInvalid`, when IMU is not ready
- `StopButton`
- `Timeout`

## Serial Debug Commands

These commands are for debugging. The mission/application layer should normally call the C++ functions directly.

### Line

```text
line front [front_cm] [speed_cm_s]
line motor <distance_cm> [speed_cm_s] [front_cm]
line rfid [uid|any] [speed_cm_s] [front_cm]
line cross [speed_cm_s] [front_cm]
```

Examples:

```text
line front
line motor 80 10 10
line rfid any 8 10
line rfid 0x12345678 8 10
line cross 10 10
```

### Wall

```text
wall front <l|r> <wall_cm> [front_cm] [speed_cm_s]
wall motor <l|r> <wall_cm> <distance_cm> [speed_cm_s] [front_cm]
wall rfid <l|r> <wall_cm> [uid|any] [speed_cm_s] [front_cm]
```

Examples:

```text
wall front r 20
wall motor l 15 100 10 10
wall rfid r 20 any 8 10
wall rfid l 20 0x12345678 8 10
```

### Drive

```text
drive front [front_cm] [speed_cm_s]
drive motor <distance_cm> [speed_cm_s] [front_cm]
drive rfid [uid|any] [speed_cm_s] [front_cm]
```

Examples:

```text
drive front
drive motor 50 10 10
drive motor -30 8 10
drive rfid any 10 10
drive rfid 0x12345678 10 10
```

### Turn

```text
turn <left|right|90|-90|180|-180|deg> [max_w] [tolerance_deg] [timeout_ms]
```

Examples:

```text
turn 90
turn -90
turn left
turn right
turn 180 2.0 2.0 10000
```

## Example Mission Usage

```cpp
#include "motion_control.hpp"

void run_simple_route() {
    MotionResult r;

    r = run_line_follow_until_cross(10.0f, 10.0f);
    if (r != MotionResult::CrossLineDetected) {
        motion_force_stop();
        return;
    }

    r = run_turn_deg(90.0f);
    if (r != MotionResult::AngleReached) {
        motion_force_stop();
        return;
    }

    r = run_drive_until_motor_cm(50.0f, 10.0f, 10.0f);
    if (r == MotionResult::FrontObstacle) {
        // handle obstacle
    }
}
```

## Notes

- `COUNTS_PER_CM = 120.0f` is a temporary value and must be calibrated on the real robot.
- `front_stop_cm` only works when `dist_front > 0`; if the ultrasonic task is disabled, the function will not stop because of a front obstacle.
- RFID functions clear `detected_uid` at the start to avoid triggering from a previous tag detection.
- Blocking functions are intended for the mission task. Do not call them from high-priority sensor or motor threads.
- The stop button is handled by the heartbeat thread through force stop, and blocking functions also return `StopButton`.
