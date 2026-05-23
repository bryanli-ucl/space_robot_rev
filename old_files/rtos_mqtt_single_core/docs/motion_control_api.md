# Motion Control Blocking API

这份文档给 mission / 应用层使用。接口定义在 `include/motion_control.hpp`，实现位于 `src/motion_control.cpp`。

这些函数都是阻塞式函数：调用后会一直控制小车运动，直到满足结束条件、stop button 被触发、传感器无效或超时。函数返回 `MotionResult`，应用层根据返回值决定下一步。

## 使用前提

在使用对应功能前，需要打开对应 task / sensor：

```cpp
CONFIG::ENABLE_TASK_CHASSIS      = true;
CONFIG::ENABLE_TASK_HEARTBEAT    = true;
CONFIG::ENABLE_TASK_SENSORS      = true;  // IR / ultrasonic 在这里更新
CONFIG::ENABLE_SENSOR_ULTRASONIC = true;  // front/left/right 超声波
CONFIG::ENABLE_SENSOR_IR_ARRAY   = true;  // 循线和十字线
CONFIG::ENABLE_TASK_RFID         = true;  // RFID
CONFIG::ENABLE_TASK_IMU          = true;  // wall yaw hold 和 turn
```

如果某个功能不用对应传感器，可以不打开。例如只测试 `drive motor`，不需要 IR/RFID，但如果保留默认 `front_stop_cm = 10`，需要超声波正常更新。

## 全局默认值

默认参数在 `include/main.hpp` 和 `include/motion_control.hpp`：

```cpp
CONFIG::COUNTS_PER_CM = 120.0f;       // 需要实测校准
MOTION_DEFAULT_SPEED_CM_S = 10.0f;
MOTION_DEFAULT_FRONT_STOP_CM = 10.0f;
CONFIG::LINE_CROSS_MIN_BLACK = 7;
CONFIG::LINE_BLACK_THRESHOLD = 700;
CONFIG::LINE_CROSS_CONFIRM = 2;
CONFIG::TURN_MAX_W = 2.0f;
CONFIG::TURN_TOLERANCE_DEG = 2.0f;
CONFIG::TURN_TIMEOUT_MS = 8000;
```

## 返回值

所有运动函数返回：

```cpp
enum class MotionResult {
    DistanceReached,      // 到达指定 motor 距离
    FrontObstacle,        // 前方超声波距离 <= front_stop_cm
    RfidDetected,         // 检测到目标 RFID
    CrossLineDetected,    // 检测到十字线
    AngleReached,         // turn 到达目标角度
    StopButton,           // stop button / button_state STOPPED
    SensorInvalid,        // 必需传感器未 ready
    Timeout,              // 超时
};
```

可用 `motion_result_name(result)` 转成字符串。

```cpp
MotionResult result = run_turn_deg(90.0f);
serial_tx("turn result=%s\n", motion_result_name(result));
```

## 强制停车

```cpp
void motion_force_stop(bool disable_chassis = true);
```

功能：
- 清除 position control
- `chassis.set_target(0, 0, 0)`
- 清除四个 motor 的 manual PWM
- 如果 `disable_chassis == true`，调用 `chassis.disable()`

heartbeat thread 里已经会在 stop button 触发时调用强停。

## 循线函数

循线使用 9 路 QTR 阵列：
- `ir_pos` 中心值约为 `4000`
- PID 输出旋转角速度 `w`
- 前进速度单位是 `cm/s`
- 所有循线函数都会检查前方超声波，默认 `10cm` 停车

### 循线直到前方障碍

```cpp
MotionResult run_line_follow_until_front_cm(
    float front_stop_cm = 10.0f,
    float speed_cm_s = 10.0f);
```

例子：

```cpp
MotionResult r = run_line_follow_until_front_cm();
MotionResult r = run_line_follow_until_front_cm(15.0f, 8.0f);
```

结束原因通常是：
- `FrontObstacle`
- `StopButton`

### 循线直到 motor 距离

```cpp
MotionResult run_line_follow_until_motor_cm(
    float distance_cm,
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

例子：

```cpp
MotionResult r = run_line_follow_until_motor_cm(80.0f);
MotionResult r = run_line_follow_until_motor_cm(80.0f, 10.0f, 12.0f);
```

结束原因可能是：
- `DistanceReached`
- `FrontObstacle`
- `StopButton`

### 循线直到任意 RFID

```cpp
MotionResult run_line_follow_until_rfid(
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

例子：

```cpp
MotionResult r = run_line_follow_until_rfid();
```

结束原因可能是：
- `RfidDetected`
- `FrontObstacle`
- `StopButton`

### 循线直到指定 RFID UID

```cpp
MotionResult run_line_follow_until_rfid_uid(
    uint32_t uid,
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

例子：

```cpp
MotionResult r = run_line_follow_until_rfid_uid(0x12345678);
MotionResult r = run_line_follow_until_rfid_uid(0x12345678, 8.0f, 10.0f);
```

### 循线直到十字线

```cpp
MotionResult run_line_follow_until_cross(
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

十字线判定：
- 9 个 IR 中至少 `CONFIG::LINE_CROSS_MIN_BLACK` 个读数超过 `CONFIG::LINE_BLACK_THRESHOLD`
- 连续确认 `CONFIG::LINE_CROSS_CONFIRM` 次

例子：

```cpp
MotionResult r = run_line_follow_until_cross();
MotionResult r = run_line_follow_until_cross(8.0f, 10.0f);
```

## 寻墙函数

寻墙按普通轮车处理，不使用麦轮平移。控制方式：
- 左/右超声波保持 `wall_dist_cm`
- IMU yaw 做弱修正，减少车身发散
- 前方超声波小于 `front_stop_cm` 会停车

`WallSide`：

```cpp
enum class WallSide {
    Left,
    Right,
};
```

### 寻墙直到前方障碍

```cpp
MotionResult run_wall_follow_until_front_cm(
    WallSide side,
    float wall_dist_cm,
    float front_stop_cm = 10.0f,
    float speed_cm_s = 10.0f);
```

例子：

```cpp
MotionResult r = run_wall_follow_until_front_cm(WallSide::Right, 20.0f);
MotionResult r = run_wall_follow_until_front_cm(WallSide::Left, 15.0f, 10.0f, 8.0f);
```

### 寻墙直到 motor 距离

```cpp
MotionResult run_wall_follow_until_motor_cm(
    WallSide side,
    float wall_dist_cm,
    float distance_cm,
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

例子：

```cpp
MotionResult r = run_wall_follow_until_motor_cm(WallSide::Right, 20.0f, 100.0f);
```

结束原因可能是：
- `DistanceReached`
- `FrontObstacle`
- `StopButton`

### 寻墙直到任意 RFID

```cpp
MotionResult run_wall_follow_until_rfid(
    WallSide side,
    float wall_dist_cm,
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

例子：

```cpp
MotionResult r = run_wall_follow_until_rfid(WallSide::Right, 20.0f);
```

### 寻墙直到指定 RFID UID

```cpp
MotionResult run_wall_follow_until_rfid_uid(
    WallSide side,
    float wall_dist_cm,
    uint32_t uid,
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

例子：

```cpp
MotionResult r = run_wall_follow_until_rfid_uid(WallSide::Left, 20.0f, 0x12345678);
```

## 直行函数

直行按普通轮车处理：
- 使用 motor encoder 估算距离
- 使用 IMU yaw 做弱保持，减少跑偏
- 速度单位 `cm/s`
- `distance_cm` 可为负数，负数表示后退

### 直行直到前方障碍

```cpp
MotionResult run_drive_until_front_cm(
    float front_stop_cm = 10.0f,
    float speed_cm_s = 10.0f);
```

例子：

```cpp
MotionResult r = run_drive_until_front_cm();
MotionResult r = run_drive_until_front_cm(12.0f, 8.0f);
MotionResult r = run_drive_until_front_cm(12.0f, -8.0f); // 后退，通常不建议同时用前方障碍作为停止条件
```

### 直行指定 motor 距离

```cpp
MotionResult run_drive_until_motor_cm(
    float distance_cm,
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

例子：

```cpp
MotionResult r = run_drive_until_motor_cm(50.0f);
MotionResult r = run_drive_until_motor_cm(-30.0f, 8.0f); // 后退 30cm
```

### 直行直到任意 RFID

```cpp
MotionResult run_drive_until_rfid(
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

例子：

```cpp
MotionResult r = run_drive_until_rfid();
```

### 直行直到指定 RFID UID

```cpp
MotionResult run_drive_until_rfid_uid(
    uint32_t uid,
    float speed_cm_s = 10.0f,
    float front_stop_cm = 10.0f);
```

例子：

```cpp
MotionResult r = run_drive_until_rfid_uid(0x12345678);
```

## 转向函数

转向使用 IMU yaw PID。角度单位是 degree：
- 正数：按当前底盘 `w` 正方向旋转
- 负数：反方向旋转
- 内部会处理 yaw wrap，例如 `179` 到 `-179`

```cpp
MotionResult run_turn_deg(
    float delta_deg,
    float max_w = CONFIG::TURN_MAX_W,
    float tolerance_deg = CONFIG::TURN_TOLERANCE_DEG,
    uint32_t timeout_ms = CONFIG::TURN_TIMEOUT_MS);
```

例子：

```cpp
MotionResult r = run_turn_deg(90.0f);
MotionResult r = run_turn_deg(-90.0f);
MotionResult r = run_turn_deg(180.0f, 2.0f, 2.0f, 10000);
```

返回值通常是：
- `AngleReached`
- `SensorInvalid`，IMU 未 ready
- `StopButton`
- `Timeout`

## Serial 调试命令

这些命令用于调试，应用层推荐直接调用 C++ 函数。

### Line

```text
line front [front_cm] [speed_cm_s]
line motor <distance_cm> [speed_cm_s] [front_cm]
line rfid [uid|any] [speed_cm_s] [front_cm]
line cross [speed_cm_s] [front_cm]
```

例子：

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

例子：

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

例子：

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

例子：

```text
turn 90
turn -90
turn left
turn right
turn 180 2.0 2.0 10000
```

## Mission 中的典型用法

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

## 注意事项

- `COUNTS_PER_CM = 120.0f` 是临时值，必须实测校准。
- `front_stop_cm` 只在 `dist_front > 0` 时生效；如果超声波 task 没开，函数不会因为前方障碍停止。
- RFID 函数调用开始时会清空 `detected_uid`，避免上一次 RFID 误触发。
- 阻塞函数适合在 mission task 中调用，不适合在高优先级 sensor/motor thread 中调用。
- stop button 强停由 heartbeat thread 处理，阻塞函数内部也会返回 `StopButton`。
