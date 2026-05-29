# Testing and Calibration Evidence

This file indexes terminal screenshots, logs and short notes for the programming viva. Put screenshots or videos in `docs/test_evidence/` and link them from the tables below.

Terminal screenshots are useful evidence if they show:

- the command that was run,
- the robot log output,
- the stop/success condition,
- any relevant calibration values.

Good examples are lines such as:

```text
mission done task=2
mission done task=3
mission done task=7
mission revive contact
line rfid stop uid=...
turn done yaw=...
pidtest done
```

## How to Capture Evidence

Use the wireless shell:

```bash
rlwrap nc <robot-ip> 7777
```

Run the behaviour, then take a terminal screenshot. Save it under:

```text
docs/test_evidence/
```

Current evidence files:

```text
shell_common_command_test.png
task_1_linefollow_test.png
task_2_test_1.png
task_2_test_2.png
task_3_test1.png
task_3_test2.png
task_7_test1.png
task_7_test2.png
```

If the screenshot is large, it is fine to crop it so the command and final success lines are readable.

## Behaviour Test Summary

| Behaviour | Command | Result | Evidence file |
|---|---|---|---|
| Shell and manual commands | `start`, `sensor`, `line`, `drive`, `turn`, etc. | Wireless shell and common controls tested | `docs/test_evidence/shell_common_command_test.png` |
| Task 1 line follow | `task 1` or line-follow command | Tested working | `docs/test_evidence/task_1_linefollow_test.png` |
| Task 2 exit/intersection/RFID | `task 2` | Tested working multiple times | `docs/test_evidence/task_2_test_1.png`, `docs/test_evidence/task_2_test_2.png` |
| Task 3 solid-grid navigation | `task 3` | Tested working multiple times | `docs/test_evidence/task_3_test1.png`, `docs/test_evidence/task_3_test2.png` |
| Task 7 obstacle avoidance | `task 7` | Tested working multiple times | `docs/test_evidence/task_7_test1.png`, `docs/test_evidence/task_7_test2.png` |
| Task 8 revive approach | `task 8` | Tested working, screenshot not added yet | TODO |

## Calibration Evidence

| Area | Commands / Method | Notes | Evidence file |
|---|---|---|---|
| Motor PID | `motor pid ...`, `motor pidtest 100`, `motor pidtest 500` | Tuned start PWM, speed feed-forward and PID for low/high speed. | Covered by terminal calibration history; dedicated screenshot TODO |
| Encoder distance | `drive forward <speed> <cm>` | Wheel diameter is 6 cm; encoder counts per cm configured in `include/config.hpp`. | `docs/test_evidence/shell_common_command_test.png` |
| Line following | `sensor ir`, `line cross 150`, `line corner left/right 150` | IR thresholds and line PID tuned on the track surface. | `docs/test_evidence/task_1_linefollow_test.png`, `docs/test_evidence/shell_common_command_test.png` |
| IMU yaw | `imu`, `imu cal gyro`, `turn 90` | Gyro bias calibrated on startup; hybrid mission turns use IMU then IR line search. | `docs/test_evidence/shell_common_command_test.png` |
| RFID | `sensor rfid`, `line rfid 150 any` | RFID UID detection tested with working door-card tag. | `docs/test_evidence/task_2_test_1.png`, `docs/test_evidence/task_2_test_2.png` |
| Ultrasonic | `sensor dist`, `sensor dist watch on` | Front sensor used for obstacle/front stop; left/right modules were less reliable. | `docs/test_evidence/shell_common_command_test.png`, `docs/test_evidence/task_7_test1.png` |
| Revive button | `sensor button`, `sensor button watch on` | D44 active-low contact switch tested in Task 8. | TODO |

## Main Mission Flow Notes

### Task 2

Flow:

1. Follow line to first cross.
2. Drive forward to center the robot.
3. Hybrid right turn.
4. Follow to first left corner.
5. Center and hybrid left turn.
6. Follow to RFID tag.
7. Wait 2 seconds.
8. Follow to second left corner.
9. Center and hybrid left turn.
10. Follow to right corner.
11. Center and hybrid right turn.
12. Follow line until front wall distance threshold.

### Task 3

Flow:

1. Follow two cross nodes.
2. Hybrid right turn.
3. Follow one cross node.
4. Hybrid left turn.
5. Follow two cross nodes and stop.

### Task 7

Flow:

1. Follow line to a cross.
2. Check front ultrasonic for obstacle.
3. If no obstacle, clear the cross and check the next cross.
4. If obstacle is detected, center, turn right, bypass by cross-counting, turn back, and return to the line.

### Task 8

Flow:

1. Follow line towards the revive target.
2. Use front ultrasonic for approach speed zones:
   - 150 while farther than 20 cm,
   - 100 inside 20 cm,
   - 80 inside 10 cm.
3. Stop successfully when the D44 contact button is pressed.
4. Fail safe on line lost, maximum distance or timeout.

## Limitations Observed

- Ultrasonic side sensors were noisy and sometimes returned invalid or stale-looking readings.
- RFID reading distance depends strongly on card type.
- Low-speed motor control is less stable than medium speed because of static friction and drivetrain losses.
- The mecanum wheels are used as normal wheels because some rollers do not spin freely.
