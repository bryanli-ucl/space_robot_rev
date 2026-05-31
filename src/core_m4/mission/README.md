# Term Three Navigation Addon

This folder is the small set of Term 3 files to add to a robot project that already contains `space_robot_rev-main`.

Copy these files into the same firmware project as `space_robot_rev-main`:

- Put this whole folder under `src/core_m4/termthree_navigation` or copy the files directly into `src/core_m4`.
- Do not copy the old `movement_arduino.cpp`, `lib/`, or `first_test.ino` files.
- Keep the existing `space_robot_rev-main/include` and `space_robot_rev-main/src/core_m4` files, because this addon calls their chassis, sensor, RFID, line follower, wall follower, IMU, state, and mission code.
- Keep a `secrets.h` beside `messaging_arduino.cpp` with your Wi-Fi, broker, and `GROUP_ID` values.

To run the navigation layer from the existing firmware, include this header from `src/core_m4/main.cpp`:

```cpp
#include "termthree_navigation/termthree_navigation.h"
```

Then call:

```cpp
termthree_navigation_begin();
termthree_navigation_set_enabled(true);
```

after the existing motors, chassis, sensors, RFID, IMU, and task threads have started. Call this repeatedly from the main loop or from a low-priority thread:

```cpp
termthree_navigation_tick();
```

The addon deliberately contains only navigation, planning, maps, tasks, risk scoring, RFID map lookup, and a thin movement adapter. Actual hardware movement, sensor fusion, RFID reads, kill switch handling, line following, wall following, IMU turns, and chassis control stay in `space_robot_rev-main`.

The included MiniMessenger bridge uses the newer whole-map flow. It sends `type=getMap ...`, accepts the 21-byte binary grid map, handles 6-byte team-status packets, and still keeps the older text handlers as fallback compatibility. This means navigation can update hole fertility/seeded state from the shared map instead of doing manual RFID map requests.

If left and right turns are reversed on the real robot, swap the signs in `turn_left_90()` and `turn_right_90()` inside `movement_space_robot.cpp`.
