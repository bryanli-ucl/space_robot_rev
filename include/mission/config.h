#ifndef CONFIG_H
#define CONFIG_H

#define GRID_W 9
#define GRID_H 9

#define RFID_TAG_ID_LEN 24
#define RFID_SCAN_START_X 0
#define RFID_SCAN_START_Y 0
#define RFID_BOOTSTRAP_SCAN_ON_START 0
#define ROBOT_START_ARMED 0

#define MAX_ROBOTS 14
#define MAX_HOLES 81
#define MAX_RESOURCES 64
#define MAX_DISABLED 16
#define MAX_OTHER_ROBOTS 13
#define MAX_PATH 256

#define MAX_OPEN 1024

#define SEED_CAPACITY 5
#define REFILL_DELAY_TICKS 10

#define ASTAR_WEIGHT 1.4f
#define ASTAR_MOVE_COST 1.0f
#define ASTAR_TURN_90_COST 0.7f

#define W_COLLISION 4.0f
#define W_STALE 1.5f
#define W_CONGESTION 3.0f
#define W_BOTTLENECK 4.0f
#define W_EVACUATION 5.0f
#define W_TIME 1.5f

#define STALE_DECAY 0.005f

#define ENTRY_X 0
#define ENTRY_Y 2

#define EXIT_X 0
#define EXIT_Y 7

#define HIGH_VALUE_ZONE_X 6

#endif
