#ifndef COMMON_H
#define COMMON_H

#include <stdbool.h>

typedef struct
{
    int x;
    int y;
} Cell;

typedef enum
{
    CELL_EMPTY,
    CELL_OBSTACLE,
    CELL_ENTRY,
    CELL_EXIT
} CellType;

typedef enum
{
    ROBOT_IDLE,
    ROBOT_MOVING,
    ROBOT_DROPPING,
    ROBOT_EVACUATING,
    ROBOT_REFILLING,
    ROBOT_WAITING
} RobotState;

typedef enum
{
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    MOVE_NONE
} MoveDirection;

typedef enum
{
    HEADING_POS_Y,
    HEADING_POS_X,
    HEADING_NEG_Y,
    HEADING_NEG_X
} GridHeading;

typedef enum
{
    TASK_NONE,
    TASK_FILL_HOLE,
    TASK_EVACUATE,
    TASK_REFILL,
    TASK_WAIT,
    TASK_RESCUE,
    TASK_COLLECT_RESOURCE
} TaskType;

typedef struct
{
    int cell_a_x;
    int cell_a_y;

    int cell_b_x;
    int cell_b_y;

    bool filled;
    bool believed_filled;
    float belief_confidence;

    int value;
    char tag_id[16];

} Hole;

typedef struct
{
    Cell pos;
    bool active;
    bool known;
    float confidence;
    int value;
} Resource;

typedef struct
{
    Cell pos;
    Cell last_pos;
    bool active;
} OtherRobot;

typedef struct
{
    TaskType type;

    Cell target;

    int hole_index;
    int resource_index;

    float utility;
    float reward;
    float risk;
    float travel;

} Task;

#endif
