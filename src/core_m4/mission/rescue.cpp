#include "rescue.h"

void spawn_disabled_robot(
    DisabledRobot* robots,
    int* count,
    int x,
    int y
)
{
    robots[*count].pos.x = x;
    robots[*count].pos.y = y;

    robots[*count].rescued = false;

    robots[*count].disable_time = 0.0f;

    (*count)++;
}

void mark_rescued(DisabledRobot* r)
{
    r->rescued = true;
}
