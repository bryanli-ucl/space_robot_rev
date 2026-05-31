#ifndef RESCUE_H
#define RESCUE_H

#include "common.h"

typedef struct
{
    Cell pos;

    bool rescued;

    float disable_time;

} DisabledRobot;

void spawn_disabled_robot(
    DisabledRobot* robots,
    int* count,
    int x,
    int y
);

void mark_rescued(DisabledRobot* r);

#endif
