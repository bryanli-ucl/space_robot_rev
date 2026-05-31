#ifndef TASKS_H
#define TASKS_H

#include "common.h"
#include "occupancy.h"
#include "rescue.h"

Task choose_best_task(
    Cell robot_pos,
    Hole* holes,
    int hole_count,
    OccupancyMap* occ,
    bool emergency,
    float time_remaining,
    DisabledRobot* disabled,
    int disabled_count,
    Resource* resources,
    int resource_count,
    int seed_inventory,
    int refill_ticks
);

#endif
