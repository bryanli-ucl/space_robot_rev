#ifndef ROBOT_H
#define ROBOT_H

#include "common.h"
#include "astar.h"
#include "occupancy.h"
#include "reservation.h"
#include "rescue.h"

typedef struct
{
    int id;

    Cell pos;
    GridHeading heading;

    RobotState state;

    Path path;

    int path_index;

    int seed_inventory;

    float score;

    Task task;

    bool can_rescue;
    int refill_ticks;
    bool seed_planted_event;
    int seed_planted_hole_index;

} Robot;

void robot_init(
    Robot* r,
    int id,
    int x,
    int y
);

void robot_update(
    Robot* r,
    OccupancyMap* occ,
    Hole* holes,
    int hole_count,
    bool emergency,
    float time_remaining,
    DisabledRobot* disabled,
    int disabled_count,
    ReservationMap* reservations,
    EdgeReservationMap* edges,
    Resource* resources,
    int resource_count
);

#endif
