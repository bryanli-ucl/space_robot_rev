#ifndef WORLD_EVENTS_H
#define WORLD_EVENTS_H

#include "common.h"
#include "occupancy.h"
#include "rescue.h"

void drift_obstacles(void);

void other_robots_init(
    OtherRobot* robots,
    int count
);

void update_other_robots(
    OtherRobot* robots,
    int count
);

void resources_init(
    Resource* resources,
    int* count
);

void known_holes_init(
    Hole* holes,
    int* hole_count
);

void resource_events(
    Resource* resources,
    int* count
);

void observe_local_environment(
    OccupancyMap* occ,
    Cell observer,
    OtherRobot* other_robots,
    int other_robot_count,
    DisabledRobot* disabled,
    int disabled_count,
    Resource* resources,
    int resource_count,
    Hole* holes,
    int hole_count
);

void mutate_holes(
    Hole* holes,
    int hole_count
);

void technician_events(
    Hole* holes,
    int hole_count,
    Resource* resources,
    int* resource_count
);

#endif
