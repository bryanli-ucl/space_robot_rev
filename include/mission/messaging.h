#ifndef MESSAGING_H
#define MESSAGING_H

#include "common.h"
#include "occupancy.h"
#include "rescue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    bool enabled;
    bool emergency;
} MessengerState;

void messaging_begin(const char* board_id);

void messaging_loop(
    MessengerState* state,
    OccupancyMap* occ,
    Hole* holes,
    int hole_count,
    Resource* resources,
    int* resource_count,
    DisabledRobot* disabled,
    int* disabled_count
);

void messaging_publish_pose(
    Cell pos,
    Task task,
    float score,
    int seed_inventory
);

void messaging_publish_seed_planted(
    Hole* hole
);

void messaging_request_map_updates(
    Cell robot_pos,
    Task task,
    OccupancyMap* occ,
    Hole* holes,
    int hole_count
);

#ifdef __cplusplus
}
#endif

#endif
