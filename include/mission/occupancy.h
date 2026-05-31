#ifndef OCCUPANCY_H
#define OCCUPANCY_H

#include <stdbool.h>
#include "common.h"
#include "config.h"

typedef struct
{
    float occupancy[GRID_W][GRID_H];
    float confidence[GRID_W][GRID_H];
    float dynamic[GRID_W][GRID_H];

} OccupancyMap;

void occupancy_init(OccupancyMap* map);

void occupancy_observe(
    OccupancyMap* map,
    int x,
    int y,
    bool blocked
);

void occupancy_observe_dynamic(
    OccupancyMap* map,
    int x,
    int y,
    bool blocked,
    bool dynamic
);

void occupancy_decay(
    OccupancyMap* map,
    float dt
);

float occupancy_collision_risk(
    OccupancyMap* map,
    int x,
    int y
);

float occupancy_stale_risk(
    OccupancyMap* map,
    int x,
    int y
);

float occupancy_dynamic_risk(
    OccupancyMap* map,
    int x,
    int y
);

#endif
