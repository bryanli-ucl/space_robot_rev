#ifndef ASTAR_H
#define ASTAR_H

#include "common.h"
#include "occupancy.h"
#include "reservation.h"


typedef struct
{
    float g;
    float h;
    float f;

    int parent_x;
    int parent_y;

    bool open;
    bool closed;

    

} AStarNode;

typedef struct
{
    Cell cells[MAX_PATH];
    int time_indices[MAX_PATH];
    int length;
} Path;

float astar_heuristic(Cell a, Cell b);

bool astar_plan_time(
    Cell start,
    GridHeading start_heading,
    Cell goal,
    OccupancyMap* occ,
    Path* out_path,
    ReservationMap* reservations,
    EdgeReservationMap* edges,
    bool emergency,
    float time_remaining
);

#endif
