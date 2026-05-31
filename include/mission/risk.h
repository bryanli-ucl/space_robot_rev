#ifndef RISK_H
#define RISK_H

#include "common.h"
#include "occupancy.h"
#include "reservation.h"

float compute_cell_risk(
    OccupancyMap* occ,
    int x,
    int y,
    bool emergency,
    float time_remaining
);

float compute_rescue_risk(
    OccupancyMap* occ,
    int x,
    int y,
    bool emergency,
    float time_remaining
);

float likely_robot_arrival_penalty(Cell target);

float compute_path_risk(
    OccupancyMap* occ,
    Cell* cells,
    int count,
    bool emergency,
    float time_remaining
);


float future_congestion_risk(
    ReservationMap* reservations,
    int x,
    int y,
    int time_index
);

float future_collision_probability(
    ReservationMap* reservations,
    EdgeReservationMap* edges,
    int x,
    int y,
    int time_index
);


#endif
