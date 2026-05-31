#ifndef RESERVATION_H
#define RESERVATION_H

#include "config.h"
#include "common.h"
#include <stdbool.h>

#define TIME_DEPTH 64

typedef struct
{
    bool reserved[GRID_W][GRID_H][TIME_DEPTH];

} ReservationMap;

typedef struct
{
    bool reserved
    [GRID_W]
    [GRID_H]
    [4]
    [TIME_DEPTH];

} EdgeReservationMap;

void reservation_init(
    ReservationMap* map
);

void edge_reservation_init(
    EdgeReservationMap* map
);

bool reservation_is_reserved(
    ReservationMap* map,
    int x,
    int y,
    int t
);

void reservation_reserve(
    ReservationMap* map,
    int x,
    int y,
    int t
);

bool edge_reserved(
    EdgeReservationMap* map,
    int x,
    int y,
    int dir,
    int t
);

void reserve_edge(
    EdgeReservationMap* map,
    int x,
    int y,
    int dir,
    int t
);

#endif