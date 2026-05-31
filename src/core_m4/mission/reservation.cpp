#include "reservation.h"

void reservation_init(
    ReservationMap* map
)
{
    for(int x = 0; x < GRID_W; x++)
    {
        for(int y = 0; y < GRID_H; y++)
        {
            for(int t = 0; t < TIME_DEPTH; t++)
            {
                map->reserved[x][y][t] = false;
            }
        }
    }
}

void edge_reservation_init(
    EdgeReservationMap* map
)
{
    for(int x = 0; x < GRID_W; x++)
    {
        for(int y = 0; y < GRID_H; y++)
        {
            for(int d = 0; d < 4; d++)
            {
                for(int t = 0; t < TIME_DEPTH; t++)
                {
                    map->reserved[x][y][d][t] = false;
                }
            }
        }
    }
}

bool reservation_is_reserved(
    ReservationMap* map,
    int x,
    int y,
    int t
)
{
    if(x < 0 || x >= GRID_W || y < 0 || y >= GRID_H || t < 0 || t >= TIME_DEPTH)
        return false;

    return map->reserved[x][y][t];
}

void reservation_reserve(
    ReservationMap* map,
    int x,
    int y,
    int t
)
{
    if(x < 0 || x >= GRID_W || y < 0 || y >= GRID_H || t < 0 || t >= TIME_DEPTH)
        return;

    map->reserved[x][y][t] = true;
}

bool edge_reserved(
    EdgeReservationMap* map,
    int x,
    int y,
    int dir,
    int t
)
{
    if(x < 0 || x >= GRID_W || y < 0 || y >= GRID_H || dir < 0 || dir >= 4 || t < 0 || t >= TIME_DEPTH)
        return false;

    return map->reserved[x][y][dir][t];
}

void reserve_edge(
    EdgeReservationMap* map,
    int x,
    int y,
    int dir,
    int t
)
{
    if(x < 0 || x >= GRID_W || y < 0 || y >= GRID_H || dir < 0 || dir >= 4 || t < 0 || t >= TIME_DEPTH)
        return;

    map->reserved[x][y][dir][t] = true;
}
