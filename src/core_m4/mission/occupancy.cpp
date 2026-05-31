#include "occupancy.h"

void occupancy_init(OccupancyMap* map)
{
    for(int x = 0; x < GRID_W; x++)
    {
        for(int y = 0; y < GRID_H; y++)
        {
            map->occupancy[x][y] = 0.0f;
            map->confidence[x][y] = 1.0f;
            map->dynamic[x][y] = 0.0f;
        }
    }
}

void occupancy_observe(
    OccupancyMap* map,
    int x,
    int y,
    bool blocked
)
{
    if(x < 0 || y < 0 || x >= GRID_W || y >= GRID_H)
    {
        return;
    }

    map->occupancy[x][y] = blocked ? 1.0f : 0.0f;
    map->confidence[x][y] = 1.0f;

    if(!blocked)
    {
        map->dynamic[x][y] = 0.0f;
    }
}

void occupancy_observe_dynamic(
    OccupancyMap* map,
    int x,
    int y,
    bool blocked,
    bool dynamic
)
{
    if(x < 0 || y < 0 || x >= GRID_W || y >= GRID_H)
    {
        return;
    }

    occupancy_observe(
        map,
        x,
        y,
        blocked
    );

    map->dynamic[x][y] = blocked && dynamic ? 1.0f : 0.0f;

    if(blocked && dynamic)
    {
        map->confidence[x][y] = 0.75f;
    }
}

void occupancy_decay(
    OccupancyMap* map,
    float dt
)
{
    float decay = STALE_DECAY * dt;
    float dynamic_decay = STALE_DECAY * dt * 8.0f;

    for(int x = 0; x < GRID_W; x++)
    {
        for(int y = 0; y < GRID_H; y++)
        {
            map->confidence[x][y] -= decay;

            if(map->confidence[x][y] < 0.0f)
            {
                map->confidence[x][y] = 0.0f;
            }

            map->dynamic[x][y] -= dynamic_decay;

            if(map->dynamic[x][y] < 0.0f)
            {
                map->dynamic[x][y] = 0.0f;
            }

            if(map->dynamic[x][y] > 0.0f)
            {
                map->occupancy[x][y] -= dynamic_decay;

                if(map->occupancy[x][y] < 0.0f)
                {
                    map->occupancy[x][y] = 0.0f;
                }
            }
        }
    }
}

float occupancy_collision_risk(
    OccupancyMap* map,
    int x,
    int y
)
{
    if(x < 0 || y < 0 || x >= GRID_W || y >= GRID_H)
    {
        return 1.0f;
    }

    return
        (map->occupancy[x][y] * map->confidence[x][y]) +
        occupancy_dynamic_risk(map, x, y);
}

float occupancy_stale_risk(
    OccupancyMap* map,
    int x,
    int y
)
{
    if(x < 0 || y < 0 || x >= GRID_W || y >= GRID_H)
    {
        return 1.0f;
    }

    return 1.0f - map->confidence[x][y];
}

float occupancy_dynamic_risk(
    OccupancyMap* map,
    int x,
    int y
)
{
    if(x < 0 || y < 0 || x >= GRID_W || y >= GRID_H)
    {
        return 1.0f;
    }

    return map->dynamic[x][y] * 1.5f;
}
