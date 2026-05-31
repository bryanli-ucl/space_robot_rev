#include "world_events.h"
#include <stdlib.h>

static int clamp_coord(int value, int max)
{
    if(value < 0)
        return 0;

    if(value >= max)
        return max - 1;

    return value;
}

static bool is_local(Cell a, Cell b)
{
    int dx = a.x - b.x;
    int dy = a.y - b.y;

    if(dx < 0)
        dx = -dx;

    if(dy < 0)
        dy = -dy;

    return dx <= 1 && dy <= 1;
}

static void init_hole(
    Hole* hole,
    int ax,
    int ay,
    int bx,
    int by,
    bool filled,
    int value,
    const char* tag_id
)
{
    int i = 0;

    hole->cell_a_x = ax;
    hole->cell_a_y = ay;
    hole->cell_b_x = bx;
    hole->cell_b_y = by;
    hole->filled = filled;
    hole->believed_filled = filled;
    hole->belief_confidence = 1.0f;
    hole->value = value;

    while(tag_id[i] != '\0' && i < 15)
    {
        hole->tag_id[i] = tag_id[i];
        i++;
    }

    hole->tag_id[i] = '\0';
}

void drift_obstacles(void)
{
    // Kept for compatibility; moving obstacle drift is modelled by update_other_robots().
}

void other_robots_init(
    OtherRobot* robots,
    int count
)
{
    for(int i = 0; i < count; i++)
    {
        robots[i].pos.x = rand() % GRID_W;
        robots[i].pos.y = rand() % GRID_H;
        robots[i].last_pos = robots[i].pos;
        robots[i].active = true;
    }
}

void update_other_robots(
    OtherRobot* robots,
    int count
)
{
    const int dx[5] = {1, -1, 0, 0, 0};
    const int dy[5] = {0, 0, 1, -1, 0};

    for(int i = 0; i < count; i++)
    {
        if(!robots[i].active)
            continue;

        int d = rand() % 5;

        robots[i].last_pos = robots[i].pos;
        robots[i].pos.x = clamp_coord(robots[i].pos.x + dx[d], GRID_W);
        robots[i].pos.y = clamp_coord(robots[i].pos.y + dy[d], GRID_H);
    }
}

void resources_init(
    Resource* resources,
    int* count
)
{
    *count = 4;

    for(int i = 0; i < *count; i++)
    {
        int high_zone_width = GRID_W - HIGH_VALUE_ZONE_X;

        if(high_zone_width <= 0)
        {
            resources[i].pos.x = rand() % GRID_W;
        }
        else
        {
            resources[i].pos.x = HIGH_VALUE_ZONE_X + (rand() % high_zone_width);
        }

        resources[i].pos.y = rand() % GRID_H;
        resources[i].active = true;
        resources[i].known = false;
        resources[i].confidence = 0.0f;
        resources[i].value = 15 + (rand() % 20);
    }
}

void known_holes_init(
    Hole* holes,
    int* hole_count
)
{
    *hole_count = 20;

    init_hole(&holes[0], 1, 1, 1, 2, false, 15, "TAG00");
    init_hole(&holes[1], 2, 3, 3, 3, false, 20, "TAG01");
    init_hole(&holes[2], 4, 1, 4, 2, false, 18, "TAG02");
    init_hole(&holes[3], 6, 2, 7, 2, false, 25, "TAG03");
    init_hole(&holes[4], 8, 1, 8, 2, false, 30, "TAG04");
    init_hole(&holes[5], 1, 5, 2, 5, false, 35, "TAG05");
    init_hole(&holes[6], 3, 6, 3, 7, false, 28, "TAG06");
    init_hole(&holes[7], 5, 4, 6, 4, false, 35, "TAG07");
    init_hole(&holes[8], 7, 5, 7, 6, false, 38, "TAG08");
    init_hole(&holes[9], 6, 7, 7, 7, false, 55, "TAG09");
    init_hole(&holes[10], 0, 8, 1, 8, false, 60, "TAG10");
    init_hole(&holes[11], 2, 0, 3, 0, false, 70, "TAG11");
    init_hole(&holes[12], 5, 8, 6, 8, false, 80, "TAG12");
    init_hole(&holes[13], 8, 6, 8, 7, false, 75, "TAG13");
    init_hole(&holes[14], 0, 4, 1, 4, false, 85, "TAG14");
    init_hole(&holes[15], 4, 5, 4, 6, false, 90, "TAG15");
    init_hole(&holes[16], 5, 0, 6, 0, false, 22, "TAG16");
    init_hole(&holes[17], 7, 0, 8, 0, false, 24, "TAG17");
    init_hole(&holes[18], 2, 8, 3, 8, false, 32, "TAG18");
    init_hole(&holes[19], 8, 3, 8, 4, false, 45, "TAG19");
}

void resource_events(
    Resource* resources,
    int* count
)
{
    for(int i = 0; i < *count; i++)
    {
        if(resources[i].known)
        {
            resources[i].confidence -= 0.01f;

            if(resources[i].confidence <= 0.0f)
            {
                resources[i].known = false;
                resources[i].confidence = 0.0f;
            }
        }

        if(resources[i].active && rand() % 1000 < 3)
        {
            resources[i].active = false;
            resources[i].known = false;
            resources[i].confidence = 0.0f;
        }
    }

    if(*count < MAX_RESOURCES && rand() % 1000 < 12)
    {
        Resource* resource = &resources[*count];

        resource->pos.x = rand() % GRID_W;
        resource->pos.y = rand() % GRID_H;
        resource->active = true;
        resource->known = false;
        resource->confidence = 0.0f;
        resource->value = 10 + (rand() % 20);

        if(resource->pos.x >= HIGH_VALUE_ZONE_X)
        {
            resource->value *= 2;
        }

        (*count)++;
    }
}

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
)
{
    for(int i = 0; i < other_robot_count; i++)
    {
        if(other_robots[i].active && is_local(observer, other_robots[i].pos))
        {
            occupancy_observe(
                occ,
                other_robots[i].pos.x,
                other_robots[i].pos.y,
                true
            );
        }
    }

    for(int i = 0; i < disabled_count; i++)
    {
        if(!disabled[i].rescued && is_local(observer, disabled[i].pos))
        {
            occupancy_observe(
                occ,
                disabled[i].pos.x,
                disabled[i].pos.y,
                true
            );
        }
    }

    for(int i = 0; i < resource_count; i++)
    {
        if(resources[i].active && is_local(observer, resources[i].pos))
        {
            resources[i].known = true;
            resources[i].confidence = 1.0f;
        }
    }

    for(int i = 0; i < hole_count; i++)
    {
        Cell a = {holes[i].cell_a_x, holes[i].cell_a_y};
        Cell b = {holes[i].cell_b_x, holes[i].cell_b_y};

        if(is_local(observer, a) || is_local(observer, b))
        {
            holes[i].believed_filled = holes[i].filled;
            holes[i].belief_confidence = 1.0f;
        }
    }
}

void mutate_holes(
    Hole* holes,
    int hole_count
)
{
    for(int i = 0; i < hole_count; i++)
    {
        int r = rand() % 1000;

        if(r < 2)
        {
            holes[i].filled = !holes[i].filled;
        }

        if(holes[i].belief_confidence > 0.0f)
        {
            holes[i].belief_confidence -= 0.002f;

            if(holes[i].belief_confidence < 0.0f)
            {
                holes[i].belief_confidence = 0.0f;
            }
        }
    }
}

void technician_events(
    Hole* holes,
    int hole_count,
    Resource* resources,
    int* resource_count
)
{
    mutate_holes(
        holes,
        hole_count
    );

    resource_events(
        resources,
        resource_count
    );
}
