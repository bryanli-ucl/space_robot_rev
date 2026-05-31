#include "rfid_map.h"
#include "movement.h"

#include <stdio.h>

static RfidTagMap global_rfid_map;

static bool cell_in_bounds(Cell cell)
{
    return cell.x >= 0 && cell.y >= 0 && cell.x < GRID_W && cell.y < GRID_H;
}

RfidTagMap* rfid_map_global(void)
{
    return &global_rfid_map;
}

void rfid_map_init(RfidTagMap* map)
{
    for(int x = 0; x < GRID_W; x++)
    {
        for(int y = 0; y < GRID_H; y++)
        {
            map->valid[x][y] = false;
            map->tag_id[x][y][0] = '\0';
        }
    }
}

bool rfid_map_set(
    RfidTagMap* map,
    Cell cell,
    const char* tag_id
)
{
    if(map == NULL || tag_id == NULL || !cell_in_bounds(cell))
    {
        return false;
    }

    snprintf(
        map->tag_id[cell.x][cell.y],
        RFID_TAG_ID_LEN,
        "%s",
        tag_id
    );

    map->valid[cell.x][cell.y] = map->tag_id[cell.x][cell.y][0] != '\0';

    return map->valid[cell.x][cell.y];
}

const char* rfid_map_get(
    RfidTagMap* map,
    Cell cell
)
{
    if(map == NULL || !cell_in_bounds(cell) || !map->valid[cell.x][cell.y])
    {
        return NULL;
    }

    return map->tag_id[cell.x][cell.y];
}

void rfid_scan_robot_init(
    Cell* robot_pos
)
{
    if(robot_pos == NULL)
    {
        return;
    }

    robot_pos->x = RFID_SCAN_START_X;
    robot_pos->y = RFID_SCAN_START_Y;
}

int rfid_scan_all_seed_drops(
    Cell* robot_pos,
    RfidTagMap* map
)
{
    int scanned = 0;
    GridHeading heading = HEADING_POS_Y;

    if(robot_pos == NULL || map == NULL)
    {
        return 0;
    }

    rfid_scan_robot_init(robot_pos);

    for(int y = 0; y < GRID_H; y++)
    {
        int start_x = (y % 2 == 0) ? 0 : GRID_W - 1;
        int end_x = (y % 2 == 0) ? GRID_W : -1;
        int step_x = (y % 2 == 0) ? 1 : -1;

        for(int x = start_x; x != end_x; x += step_x)
        {
            char tag_id[RFID_TAG_ID_LEN];
            Cell cell = {x, y};

            robot_pos->x = x;
            robot_pos->y = y;

            if(read_rfid_tag(tag_id, sizeof(tag_id)))
            {
                if(rfid_map_set(map, cell, tag_id))
                {
                    scanned++;
                }
            }

            if(x + step_x != end_x)
            {
                execute_move_to_heading(
                    &heading,
                    step_x > 0 ? HEADING_POS_X : HEADING_NEG_X
                );
            }
        }

        if(y + 1 < GRID_H)
        {
            execute_move_to_heading(&heading, HEADING_POS_Y);
        }
    }

    return scanned;
}
