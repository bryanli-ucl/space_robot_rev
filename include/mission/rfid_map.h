#ifndef RFID_MAP_H
#define RFID_MAP_H

#include "common.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    bool valid[GRID_W][GRID_H];
    char tag_id[GRID_W][GRID_H][RFID_TAG_ID_LEN];
} RfidTagMap;

RfidTagMap* rfid_map_global(void);

void rfid_map_init(RfidTagMap* map);

bool rfid_map_set(
    RfidTagMap* map,
    Cell cell,
    const char* tag_id
);

const char* rfid_map_get(
    RfidTagMap* map,
    Cell cell
);

void rfid_scan_robot_init(
    Cell* robot_pos
);

int rfid_scan_all_seed_drops(
    Cell* robot_pos,
    RfidTagMap* map
);

#ifdef __cplusplus
}
#endif

#endif

