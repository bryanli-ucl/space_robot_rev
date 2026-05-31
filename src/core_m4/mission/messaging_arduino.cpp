#ifdef ARDUINO

#include "messaging.h"

#include <Arduino.h>
#include <MiniMessenger.h>
#include <string.h>

#if __has_include("secrets.h")
#include "secrets.h"
static constexpr const char* MM_WIFI_SSID = WIFI_SSID;
static constexpr const char* MM_WIFI_PASS = WIFI_PASSWORD;
static constexpr const char* MM_BROKER_HOST = BROKER_HOST;
static constexpr uint16_t MM_BROKER_PORT = BROKER_PORT;
static constexpr const char* MM_GROUP_ID = GROUP_ID;
#else
#include "config.hpp"
static constexpr const char* MM_WIFI_SSID = WIFI_SSID;
static constexpr const char* MM_WIFI_PASS = WIFI_PASS;
static constexpr const char* MM_BROKER_HOST = "192.168.0.74";
static constexpr uint16_t MM_BROKER_PORT = 1883;
static constexpr const char* MM_GROUP_ID = "12";
#endif

static MiniMessenger messenger;
static MessengerState* live_state = NULL;
static OccupancyMap* live_occ = NULL;
static Hole* live_holes = NULL;
static int live_hole_count = 0;
static Resource* live_resources = NULL;
static int* live_resource_count = NULL;
static DisabledRobot* live_disabled = NULL;
static int* live_disabled_count = NULL;
static unsigned long last_pose_publish_ms = 0;
static unsigned long last_map_request_ms = 0;
static unsigned long last_register_ms = 0;
static const char* self_board_id = "Terminator";

static constexpr int MAP_REQUEST_INTERVAL_MS = 5000;
static constexpr int REGISTER_INTERVAL_MS = 10000;
static constexpr size_t BINARY_MAP_BYTES = 21;

enum MapCellState
{
    MAP_CELL_STERILE = 0,
    MAP_CELL_FERTILE = 1,
    MAP_CELL_SEEDED = 2,
    MAP_CELL_UNEXPLORED = 3
};

static int parse_int_token(const char* text, const char* key, int fallback)
{
    const char* pos = strstr(text, key);

    if(pos == NULL)
    {
        return fallback;
    }

    pos += strlen(key);

    return atoi(pos);
}

static bool parse_bool_token(const char* text, const char* key, bool fallback)
{
    const char* pos = strstr(text, key);

    if(pos == NULL)
    {
        return fallback;
    }

    pos += strlen(key);

    if(strncmp(pos, "true", 4) == 0)
    {
        return true;
    }

    if(strncmp(pos, "false", 5) == 0)
    {
        return false;
    }

    return atoi(pos) != 0;
}

static bool has_type(const char* text, const char* type)
{
    char expected[40];

    snprintf(
        expected,
        sizeof(expected),
        "type=%s",
        type
    );

    return strstr(text, expected) != NULL;
}

static MapCellState map_cell_state_at(
    const uint8_t* payload,
    int x,
    int y
)
{
    int cell_index = y * GRID_W + x;
    int bit_index = cell_index * 2;
    int byte_index = bit_index / 8;
    int shift = bit_index % 8;

    return (MapCellState)((payload[byte_index] >> shift) & 0x03);
}

static int find_hole_at(int x, int y)
{
    for(int i = 0; i < live_hole_count; i++)
    {
        if((live_holes[i].cell_a_x == x && live_holes[i].cell_a_y == y) ||
           (live_holes[i].cell_b_x == x && live_holes[i].cell_b_y == y))
        {
            return i;
        }
    }

    return -1;
}

static void apply_map_cell(
    int x,
    int y,
    MapCellState state
)
{
    int hole_index = find_hole_at(x, y);

    if(state == MAP_CELL_UNEXPLORED)
    {
        live_occ->occupancy[x][y] = 0.0f;
        live_occ->confidence[x][y] = 0.0f;
        live_occ->dynamic[x][y] = 0.0f;
        return;
    }

    occupancy_observe(live_occ, x, y, false);

    if(hole_index < 0 || hole_index >= live_hole_count)
    {
        return;
    }

    bool filled = state == MAP_CELL_STERILE || state == MAP_CELL_SEEDED;

    live_holes[hole_index].filled = filled;
    live_holes[hole_index].believed_filled = filled;
    live_holes[hole_index].belief_confidence = 1.0f;
}

static void handle_binary_map(const uint8_t* payload, size_t length)
{
    if(payload == NULL || length < BINARY_MAP_BYTES)
    {
        return;
    }

    for(int y = 0; y < GRID_H; y++)
    {
        for(int x = 0; x < GRID_W; x++)
        {
            apply_map_cell(x, y, map_cell_state_at(payload, x, y));
        }
    }
}

static void handle_team_status(const uint8_t* payload, size_t length)
{
    if(payload == NULL || length < 6 || live_state == NULL)
    {
        return;
    }

    if(payload[4] != 0)
    {
        live_state->emergency = true;
    }
}

static void handle_cell_message(const char* text)
{
    int x = parse_int_token(text, "x=", -1);
    int y = parse_int_token(text, "y=", -1);
    bool occupied = parse_bool_token(text, "occupied=", true);
    bool dynamic = parse_bool_token(text, "dynamic=", false);

    if(x < 0 || x >= GRID_W || y < 0 || y >= GRID_H)
    {
        return;
    }

    occupancy_observe_dynamic(
        live_occ,
        x,
        y,
        occupied,
        dynamic
    );
}

static void request_whole_map(const char* reason)
{
    if(!messenger.isConnected())
    {
        return;
    }

    char payload[96];

    snprintf(
        payload,
        sizeof(payload),
        "type=getMap team_id=%s board_id=%s reason=%s",
        MM_GROUP_ID,
        self_board_id,
        reason
    );

    messenger.sendToGroup(payload);
}

static void handle_fertile_reply(const char* text)
{
    int x = parse_int_token(text, "x=", -1);
    int y = parse_int_token(text, "y=", -1);
    bool fertile = parse_bool_token(text, "fertile=", true);
    bool planted = parse_bool_token(text, "planted=", false);
    int hole_index = parse_int_token(text, "hole=", -1);

    if(hole_index < 0)
    {
        hole_index = find_hole_at(x, y);
    }

    if(hole_index < 0 || hole_index >= live_hole_count)
    {
        return;
    }

    bool filled = planted || !fertile;

    live_holes[hole_index].filled = filled;
    live_holes[hole_index].believed_filled = filled;
    live_holes[hole_index].belief_confidence = 1.0f;

    occupancy_observe(
        live_occ,
        live_holes[hole_index].cell_a_x,
        live_holes[hole_index].cell_a_y,
        false
    );
}

static void handle_text_map_cell(const char* text)
{
    int x = parse_int_token(text, "x=", -1);
    int y = parse_int_token(text, "y=", -1);
    int state = parse_int_token(text, "state=", MAP_CELL_UNEXPLORED);

    if(x < 0 || x >= GRID_W || y < 0 || y >= GRID_H)
    {
        return;
    }

    apply_map_cell(x, y, (MapCellState)state);
}

static void handle_resource_message(const char* text)
{
    if(live_resource_count == NULL || *live_resource_count >= MAX_RESOURCES)
    {
        return;
    }

    int x = parse_int_token(text, "x=", -1);
    int y = parse_int_token(text, "y=", -1);
    int value = parse_int_token(text, "value=", 20);
    bool active = parse_bool_token(text, "active=", true);

    if(x < 0 || x >= GRID_W || y < 0 || y >= GRID_H)
    {
        return;
    }

    Resource* resource = &live_resources[*live_resource_count];
    resource->pos.x = x;
    resource->pos.y = y;
    resource->active = active;
    resource->known = true;
    resource->confidence = 1.0f;
    resource->value = value;

    (*live_resource_count)++;
}

static void handle_help_message(const char* text)
{
    int x = parse_int_token(text, "x=", -1);
    int y = parse_int_token(text, "y=", -1);

    if(x < 0 || x >= GRID_W || y < 0 || y >= GRID_H)
    {
        return;
    }

    if(live_disabled_count == NULL || *live_disabled_count >= MAX_DISABLED)
    {
        return;
    }

    spawn_disabled_robot(
        live_disabled,
        live_disabled_count,
        x,
        y
    );

    occupancy_observe_dynamic(
        live_occ,
        x,
        y,
        true,
        false
    );
}

static void handle_distress_message(const char* text)
{
    if(live_disabled_count == NULL || *live_disabled_count >= MAX_DISABLED)
    {
        return;
    }

    const char* robot = strstr(text, "robot0=");

    if(robot == NULL)
    {
        robot = strstr(text, "robot=");
    }

    if(robot == NULL)
    {
        handle_help_message(text);
        return;
    }

    const char* first_comma = strchr(robot, ',');
    const char* second_comma = first_comma == NULL ? NULL : strchr(first_comma + 1, ',');

    if(first_comma == NULL || second_comma == NULL)
    {
        return;
    }

    int x = atoi(first_comma + 1);
    int y = atoi(second_comma + 1);

    if(x < 0 || x >= GRID_W || y < 0 || y >= GRID_H)
    {
        return;
    }

    spawn_disabled_robot(
        live_disabled,
        live_disabled_count,
        x,
        y
    );

    occupancy_observe_dynamic(live_occ, x, y, true, false);
}

static void on_message(
    const MessageMetadata& metadata,
    const uint8_t* payload,
    size_t length
)
{
    (void)metadata;

    if(live_state == NULL || live_occ == NULL)
    {
        return;
    }

    if(length == BINARY_MAP_BYTES)
    {
        handle_binary_map(payload, length);
        return;
    }

    if(length == 6)
    {
        handle_team_status(payload, length);
        return;
    }

    char text[128];
    size_t n = length;

    if(n >= sizeof(text))
    {
        n = sizeof(text) - 1;
    }

    memcpy(text, payload, n);
    text[n] = '\0';

    if(has_type(text, "heartbeat"))
    {
        live_state->enabled = parse_bool_token(text, "enable=", live_state->enabled);
        return;
    }

    if(has_type(text, "emergency"))
    {
        live_state->emergency = parse_bool_token(text, "enabled=", true);
        return;
    }

    if(has_type(text, "disable"))
    {
        live_state->enabled = parse_bool_token(text, "enabled=", false);
        return;
    }

    if(has_type(text, "isFertileReply"))
    {
        handle_fertile_reply(text);
        return;
    }

    if(has_type(text, "map") || has_type(text, "mapCell"))
    {
        handle_text_map_cell(text);
        return;
    }

    if(strstr(text, "CELL") != NULL || strstr(text, "type=cell") != NULL)
    {
        handle_cell_message(text);
        return;
    }

    if(strstr(text, "HOLE") != NULL || strstr(text, "type=hole") != NULL)
    {
        handle_fertile_reply(text);
        return;
    }

    if(strstr(text, "RESOURCE") != NULL || strstr(text, "SEED") != NULL)
    {
        handle_resource_message(text);
        return;
    }

    if(strstr(text, "HELP") != NULL || strstr(text, "DISABLED") != NULL)
    {
        handle_help_message(text);
        return;
    }

    if(strstr(text, "distress") != NULL || strstr(text, "DISTRESS") != NULL)
    {
        handle_distress_message(text);
        return;
    }
}

void messaging_begin(const char* board_id)
{
    self_board_id = board_id;

    messenger.onMessage(on_message);
    messenger.begin(
        MM_WIFI_SSID,
        MM_WIFI_PASS,
        MM_BROKER_HOST,
        MM_BROKER_PORT,
        MM_GROUP_ID,
        board_id
    );
}

void messaging_loop(
    MessengerState* state,
    OccupancyMap* occ,
    Hole* holes,
    int hole_count,
    Resource* resources,
    int* resource_count,
    DisabledRobot* disabled,
    int* disabled_count
)
{
    live_state = state;
    live_occ = occ;
    live_holes = holes;
    live_hole_count = hole_count;
    live_resources = resources;
    live_resource_count = resource_count;
    live_disabled = disabled;
    live_disabled_count = disabled_count;

    messenger.loop();

    if(messenger.isConnected() && millis() - last_register_ms > REGISTER_INTERVAL_MS)
    {
        char payload[96];

        snprintf(
            payload,
            sizeof(payload),
            "type=register team_id=%s board_id=%s",
            MM_GROUP_ID,
            self_board_id
        );

        messenger.sendToGroup(payload);
        last_register_ms = millis();
        request_whole_map("register");
    }
}

void messaging_publish_pose(
    Cell pos,
    Task task,
    float score,
    int seed_inventory
)
{
    if(!messenger.isConnected() || millis() - last_pose_publish_ms < 1000)
    {
        return;
    }

    last_pose_publish_ms = millis();

    char payload[128];

    snprintf(
        payload,
        sizeof(payload),
        "type=pose team_id=%s board_id=%s x=%d y=%d task=%d target_x=%d target_y=%d score=%ld seeds=%d",
        MM_GROUP_ID,
        self_board_id,
        pos.x,
        pos.y,
        (int)task.type,
        task.target.x,
        task.target.y,
        (long)score,
        seed_inventory
    );

    messenger.sendToGroup(payload);
}

void messaging_publish_seed_planted(
    Hole* hole
)
{
    if(!messenger.isConnected() || hole == NULL)
    {
        return;
    }

    char payload[128];

    snprintf(
        payload,
        sizeof(payload),
        "type=seedPlanted tag_id=%s team_id=%s board_id=%s x=%d y=%d",
        hole->tag_id,
        MM_GROUP_ID,
        self_board_id,
        hole->cell_a_x,
        hole->cell_a_y
    );

    messenger.sendToGroup(payload);
}

void messaging_request_map_updates(
    Cell robot_pos,
    Task task,
    OccupancyMap* occ,
    Hole* holes,
    int hole_count
)
{
    if(!messenger.isConnected() || millis() - last_map_request_ms < MAP_REQUEST_INTERVAL_MS)
    {
        return;
    }

    (void)robot_pos;
    (void)task;
    (void)occ;
    (void)holes;
    (void)hole_count;

    last_map_request_ms = millis();
    request_whole_map("periodic");
}

#endif
