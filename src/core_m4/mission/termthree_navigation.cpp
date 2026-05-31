#include "termthree_navigation.h"

#include "occupancy.h"
#include "messaging.h"
#include "reservation.h"
#include "rescue.h"
#include "rfid_map.h"
#include "tasks.h"
#include "world_events.h"

#include <Arduino.h>
#include <stdlib.h>

static OccupancyMap occ;
static Robot robot;
static ReservationMap reservations;
static EdgeReservationMap edges;
static Hole holes[MAX_HOLES];
static DisabledRobot disabled[MAX_DISABLED];
static Resource resources[MAX_RESOURCES];
static OtherRobot other_robots[MAX_OTHER_ROBOTS];

static int hole_count = 0;
static int disabled_count = 0;
static int resource_count = 0;
static bool emergency = false;
static float time_remaining = 300.0f;
static bool nav_enabled = false;
static MessengerState messenger_state = {true, false};
static unsigned long last_tick_ms = 0;

static void init_other_robots_inactive(void)
{
    for(int i = 0; i < MAX_OTHER_ROBOTS; i++)
    {
        other_robots[i].active = false;
        other_robots[i].pos.x = 0;
        other_robots[i].pos.y = 0;
        other_robots[i].last_pos = other_robots[i].pos;
    }
}

void termthree_navigation_begin(void)
{
    occupancy_init(&occ);
    robot_init(&robot, 0, ENTRY_X, ENTRY_Y);

    RfidTagMap* rfid_tags = rfid_map_global();
    rfid_map_init(rfid_tags);

    if(RFID_BOOTSTRAP_SCAN_ON_START)
    {
        rfid_scan_all_seed_drops(&robot.pos, rfid_tags);
    }

    reservation_init(&reservations);
    edge_reservation_init(&edges);
    known_holes_init(holes, &hole_count);
    resources_init(resources, &resource_count);
    init_other_robots_inactive();

    disabled_count = 0;
    emergency = false;
    time_remaining = 300.0f;
    nav_enabled = ROBOT_START_ARMED != 0;
    messenger_state.enabled = true;
    messenger_state.emergency = false;
    last_tick_ms = millis();

    messaging_begin("Terminator");
}

void termthree_navigation_tick(void)
{
    const unsigned long now_ms = millis();
    const float dt_s = last_tick_ms == 0 ? 0.1f : (float)(now_ms - last_tick_ms) / 1000.0f;
    last_tick_ms = now_ms;

    occupancy_decay(&occ, dt_s);

    messaging_loop(
        &messenger_state,
        &occ,
        holes,
        hole_count,
        resources,
        &resource_count,
        disabled,
        &disabled_count
    );

    emergency = messenger_state.emergency;

    messaging_request_map_updates(
        robot.pos,
        robot.task,
        &occ,
        holes,
        hole_count
    );

    if(!nav_enabled || !messenger_state.enabled)
    {
        return;
    }

    observe_local_environment(
        &occ,
        robot.pos,
        other_robots,
        MAX_OTHER_ROBOTS,
        disabled,
        disabled_count,
        resources,
        resource_count,
        holes,
        hole_count
    );

    technician_events(
        holes,
        hole_count,
        resources,
        &resource_count
    );

    reservation_init(&reservations);
    edge_reservation_init(&edges);

    robot_update(
        &robot,
        &occ,
        holes,
        hole_count,
        emergency,
        time_remaining,
        disabled,
        disabled_count,
        &reservations,
        &edges,
        resources,
        resource_count
    );

    messaging_publish_pose(
        robot.pos,
        robot.task,
        robot.score,
        robot.seed_inventory
    );

    if(robot.seed_planted_event &&
       robot.seed_planted_hole_index >= 0 &&
       robot.seed_planted_hole_index < hole_count)
    {
        messaging_publish_seed_planted(
            &holes[robot.seed_planted_hole_index]
        );

        robot.seed_planted_event = false;
        robot.seed_planted_hole_index = -1;
    }

    time_remaining -= dt_s;
    if(time_remaining < 60.0f)
    {
        emergency = true;
    }
}

void termthree_navigation_set_enabled(bool enabled)
{
    nav_enabled = enabled;
}

bool termthree_navigation_enabled(void)
{
    return nav_enabled;
}

const Robot* termthree_navigation_robot(void)
{
    return &robot;
}
