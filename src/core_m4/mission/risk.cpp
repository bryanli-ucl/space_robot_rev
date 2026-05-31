#include "risk.h"
#include "config.h"
#include "reservation.h"


static float bottleneck_risk(int x)
{
    if(x <= 1)
    {
        return W_BOTTLENECK;
    }

    return 0.0f;
}

static float evacuation_risk(
    int x,
    bool emergency
)
{
    if(!emergency)
    {
        return 0.0f;
    }

    return ((float)x / GRID_W) * W_EVACUATION;
}

static float time_risk(float time_remaining)
{
    if(time_remaining < 60.0f)
    {
        return W_TIME;
    }

    return 0.0f;
}

float compute_cell_risk(
    OccupancyMap* occ,
    int x,
    int y,
    bool emergency,
    float time_remaining
)
{
    float collision =
        occupancy_collision_risk(occ, x, y) * W_COLLISION;

    float stale =
        occupancy_stale_risk(occ, x, y) * W_STALE;

    float bottleneck = bottleneck_risk(x);

    float evac = evacuation_risk(x, emergency);

    float time = time_risk(time_remaining);

    return
        collision +
        stale +
        bottleneck +
        evac +
        time;
}

float compute_rescue_risk(
    OccupancyMap* occ,
    int x,
    int y,
    bool emergency,
    float time_remaining
)
{
    float base =
        compute_cell_risk(
            occ,
            x,
            y,
            emergency,
            time_remaining
        );

    float blocked_body_risk = 6.0f;
    float interaction_delay_risk = emergency ? 8.0f : 2.0f;

    return base + blocked_body_risk + interaction_delay_risk;
}

float likely_robot_arrival_penalty(Cell target)
{
    float right_side_competition =
        target.x >= HIGH_VALUE_ZONE_X ? 6.0f : 2.0f;

    float entry_congestion =
        target.x <= 2 ? 4.0f : 0.0f;

    return right_side_competition + entry_congestion;
}

float compute_path_risk(
    OccupancyMap* occ,
    Cell* cells,
    int count,
    bool emergency,
    float time_remaining
)
{
    float total = 0.0f;

    for(int i = 0; i < count; i++)
    {
        total += compute_cell_risk(
            occ,
            cells[i].x,
            cells[i].y,
            emergency,
            time_remaining
        );
    }

    return total;
}

float future_congestion_risk(
    ReservationMap* reservations,
    int x,
    int y,
    int time_index
)
{
    if(reservation_is_reserved(
        reservations,
        x,
        y,
        time_index
    ))
    {
        return 5.0f;
    }

    return 0.0f;
}

float future_collision_probability(
    ReservationMap* reservations,
    EdgeReservationMap* edges,
    int x,
    int y,
    int time_index
)
{
    float risk = 0.0f;

    if(reservation_is_reserved(
        reservations,
        x,
        y,
        time_index
    ))
    {
        risk += 8.0f;
    }

    for(int d = 0; d < 4; d++)
    {
        if(edge_reserved(
            edges,
            x,
            y,
            d,
            time_index
        ))
        {
            risk += 4.0f;
        }
    }

    return risk;
}
