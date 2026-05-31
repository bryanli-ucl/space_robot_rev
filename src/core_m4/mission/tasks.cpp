#include "tasks.h"
#include "risk.h"
#include <math.h>
#include "rescue.h"

static float dist(Cell a, Cell b)
{
    return fabsf((float)(a.x - b.x)) + fabsf((float)(a.y - b.y));
}

Task choose_best_task(
    Cell robot_pos,
    Hole* holes,
    int hole_count,
    OccupancyMap* occ,
    bool emergency,
    float time_remaining,
    DisabledRobot* disabled,
    int disabled_count,
    Resource* resources,
    int resource_count,
    int seed_inventory,
    int refill_ticks
)
{
    Task best;

    best.type = TASK_WAIT;
    best.target = robot_pos;
    best.hole_index = -1;
    best.resource_index = -1;
    best.reward = 0.0f;
    best.risk = 0.0f;
    best.travel = 0.0f;

    best.utility = -999999.0f;

    if(emergency)
    {
        Cell exit = {EXIT_X, EXIT_Y};
        float travel = dist(robot_pos, exit);
        float risk = compute_cell_risk(occ, exit.x, exit.y, true, time_remaining);

        best.type = TASK_EVACUATE;
        best.target = exit;
        best.travel = travel;
        best.reward = 500.0f;
        best.risk = risk;
        best.utility = best.reward - best.risk - best.travel;

        return best;
    }

    if(refill_ticks > 0)
    {
        best.type = TASK_WAIT;
        best.utility = 0.0f;

        return best;
    }

    if(seed_inventory <= 0)
    {
        Cell entry = {ENTRY_X, ENTRY_Y};
        float travel = dist(robot_pos, entry);
        float risk = compute_cell_risk(occ, entry.x, entry.y, false, time_remaining);

        best.type = TASK_REFILL;
        best.target = entry;
        best.travel = travel;
        best.reward = 40.0f;
        best.risk = risk;
        best.utility = best.reward - best.risk - best.travel;
    }

    for(int i = 0; i < disabled_count; i++)
    {
        if(disabled[i].rescued)
            continue;

        Cell target = disabled[i].pos;

        float travel =
            dist(robot_pos, target);

        float risk =
            compute_rescue_risk(
                occ,
                target.x,
                target.y,
                emergency,
                time_remaining
            );

        float competition =
            likely_robot_arrival_penalty(target);

        float reward = 80.0f;

        float utility =
            reward -
            risk -
            travel -
            competition;

        if(utility > best.utility)
        {
            best.type = TASK_RESCUE;

            best.target = target;
            best.travel = travel;
            best.reward = reward;
            best.risk = risk;

            best.utility = utility;
        }
    }

    if(seed_inventory < SEED_CAPACITY)
    {
        for(int i = 0; i < resource_count; i++)
        {
            if(!resources[i].active || !resources[i].known)
                continue;

            Cell target = resources[i].pos;

            float travel =
                dist(robot_pos, target);

            float risk =
                compute_cell_risk(
                    occ,
                    target.x,
                    target.y,
                    emergency,
                    time_remaining
                );

            float zone_bonus = target.x >= HIGH_VALUE_ZONE_X ? 2.0f : 1.0f;
            float reward = (float)resources[i].value * zone_bonus;

            float utility =
                reward -
                risk -
                travel;

            if(utility > best.utility)
            {
                best.type = TASK_COLLECT_RESOURCE;

                best.target = target;
                best.resource_index = i;
                best.travel = travel;
                best.reward = reward;
                best.risk = risk;

                best.utility = utility;
            }
        }
    }

    for(int i = 0; i < hole_count; i++)
    {
        if(holes[i].believed_filled)
            continue;

        if(seed_inventory <= 0)
            continue;

        Cell target = {holes[i].cell_a_x, holes[i].cell_a_y};

        float travel =
            dist(robot_pos, target);

        float risk =
            compute_cell_risk(
                occ,
                target.x,
                target.y,
                emergency,
                time_remaining
            ) + ((1.0f - holes[i].belief_confidence) * W_STALE);

        float utility =
            (float)holes[i].value -
            travel -
            risk;

        if(utility > best.utility)
        {
            best.type = TASK_FILL_HOLE;

            best.target = target;
            best.hole_index = i;
            best.resource_index = -1;
            best.travel = travel;
            best.reward = (float)holes[i].value;
            best.risk = risk;

            best.utility = utility;
        }
    }

    return best;
}
