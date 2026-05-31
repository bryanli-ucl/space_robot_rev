#include "robot.h"
#include "tasks.h"
#include "movement.h"
#include "risk.h"

static GridHeading heading_for_step(
    Cell a,
    Cell b
)
{
    int dx = b.x - a.x;
    int dy = b.y - a.y;

    if(dx == 1)
        return HEADING_POS_X;

    if(dx == -1)
        return HEADING_NEG_X;

    if(dy == 1)
        return HEADING_POS_Y;

    return HEADING_NEG_Y;
}

static void complete_task(
    Robot* r,
    Hole* holes,
    int hole_count,
    DisabledRobot* disabled,
    int disabled_count,
    Resource* resources,
    int resource_count
)
{
    if(r->task.type == TASK_FILL_HOLE)
    {
        if(r->task.hole_index >= 0 && r->task.hole_index < hole_count)
        {
            if(!holes[r->task.hole_index].filled)
            {
                align_to_hole_midpoint();
                drop_seed();

                holes[r->task.hole_index].filled = true;

                if(r->seed_inventory > 0)
                {
                    r->seed_inventory--;
                }

                r->score += r->task.reward;
                r->seed_planted_event = true;
                r->seed_planted_hole_index = r->task.hole_index;
            }

            holes[r->task.hole_index].believed_filled = true;
            holes[r->task.hole_index].belief_confidence = 1.0f;
        }
    }
    else if(r->task.type == TASK_RESCUE)
    {
        for(int i = 0; i < disabled_count; i++)
        {
            if(disabled[i].rescued)
                continue;

            if(disabled[i].pos.x == r->pos.x && disabled[i].pos.y == r->pos.y)
            {
                mark_rescued(&disabled[i]);
                r->score += r->task.reward;
                break;
            }
        }
    }
    else if(r->task.type == TASK_COLLECT_RESOURCE)
    {
        if(r->task.resource_index >= 0 && r->task.resource_index < resource_count)
        {
            Resource* resource = &resources[r->task.resource_index];

            if(resource->active && resource->pos.x == r->pos.x && resource->pos.y == r->pos.y)
            {
                resource->active = false;

                if(r->seed_inventory < SEED_CAPACITY)
                {
                    r->seed_inventory++;
                }

                r->score += r->task.reward;
            }
        }
    }
    else if(r->task.type == TASK_REFILL || r->task.type == TASK_EVACUATE)
    {
        if(r->pos.x == ENTRY_X || r->pos.x == EXIT_X)
        {
            r->seed_inventory = SEED_CAPACITY;
            r->refill_ticks = REFILL_DELAY_TICKS;
        }
    }

    r->task.type = TASK_WAIT;
    r->task.target = r->pos;
    r->path.length = 0;
    r->path_index = -1;
    r->state = ROBOT_IDLE;
}

void robot_init(
    Robot* r,
    int id,
    int x,
    int y
)
{
    r->id = id;

    r->pos.x = x;
    r->pos.y = y;
    r->heading = HEADING_POS_Y;

    r->state = ROBOT_IDLE;

    r->path.length = 0;

    r->path_index = -1;

    r->seed_inventory = SEED_CAPACITY;
    r->refill_ticks = 0;

    r->score = 0.0f;

    r->task.type = TASK_WAIT;
    r->task.target = r->pos;

    r->can_rescue = true;
    r->seed_planted_event = false;
    r->seed_planted_hole_index = -1;
}

void robot_update(
    Robot* r,
    OccupancyMap* occ,
    Hole* holes,
    int hole_count,
    bool emergency,
    float time_remaining,
    DisabledRobot* disabled,
    int disabled_count,
    ReservationMap* reservations,
    EdgeReservationMap* edges,
    Resource* resources,
    int resource_count
)
{
    if(r->refill_ticks > 0)
    {
        r->refill_ticks--;
        r->state = ROBOT_REFILLING;
        return;
    }

    
    occupancy_observe(
        occ,
        r->pos.x,
        r->pos.y,
        false
    );

    movement_observe_adjacent_cells(
        occ,
        r->pos,
        r->heading
    );
if(emergency)
{
    if(r->seed_inventory > 0)
    {
        float evac_risk =
            compute_cell_risk(
                occ,
                r->pos.x,
                r->pos.y,
                true,
                time_remaining
            );

        if(evac_risk > 12.0f)
        {
        r->state = ROBOT_EVACUATING;
        r->path.length = 0;
        r->path_index = -1;
        }
    }
}

    Task candidate = choose_best_task(
        r->pos,
        holes,
        hole_count,
        occ,
        emergency,
        time_remaining,
        disabled,
        disabled_count,
        resources,
        resource_count,
        r->seed_inventory,
        r->refill_ticks
    );

    if(r->path.length == 0 ||
       candidate.type == TASK_EVACUATE ||
       candidate.utility > r->task.utility + 10.0f)
    {
        r->task = candidate;

        if(r->task.type == TASK_WAIT)
        {
            r->state = ROBOT_WAITING;
            return;
        }

        bool planned = astar_plan_time(
            r->pos,
            r->heading,
            r->task.target,
            occ,
            &r->path,
            reservations,
            edges,
            emergency,
            time_remaining
        );

        if(!planned)
        {
            r->task.type = TASK_WAIT;
            r->task.target = r->pos;
            r->path.length = 0;
            r->path_index = -1;
            r->state = ROBOT_WAITING;
            return;
        }

        r->path_index = r->path.length - 2;
        r->state = ROBOT_MOVING;
    }

    if(r->path_index >= 0 && r->path_index < r->path.length)
    {
        Cell next = r->path.cells[r->path_index];

        GridHeading desired_heading = heading_for_step(r->pos, next);
        execute_move_to_heading(&r->heading, desired_heading);

        r->pos = next;
        r->path_index--;
    }

    if(r->path_index < 0)
    {
        complete_task(
            r,
            holes,
            hole_count,
            disabled,
            disabled_count,
            resources,
            resource_count
        );
    }
}
