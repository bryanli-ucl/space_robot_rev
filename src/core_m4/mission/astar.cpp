#include "astar.h"
#include "risk.h"
#include "reservation.h"
#include <math.h>
#include <stdbool.h>

#define INF 999999.0f

static float g_cost[GRID_W][GRID_H][4];
static float arrival_time[GRID_W][GRID_H][4];
static bool open_set[GRID_W][GRID_H][4];
static bool closed_set[GRID_W][GRID_H][4];
static Cell parent[GRID_W][GRID_H][4];
static GridHeading parent_heading[GRID_W][GRID_H][4];

static bool valid(int x, int y)
{
    return (x >= 0 && y >= 0 && x < GRID_W && y < GRID_H);
}

float astar_heuristic(Cell a, Cell b)
{
    return ASTAR_WEIGHT *
        (fabsf((float)(a.x - b.x)) + fabsf((float)(a.y - b.y)));
}

typedef struct
{
    Cell cell;
    GridHeading heading;
} AStarState;

static int turn_quarter_steps(
    GridHeading from,
    GridHeading to
)
{
    int clockwise = ((int)to - (int)from + 4) % 4;

    if(clockwise > 2)
    {
        return 4 - clockwise;
    }

    return clockwise;
}

static GridHeading heading_for_delta(int dx, int dy)
{
    if(dx == 1)
        return HEADING_POS_X;

    if(dx == -1)
        return HEADING_NEG_X;

    if(dy == 1)
        return HEADING_POS_Y;

    return HEADING_NEG_Y;
}

static float move_time_cost(
    GridHeading current_heading,
    GridHeading next_heading
)
{
    return ASTAR_MOVE_COST +
        ((float)turn_quarter_steps(current_heading, next_heading) * ASTAR_TURN_90_COST);
}

static AStarState lowest_node(Cell goal)
{
    float best = INF;
    AStarState result = {{-1, -1}, HEADING_POS_Y};

    for (int x = 0; x < GRID_W; x++)
    {
        for (int y = 0; y < GRID_H; y++)
        {
            for(int heading = 0; heading < 4; heading++)
            {
                if (!open_set[x][y][heading])
                    continue;

                Cell c = {x, y};

                float f = g_cost[x][y][heading] + astar_heuristic(c, goal);

                if (f < best)
                {
                    best = f;
                    result.cell = c;
                    result.heading = (GridHeading)heading;
                }
            }
        }
    }

    return result;
}

bool astar_plan_time(
    Cell start,
    GridHeading start_heading,
    Cell goal,
    OccupancyMap* occ,
    Path* out_path,
    ReservationMap* reservations,
    EdgeReservationMap* edges,
    bool emergency,
    float time_remaining
)
{
    // reset
    for (int x = 0; x < GRID_W; x++)
    {
        for (int y = 0; y < GRID_H; y++)
        {
            for(int heading = 0; heading < 4; heading++)
            {
                g_cost[x][y][heading] = INF;
                arrival_time[x][y][heading] = INF;
                open_set[x][y][heading] = false;
                closed_set[x][y][heading] = false;
            }
        }
    }

    g_cost[start.x][start.y][start_heading] = 0.0f;
    arrival_time[start.x][start.y][start_heading] = 0.0f;
    open_set[start.x][start.y][start_heading] = true;

    while (true)
    {
        AStarState current_state = lowest_node(goal);
        Cell current = current_state.cell;
        GridHeading current_heading = current_state.heading;

        if (current.x == -1)
            return false;

        if (current.x == goal.x && current.y == goal.y)
        {
            // ============================
            // PATH RECONSTRUCTION
            // ============================
            out_path->length = 0;

            Cell c = goal;
            GridHeading h = current_heading;

            while (!(c.x == start.x && c.y == start.y))
            {
                int path_index = out_path->length++;

                out_path->cells[path_index] = c;
                out_path->time_indices[path_index] =
                    (int)ceilf(arrival_time[c.x][c.y][h]);

                Cell p = parent[c.x][c.y][h];
                GridHeading ph = parent_heading[c.x][c.y][h];

                c = p;
                h = ph;
            }

            int start_index = out_path->length++;

            out_path->cells[start_index] = start;
            out_path->time_indices[start_index] = 0;

            // ============================
            // RESERVATION WRITING (ADDED)
            // ============================
            if(out_path->length == 1)
            {
                reservation_reserve(
                    reservations,
                    start.x,
                    start.y,
                    0
                );
            }

            for (int i = out_path->length - 1; i > 0; i--)
            {
                Cell from = out_path->cells[i];
                Cell to = out_path->cells[i - 1];
                int depart_time = out_path->time_indices[i];
                int arrive_time = out_path->time_indices[i - 1];

                for(int t = depart_time; t < arrive_time; t++)
                {
                    reservation_reserve(
                        reservations,
                        from.x,
                        from.y,
                        t
                    );
                }

                reservation_reserve(
                    reservations,
                    to.x,
                    to.y,
                    arrive_time
                );
            }

            return true;
        }

        open_set[current.x][current.y][current_heading] = false;
        closed_set[current.x][current.y][current_heading] = true;

        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        for (int i = 0; i < 4; i++)
        {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (!valid(nx, ny))
                continue;

            GridHeading next_heading = heading_for_delta(dx[i], dy[i]);

            if (closed_set[nx][ny][next_heading])
                continue;

            if(!(nx == goal.x && ny == goal.y) && occ->occupancy[nx][ny] > 0.95f)
                continue;

            float risk = compute_cell_risk(
                occ,
                nx,
                ny,
                emergency,
                time_remaining
            );

            float movement_time =
                move_time_cost(current_heading, next_heading);

            float next_arrival_time =
                arrival_time[current.x][current.y][current_heading] +
                movement_time;

            float base =
                g_cost[current.x][current.y][current_heading] +
                movement_time +
                risk;

            int future_time = (int)ceilf(next_arrival_time);

            float future_risk =
                future_collision_probability(
                    reservations,
                    edges,
                    nx,
                    ny,
                    future_time
                );

            float tentative = base + future_risk;

            if (!open_set[nx][ny][next_heading] ||
                tentative < g_cost[nx][ny][next_heading])
            {
                g_cost[nx][ny][next_heading] = tentative;
                arrival_time[nx][ny][next_heading] = next_arrival_time;
                parent[nx][ny][next_heading] = current;
                parent_heading[nx][ny][next_heading] = current_heading;
                open_set[nx][ny][next_heading] = true;
            }
        }
    }

    return false;
}
