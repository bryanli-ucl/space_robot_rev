#include "../termthree_navigation/movement.h"
#include "../termthree_navigation/termthree_navigation.h"

#include "logger.hpp"
#include "mission.hpp"
#include "shell.hpp"
#include "state.hpp"

#include <stdlib.h>
#include <string.h>

static bool ensure_robot_started(const char* command)
{
    if(running_state != RunningState::STOPPED)
    {
        return true;
    }

    loggf("%s aborted: robot is stopped. use start first.\n", command);
    return false;
}

static void print_nav_status()
{
    const Robot* robot = termthree_navigation_robot();

    if(robot == nullptr)
    {
        loggf("nav status unavailable\n");
        return;
    }

    loggf(
        "nav enabled=%d pos=%d,%d heading=%d state=%d task=%d target=%d,%d score=%.1f seeds=%d\n",
        termthree_navigation_enabled() ? 1 : 0,
        robot->pos.x,
        robot->pos.y,
        static_cast<int>(robot->heading),
        static_cast<int>(robot->state),
        static_cast<int>(robot->task.type),
        robot->task.target.x,
        robot->task.target.y,
        robot->score,
        robot->seed_inventory
    );
}

static bool run_step_token(const char* token)
{
    if((token[0] == 'f' || strncmp(token, "forward", 7) == 0) && token[1] != '\0')
    {
        const char* count_text = token[0] == 'f' ? token + 1 : token + 7;
        int count = atoi(count_text);
        if(count <= 0)
        {
            loggf("nav test invalid forward cell count '%s'\n", token);
            return false;
        }

        loggf("nav test step forward_rfid cells=%d\n", count);
        return move_forward_rfid_cells(count);
    }

    if(strcmp(token, "f") == 0 || strcmp(token, "forward") == 0)
    {
        loggf("nav test step forward_rfid cells=1\n");
        return move_forward_rfid_cells(1);
    }

    if(strcmp(token, "b") == 0 || strcmp(token, "back") == 0 || strcmp(token, "backward") == 0)
    {
        loggf("nav test step backward\n");
        move_backward_one_cell();
        return true;
    }

    if(strcmp(token, "l") == 0 || strcmp(token, "left") == 0)
    {
        loggf("nav test step left\n");
        turn_left_90();
        return true;
    }

    if(strcmp(token, "r") == 0 || strcmp(token, "right") == 0)
    {
        loggf("nav test step right\n");
        turn_right_90();
        return true;
    }

    if(strcmp(token, "sl") == 0 || strcmp(token, "strafe_left") == 0)
    {
        loggf("nav test step strafe_left\n");
        strafe_left_one_cell();
        return true;
    }

    if(strcmp(token, "sr") == 0 || strcmp(token, "strafe_right") == 0)
    {
        loggf("nav test step strafe_right\n");
        strafe_right_one_cell();
        return true;
    }

    if(strcmp(token, "seed") == 0 || strcmp(token, "drop") == 0)
    {
        loggf("nav test step drop_seed\n");
        drop_seed();
        return true;
    }

    loggf("nav test unknown step '%s'\n", token);
    return false;
}

static void nav_cmd(int argc, char** argv)
{
    if(argc == 1 || (argc == 2 && strcmp(argv[1], "status") == 0))
    {
        print_nav_status();
        return;
    }

    if(argc == 2 && strcmp(argv[1], "start") == 0)
    {
        if(!ensure_robot_started("nav start"))
        {
            return;
        }

        termthree_navigation_set_enabled(true);
        loggf("nav enabled\n");
        print_nav_status();
        return;
    }

    if(argc == 2 && strcmp(argv[1], "stop") == 0)
    {
        termthree_navigation_set_enabled(false);
        movement_stop_all();
        loggf("nav disabled\n");
        print_nav_status();
        return;
    }

    if(argc == 2 && strcmp(argv[1], "tick") == 0)
    {
        if(!ensure_robot_started("nav tick"))
        {
            return;
        }

        termthree_navigation_tick();
        print_nav_status();
        return;
    }

    if(argc >= 3 && (strcmp(argv[1], "test") == 0 || strcmp(argv[1], "step") == 0))
    {
        if(!ensure_robot_started("nav test"))
        {
            return;
        }

        mission_stop();
        termthree_navigation_set_enabled(false);

        for(int i = 2; i < argc; i++)
        {
            if(!run_step_token(argv[i]))
            {
                movement_stop_all();
                loggf("usage: nav test <f|f2|f3|b|l|r|sl|sr|seed> [...]\n");
                return;
            }

            if(running_state == RunningState::STOPPED)
            {
                movement_stop_all();
                loggf("nav test stopped by state\n");
                return;
            }
        }

        movement_stop_all();
        loggf("nav test done\n");
        return;
    }

    if(argc == 3 && strcmp(argv[1], "rfid") == 0)
    {
        if(!ensure_robot_started("nav rfid"))
        {
            return;
        }

        const int count = atoi(argv[2]);
        if(count <= 0)
        {
            loggf("usage: nav rfid <cell_count>\n");
            return;
        }

        mission_stop();
        termthree_navigation_set_enabled(false);

        if(move_forward_rfid_cells(count))
        {
            loggf("nav rfid done cells=%d\n", count);
        }
        else
        {
            movement_stop_all();
            loggf("nav rfid failed cells=%d\n", count);
        }

        return;
    }

    loggf("usage: nav status | nav start | nav stop | nav tick | nav rfid <n> | nav test <f|f2|b|l|r|sl|sr|seed> [...]\n");
}

SHELL_COMMAND("nav", nav_cmd, "term-three navigation and movement tests")
