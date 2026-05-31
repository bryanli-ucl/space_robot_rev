#ifndef TERMTHREE_NAVIGATION_H
#define TERMTHREE_NAVIGATION_H

#include "robot.h"

#ifdef __cplusplus
extern "C" {
#endif

void termthree_navigation_begin(void);
void termthree_navigation_tick(void);
void termthree_navigation_set_enabled(bool enabled);
bool termthree_navigation_enabled(void);
const Robot* termthree_navigation_robot(void);

#ifdef __cplusplus
}
#endif

#endif
