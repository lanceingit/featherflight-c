#pragma once

enum nav_mode
{
    STABILIZE,
    ALTHOLD,
    POSHOLD,
    TAKEOFF,
    LAND,      
    STOP,
};

bool navigator_set_mode(enum nav_mode mode);
void navigator_update(void);

#include "stabilize.h"

