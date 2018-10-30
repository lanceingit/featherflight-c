#include "board.h"
#include "navigator.h"
#include "timer.h"

typedef void(*nav_update_func)(float dt);

struct navigator_s
{
    enum nav_mode curr_mode;
    times_t last_update_time;
    nav_update_func nav_update;
};

struct navigator_s navigator = {
    .curr_mode = STOP,
    .nav_update = NULL,
};

static struct navigator_s* this = &navigator;

bool navigator_set_mode(enum nav_mode mode)
{
    bool ret = false;

    if (mode == this->curr_mode) {
        return true;
    }

    switch(mode) {
        case STABILIZE:
            if((ret=stabilize_init())) {
                this->nav_update = stabilize_update;
            }
            break;
        case ALTHOLD:
            break;
        case POSHOLD:
            break;
        case TAKEOFF:
            break;
        case LAND:      
            break;
        case STOP:        
            break;
        default:break;
    }

    if (ret == true) {
        this->curr_mode = mode;
    }

    return ret;    
}

void navigator_update(void)
{
    float dt = timer_get_dt(&this->last_update_time, 0.02f, 0.00001f);
    if(this->nav_update != NULL) {
        this->nav_update(dt);
    }
}

