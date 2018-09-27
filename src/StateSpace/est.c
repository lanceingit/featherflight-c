#include "est.h"

static struct att_est_s* est_att;

void att_est_register(struct att_est_s* att)
{
    est_att = att;
}

float att_get_roll(void)
{
    return est_att->roll;
}

float att_get_pitch(void)
{
    return est_att->pitch;
}

float att_get_yaw(void)
{
    return est_att->yaw;
}

void est_init(void)
{
    est_att->heir.init();   
}

void est_att_run(void)
{
    est_att->heir.run();
}
