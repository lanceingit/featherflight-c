#include "commander.h"
#include "att_control.h"
#include "alt_control.h"
#include "mixer.h"

void althold_update(float dt)
{
    att_control_roll_pitch_update(dt, commander_get_roll(), commander_get_pitch(), 1.0f);
    att_control_yaw_rate_update(dt, commander_get_yaw_rate());
    alt_control_update(dt, commander_get_thrust());
}
