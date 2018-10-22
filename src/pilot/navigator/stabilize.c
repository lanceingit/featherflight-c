#include "commander.h"
#include "att_control.h"
#include "mixer.h"

void stabilize_update(float dt)
{
    att_set_roll_target(commander_get_roll());
    att_set_pitch_target(commander_get_pitch());
    att_set_yaw_target(commander_get_yaw());
    mixer_set_thrust(commander_get_thrust());
}
