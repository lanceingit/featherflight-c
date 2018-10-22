#pragma once

void att_control_init(void);
void att_control_update(float dt);
void att_set_roll_target(float t);
void att_set_pitch_target(float t);
void att_set_yaw_rate_target(float t);

