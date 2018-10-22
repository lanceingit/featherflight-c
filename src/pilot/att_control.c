#include "pid.h"
#include "est.h"
#include "mixer.h"
#include "mathlib.h"
#include "param.h"
#include "attc_param.h"


struct pid_s att_pid_roll;
struct pid_s att_pid_pitch;
struct pid_s att_pid_yaw;

struct pid_s rate_pid_roll;
struct pid_s rate_pid_pitch;
struct pid_s rate_pid_yaw;

static float att_roll_target;
static float att_pitch_target;
static float att_yaw_target;
static float rate_roll_limit;
static float rate_pitch_limit;

void att_set_roll_target(float t)
{
    att_roll_target = t;
}

void att_set_pitch_target(float t)
{
    att_pitch_target = t;
}

void att_set_yaw_target(float t)
{
    att_yaw_target = t;
}

void att_set_rate_roll_limit(float l)
{
    rate_roll_limit = l;
}

void att_set_rate_pitch_limit(float l)
{
    rate_pitch_limit = l;
}

void att_control_init(void)
{
    PARAM_REGISTER(attc)
	pid_init(&att_pid_roll, PARAM_GET(ATTC_ATT_ROLL_P), 0, 0, 0, PARAM_GET(ATTC_ATT_RP_OUT_LIMIT), 0);
	pid_init(&att_pid_pitch, PARAM_GET(ATTC_ATT_PITCH_P), 0, 0, 0, PARAM_GET(ATTC_ATT_RP_OUT_LIMIT), 0);
	pid_init(&att_pid_yaw, PARAM_GET(ATTC_ATT_YAW_P), 0, 0, 0, PARAM_GET(ATTC_ATT_YAW_OUT_LIMIT), 0);
	
	pid_init(&rate_pid_roll, PARAM_GET(ATTC_RATE_ROLL_P),
                                PARAM_GET(ATTC_RATE_ROLL_I),
                                PARAM_GET(ATTC_RATE_ROLL_I),
                                PARAM_GET(ATTC_RATE_RP_I_LIMIT),
                                PARAM_GET(ATTC_RATE_RP_OUT_LIMIT),
                                PARAM_GET(ATTC_RATE_RP_D_WEIGHT));
	pid_init(&rate_pid_pitch, PARAM_GET(ATTC_RATE_PITCH_P),
                                PARAM_GET(ATTC_RATE_PITCH_I),
                                PARAM_GET(ATTC_RATE_PITCH_I),
                                PARAM_GET(ATTC_RATE_RP_I_LIMIT),
                                PARAM_GET(ATTC_RATE_RP_OUT_LIMIT),
                                PARAM_GET(ATTC_RATE_RP_D_WEIGHT));
	pid_init(&rate_pid_yaw, PARAM_GET(ATTC_RATE_YAW_P),
                                PARAM_GET(ATTC_RATE_YAW_I),
                                PARAM_GET(ATTC_RATE_YAW_I),
                                PARAM_GET(ATTC_RATE_YAW_I_LIMIT),
                                PARAM_GET(ATTC_RATE_YAW_OUT_LIMIT),
                                PARAM_GET(ATTC_RATE_YAW_D_WEIGHT));                                                                
}

void att_control_update(float dt)
{
	float rate_taret_roll;
	float rate_taret_pitch;
	float rate_taret_yaw;
	float roll_output;
	float pitch_output;    
	float yaw_output;    

	att_roll_target = constrain(att_roll_target, -PARAM_GET(ATTC_RP_LIMIT), PARAM_GET(ATTC_RP_LIMIT));
	att_pitch_target = constrain(att_pitch_target, -PARAM_GET(ATTC_RP_LIMIT), PARAM_GET(ATTC_RP_LIMIT));

    rate_taret_roll  = pid_update(&att_pid_roll, att_roll_target-att_get_roll(), dt);
    rate_taret_pitch = pid_update(&att_pid_pitch, att_pitch_target-att_get_pitch(), dt);
    rate_taret_yaw   = pid_update(&att_pid_yaw, att_yaw_target-att_get_yaw(), dt);

	rate_taret_roll = constrain(rate_taret_roll, -rate_roll_limit, rate_roll_limit);
	rate_taret_pitch = constrain(rate_taret_pitch, -rate_pitch_limit, rate_pitch_limit);

	roll_output = pid_update(&rate_pid_roll, rate_taret_roll-att_get_roll_rate(), dt);
	pitch_output = pid_update(&rate_pid_pitch, rate_taret_pitch-att_get_pitch_rate(), dt);
	yaw_output = pid_update(&rate_pid_yaw, rate_taret_yaw-att_get_yaw_rate(), dt);

	mixer_set_roll(roll_output);
	mixer_set_pitch(pitch_output);    
	mixer_set_yaw(yaw_output);    
}
