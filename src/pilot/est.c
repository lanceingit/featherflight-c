#include "board.h"

#include "est.h"
#include "timer.h"
#include "debug.h"
#include "trigger.h"

static struct att_est_s* est_att=NULL;
static struct alt_est_s* est_alt=NULL;

void att_est_register(struct att_est_s* est)
{
    est_att = est;
}

void att_init(void)
{
	if(est_att == NULL) {
		PRINT("no est att register\n");
		return;
	}
    est_att->use_compass = false;
	est_att->heir.last_time = 0;
	est_att->inited = false;    
    
    est_att->heir.init();   	
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

float att_get_roll_rate(void)
{
    return est_att->roll_rate;
}

float att_get_pitch_rate(void)
{
    return est_att->pitch_rate;
}

float att_get_yaw_rate(void)
{
    return est_att->yaw_rate;
}

bool att_valid(void)
{
	return est_att->valid;  
}

void att_get_dcm(Dcm r)
{
	quaternion_to_dcm(est_att->q, r);
}

void alt_init(void)
{
	if(est_alt == NULL) {
		PRINT("no est alt register\n");
		return;
	}

    est_alt->inited = false;
	est_alt->alt = 0.0f;		
	est_alt->vel = 0.0f;
	est_alt->valid = false;
	est_alt->ref_alt = 0.0f;
    est_alt->ref_inited = false;
	est_alt->heir.init();
}

void alt_est_register(struct alt_est_s* est)
{
	est_alt = est;
}

float alt_est_get_alt(void)
{
	return est_alt->alt;
}

float alt_est_get_vel(void)
{
	return est_alt->vel;
}

float alt_est_get_ref_alt(void)
{
	return est_alt->ref_alt;
}

float alt_est_get_terrain_alt(void)
{
	return est_alt->alt - est_alt->terrain_offset;
}

float alt_est_get_terrain_offset(void)
{
	return est_alt->terrain_offset;
}

void est_init(void)
{
	att_init();
	alt_init();
}

void est_att_run(void)
{
	if(imu_ready(0)) {
		est_att->gyro.x = imu_get_gyro_x(0);
		est_att->gyro.y = imu_get_gyro_y(0);
		est_att->gyro.z = imu_get_gyro_z(0);

		est_att->acc.x = imu_get_acc_x(0);
		est_att->acc.y = imu_get_acc_y(0);
		est_att->acc.z = imu_get_acc_z(0);

		if (vector_length(est_att->acc) < 0.01f) {
			PRINT("WARNING: degenerate accel!\n");
			return;
		}
	} else {
		return;
	}

	if(est_att->use_compass) {
        compass_get_mag(0, &est_att->mag);

		if (vector_length(est_att->mag) < 0.01f) {
			PRINT("WARNING: degenerate mag!\n");
			return;
		}
    }

	float dt = timer_get_dt(&est_att->heir.last_time, 0.02f, 0.00001f);

	if (!est_att->heir.run(dt)) {
		return;
	}

    Vector euler = quaternion_to_euler(est_att->q);  

    est_att->roll_rate =  (imu_get_gyro_x(0) + est_att->gyro_bias.x)*M_RAD_TO_DEG;
    est_att->pitch_rate = (imu_get_gyro_y(0) + est_att->gyro_bias.y)*M_RAD_TO_DEG;
    est_att->yaw_rate =   (imu_get_gyro_z(0) + est_att->gyro_bias.z)*M_RAD_TO_DEG;

    est_att->roll = euler.x*M_RAD_TO_DEG;
    est_att->pitch = euler.y*M_RAD_TO_DEG;
    est_att->yaw = euler.z*M_RAD_TO_DEG;    
}


void est_alt_run(void)
{
	#define BARO_CAL_MAX  500
	static uint16_t baro_read_cnt=0;

	if(!est_alt->ref_inited) {
		if(baro_read_cnt < BARO_CAL_MAX) {
			est_alt->ref_alt += baro_get_altitude(0); 
			baro_read_cnt++;
		} else {
			est_alt->ref_alt /= BARO_CAL_MAX;
			est_alt->ref_inited = true; 
		}
		return;
	}

	float dt = timer_get_dt(&est_alt->heir.last_time, 0.02f, 0.001f);

	TRIGGER_DEF(arm_trigger)
	uint8_t arm_status = trigger_check(&arm_trigger, system_armed()); 
	if(arm_status == TRIGGER_0_TO_1) {
		est_alt->terrain_offset = est_alt->alt;
		//在螺旋桨转起来前，设置气压计权重为0，关闭气压计融合。
		est_alt->set_scene(ALT_PRE_TAKEOFF);
	} else if(arm_status == TRIGGER_1_TO_0) {
		est_alt->terrain_offset = est_alt->alt;
	}

	if(baro_get_altitude_smooth(0) > est_alt->terrain_offset && commader_get_alt_action() == ALT_TAKEOFF) {
		//当气压计高度大于起飞前高度，设置为起飞模式
		//pos，vel权重增加，得到一个准确值。同时减小bias权重，因为这时修正不准。
		est_alt->set_scene(ALT_TAKEOFF);
	} else {
		est_alt->set_scene(ALT_NORMAL);
	} 

	est_alt->heir.run(dt);

}
