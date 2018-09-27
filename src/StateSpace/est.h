#pragma once

#include "quaternion.h"
#include "vector.h"
#include "func_type.h"


struct est_s
{
    //method
    init_func* init;
    run_func* run;    

};


struct att_est_s 
{
    //member
    struct est_s heir;

    bool inited;

    float roll;
    float pitch;
    float yaw;

    float roll_rate;
    float pitch_rate;
    float yaw_rate;    

    Quaternion	q;
    Vector gyro_bias;
};

struct pos_est_s 
{
    //member
    float x;		//N
	float y;		//E
	float z;		//D
	float vx;
	float vy;
	float vz;

	float eph;
	float epv;
	bool xy_valid;
	bool z_valid;

	double lat;
	double lon;
	float alt;
	float terrain_alt;

	double ref_lat;
	double ref_lon;
	float ref_alt;

    struct est_s est;
};


void att_est_register(struct att_est_s* att);
float att_get_roll(void);
float att_get_pitch(void);
float att_get_yaw(void);

void est_init(void);
void est_att_run(void);

