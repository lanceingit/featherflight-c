#pragma once

#include "quaternion.h"
#include "vector.h"
#include "dcm.h"
#include "func_type.h"


struct est_s
{
    //method
    init_func* init;
    run_func* run;    
};

#define SPIN_RATE_LIMIT    0.175f

struct att_est_s 
{
    //member
    struct est_s heir;

    bool inited;
    bool valid;

    bool use_compass;
	uint64_t last_time;
	float dt_max;

    Vector acc;
    Vector gyro;
    Vector mag;

    float roll;
    float pitch;
    float yaw;

    float roll_rate;
    float pitch_rate;
    float yaw_rate;    

    Quaternion	q;
    Vector gyro_bias;
	Vector corr;
    float spin_rate;
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

struct alt_est_s 
{
    //member
    struct est_s heir;

    bool inited;
	float alt;		//U
	float vel;

    float acc_ned_z;
    float bias;

	float epv;
	bool valid;

	float terrain_alt;
};

void att_est_register(struct att_est_s* att);
float att_get_roll(void);
float att_get_pitch(void);
float att_get_yaw(void);
float att_get_roll_rate(void);
float att_get_pitch_rate(void);
float att_get_yaw_rate(void);
bool att_valid(void);
void att_get_dcm(Dcm r);

void alt_est_register(struct alt_est_s* att);
float alt_est_get_alt(void);
float alt_est_get_vel(void);

void est_init(void);
void est_att_run(void);

