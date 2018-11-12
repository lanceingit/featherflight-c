#pragma once

#include "mathlib.h"
#include "sensor.h"
#include "func_type.h"
#include "timer.h"
#include "commander.h"


struct est_s
{
    //method
    init_func* init;
    run_func* run;    
	times_t last_time;
};

#define SPIN_RATE_LIMIT    0.175f

struct att_est_s 
{
    //member
    struct est_s heir;

    bool inited;
    bool valid;

    bool use_compass;
	float dt_max;

    Vector acc;
    Vector gyro;
    Vector mag;

    float roll;     //DEG
    float pitch;    //DEG
    float yaw;      //DEG

    float roll_rate;    //DEG/S
    float pitch_rate;   //DEG/S
    float yaw_rate;     //DEG/S

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

    float acc_neu_z;

	float epv;
	bool valid;

    float terrain_offset;

	float ref_alt;
    bool ref_inited;
    set_val_func* set_scene;
    enum alt_scene_e scene;
};

void att_est_register(struct att_est_s* est);
float att_get_roll(void);
float att_get_pitch(void);
float att_get_yaw(void);
float att_get_roll_rate(void);
float att_get_pitch_rate(void);
float att_get_yaw_rate(void);
bool att_valid(void);
void att_get_dcm(Dcm r);

void alt_est_register(struct alt_est_s* est);
float alt_est_get_alt(void);
float alt_est_get_vel(void);
float alt_est_get_ref_alt(void);
float alt_est_get_terrain_alt(void);
float alt_est_get_terrain_offset(void);
float alt_est_get_takeoff_alt(void);

bool pos_est_valid(void);

void est_init(void);
void est_att_run(void);
void est_alt_run(void);

#include "att_est_q.h"
#include "att_est_cf.h"
#include "alt_est_3o.h"
#include "alt_est_inav.h"
