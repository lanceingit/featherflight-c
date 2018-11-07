#include "board.h"
#include "commander.h"
#include "timer.h"
#include "debug.h"
#include <math.h>
#include "param.h"
#include "cmder_param.h"

#define LEFT_STICK_LEFT			(1<<0)
#define LEFT_STICK_H_CENTER	    (1<<1)
#define LEFT_STICK_RIGHT		(1<<2)
#define LEFT_STICK_UP		    (1<<3)
#define LEFT_STICK_V_CENTER	    (1<<4)
#define LEFT_STICK_DOWN			(1<<5)
#define RIGHT_STICK_LEFT		(1<<6)
#define RIGHT_STICK_H_CENTER	(1<<7)
#define RIGHT_STICK_RIGHT		(1<<8)
#define RIGHT_STICK_UP			(1<<9)
#define RIGHT_STICK_V_CENTER	(1<<10)
#define RIGHT_STICK_DOWN		(1<<11)

struct commander_s
{
    bool armed;
    bool flying;
    uint8_t curr_controler;
    struct stick_s stick;
    float rp_gain;
    float yaw_rate_gain;
};


struct commander_s commander = {
    .armed = false,
    .flying = false,
    .curr_controler = 0,
};

struct commander_s* this = &commander;


bool system_armed(void)
{
    return this->armed;
}

float commander_get_roll(void)
{
    return this->stick.roll*this->rp_gain;
}

float commander_get_pitch(void)
{
    return this->stick.pitch*this->rp_gain;

}

float commander_get_yaw(void)
{
    return this->stick.yaw;

}

float commander_get_thrust(void)
{
    return this->stick.thrust;
}

float commander_get_yaw_rate(void)
{
    return this->stick.yaw*this->yaw_rate_gain;
}

void commander_set_roll(uint8_t ch, float v)
{
    if(ch > this->curr_controler) {
        this->stick.roll = v;
    }
}

void commander_set_pitch(uint8_t ch, float v)
{
    if(ch > this->curr_controler) {
        this->stick.pitch = v;
    }    
}

void commander_set_yaw(uint8_t ch, float v)
{
    if(ch > this->curr_controler) {
        this->stick.yaw = v;
    }
}

void commander_set_thrust(uint8_t ch, float v)
{
    if(ch > this->curr_controler) {
        this->stick.thrust = v;
    }
}

uint16_t stick_get_position(struct stick_s* s, float limit, float deadzone)
{
	uint16_t pos=0;

	if(s->yaw<-limit){
		pos |= LEFT_STICK_LEFT;
	} else if (s->yaw>limit) {
		pos |= LEFT_STICK_RIGHT;
	} else if (fabsf(s->yaw>limit)<deadzone) {
		pos |= LEFT_STICK_H_CENTER;
    }

	if(s->thrust>limit){
		pos |= LEFT_STICK_UP;
	} else if (s->thrust<-limit) {
		pos |= LEFT_STICK_DOWN;
	} else if (fabsf(s->thrust>limit)<deadzone) {
		pos |= LEFT_STICK_V_CENTER;
    }

	if(s->roll<-limit){
		pos |= RIGHT_STICK_LEFT;
	} else if (s->roll>limit) {
		pos |= RIGHT_STICK_RIGHT;
	} else if (fabsf(s->roll>limit)<deadzone) {
		pos |= RIGHT_STICK_H_CENTER;
    }

	if(s->pitch<-limit){
		pos |= RIGHT_STICK_UP;
	} else if (s->pitch>limit) {
		pos |= RIGHT_STICK_DOWN;
	} else if (fabsf(s->pitch>limit)<deadzone) {
		pos |= RIGHT_STICK_V_CENTER;
    }
	return pos;
}

bool check_stick_arm(void)
{
#define STICK_LIMIT 0.8f
#define STICK_DEADZONE 0.15f

#define WAIT_ARM_PRESS      0
#define WAIT_ARM_RELEASE    1
#define WAIT_DISARM_PRESS   2
#define WAIT_DISARM_RELEASE 3

    static times_t arm_time;
    static times_t disarm_time;
    static uint8_t status = WAIT_ARM_PRESS;
    bool armed=this->armed;

// PRINT("stick:t:%f y:%f r:%f p:%f\n", this->stick.thrust, this->stick.yaw, this->stick.roll, this->stick.p);

    if(status == WAIT_ARM_PRESS) {
        armed = false;
        if(stick_get_position(&this->stick, STICK_LIMIT, STICK_DEADZONE) == (LEFT_STICK_LEFT|LEFT_STICK_DOWN|
                                                                            RIGHT_STICK_RIGHT|RIGHT_STICK_DOWN)
        || stick_get_position(&this->stick, STICK_LIMIT, STICK_DEADZONE) == (LEFT_STICK_RIGHT|LEFT_STICK_DOWN|
                                                                            RIGHT_STICK_LEFT|RIGHT_STICK_DOWN)                                               
        ) {
            if(timer_check(&arm_time, 1500*1000)) {
                armed = true;
                status = WAIT_ARM_RELEASE;
            } 
        } else {
            arm_time = timer_now();            
        }    
    } else if(status == WAIT_ARM_RELEASE) {
        if(stick_get_position(&this->stick, STICK_LIMIT, STICK_DEADZONE) == (LEFT_STICK_V_CENTER|LEFT_STICK_H_CENTER|
                                                                            RIGHT_STICK_H_CENTER|RIGHT_STICK_V_CENTER)                                              
        ) {  
            status = WAIT_DISARM_PRESS;
        }
        armed = true;   
    } else if(status == WAIT_DISARM_PRESS) {
        armed = true;
        if(stick_get_position(&this->stick, STICK_LIMIT, STICK_DEADZONE) == (LEFT_STICK_LEFT|LEFT_STICK_DOWN|
                                                                            RIGHT_STICK_RIGHT|RIGHT_STICK_DOWN)
        || stick_get_position(&this->stick, STICK_LIMIT, STICK_DEADZONE) == (LEFT_STICK_RIGHT|LEFT_STICK_DOWN|
                                                                            RIGHT_STICK_LEFT|RIGHT_STICK_DOWN)                                               
        || stick_get_position(&this->stick, STICK_LIMIT, STICK_DEADZONE) == (LEFT_STICK_DOWN|LEFT_STICK_H_CENTER|
                                                                            RIGHT_STICK_H_CENTER|RIGHT_STICK_V_CENTER)                                               
        ) {
            if(timer_check(&disarm_time, 500*1000)) {
                armed = false;
                status = WAIT_DISARM_RELEASE;
            }
        } else {
            disarm_time = timer_now();
        }         
    } else if(status == WAIT_DISARM_RELEASE) {
        if(stick_get_position(&this->stick, STICK_LIMIT, STICK_DEADZONE) == (LEFT_STICK_V_CENTER|LEFT_STICK_H_CENTER|
                                                                            RIGHT_STICK_H_CENTER|RIGHT_STICK_V_CENTER)                                              
        ) {  
            status = WAIT_ARM_PRESS;
        }        
        armed = false;
    }
    return armed;
}

void commander_init(void)
{
    PARAM_REGISTER(cmder);
    this->rp_gain = PARAM_GET(CMDER_RP_GAIN);
    this->yaw_rate_gain = PARAM_GET(CMDER_YAW_RATE_GAIN);
}

void commander_update(void)
{
    bool armed;
    if(!this->flying) {
        armed = check_stick_arm();
        if(armed != this->armed) {
            PRINT("armd change %d->%d\n", this->armed, armed);
            this->armed = armed;
        }    
    }




}
