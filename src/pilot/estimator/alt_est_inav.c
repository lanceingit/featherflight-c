#include "board.h"

#include "alt_est_inav.h"
#include "mathlib.h"
#include "sensor.h"
#include "debug.h"
#include "param.h"
#include "alti_param.h"


struct alt_est_inav_s alt_est_inav = {
	.heir = {
        .heir = {
            .init = &alt_est_inav_init,
            .run = &alt_est_inav_run,
        },
	},
};

static struct alt_est_inav_s* this=&alt_est_inav;

void alt_est_inav_shell(int argc, char *argv[]);


void alt_est_inav_reset(void)
{
    this->acc_bias = vector_set(0.0f, 0.0f, 0.0f);
    this->acc_bias_corr = vector_set(0.0f, 0.0f, 0.0f);
    this->heir.alt = 0.0f;
    this->heir.vel = 0.0f;
    this->acc_alt = 0.0f;
    this->acc_vel = 0.0f;
    this->bias_inited = false;
}

bool alt_est_inav_init(void)
{
    PARAM_REGISTER(alti);
    cli_regist("alti", alt_est_inav_shell);
    this->w_pos = PARAM_GET(ALTI_W_POS);
    this->w_vel = PARAM_GET(ALTI_W_VEL);
    this->w_bias = PARAM_GET(ALTI_W_BIAS);
    this->w_c2bias = PARAM_GET(ALTI_W_C2BIAS);
    this->w_baro_off = PARAM_GET(ALTI_W_BARO_OFF);
    this->baro_check_vel = PARAM_GET(ALTI_BARO_CHECK_V);
    alt_est_inav_reset();
    this->heir.inited = true;

    return true;
}

bool alt_est_inav_run(float dt)
{
    Vector acc;
    Dcm r;

    imu_get_acc(0, &acc);
    acc = vector_sub(acc, this->acc_bias);
    att_get_dcm(r);
    acc = rotation_ef(r, &acc);

    acc.z += CONSTANTS_ONE_G;
    this->heir.acc_neu_z = -acc.z;

    this->heir.alt += this->heir.vel * dt + acc.z * dt * dt / 2.0f;
    this->heir.vel += + acc.z * dt;

    this->acc_alt += this->acc_vel * dt + acc.z * dt * dt / 2.0f;
    this->acc_vel += acc.z*dt;
    this->acc_vel_bias = (this->acc_vel - acc.z*dt)*this->w_bias;
    this->acc_vel -= this->acc_vel_bias;

    this->baro_offset = (this->acc_alt - baro_get_altitude_smooth(0))*this->w_baro_off;
    if(this->heir.vel < this->baro_check_vel) {
        this->baro_corr = -baro_get_altitude_smooth(0) + this->baro_offset - this->heir.alt;
    } else {
        this->baro_corr = -baro_get_altitude_smooth(0) - this->heir.alt;
    }

    this->acc_bias_corr.x = 0.0f;
    this->acc_bias_corr.y = 0.0f;
    this->acc_bias_corr.z -= this->baro_corr * 0.1f;
    if(!this->bias_inited) {
        this->acc_bias_corr.z = acc.z;
        this->bias_inited = true;
        this->acc_bias = rotation_bf(r, &this->acc_bias_corr);
    } else {
        this->acc_bias = rotation_bf(r, &this->acc_bias_corr);
        this->acc_bias = vector_add(this->acc_bias, vector_mul(this->acc_bias, this->w_bias*dt));
    }

    this->heir.alt += this->baro_corr * this->w_pos;
    this->heir.vel += this->baro_corr * this->w_vel;

    return true;
}

void alt_est_inav_shell(int argc, char *argv[])
{
    
}
