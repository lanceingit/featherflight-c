#include "board.h"
#include <math.h>

#include "sensor.h"
#include "alt_est_3o.h"
#include "debug.h"
#include "param.h"
#include "mathlib.h"
#include "alt3o_param.h"


struct alt_est_3o_s alt_est_3o = {
	.heir = {
        .heir = {
            .init = &alt_est_3o_init,
            .run = &alt_est_3o_run,
        },
	},
};

static struct alt_est_3o_s* this=&alt_est_3o;

void alt_est_3o_shell(int argc, char *argv[]);


void alt_est_3o_reset(void)
{
    this->heir.alt = 0.0f;
    this->heir.vel = 0.0f;
    this->acc_corr = 0.0f;
    this->vel_corr = 0.0f;
    this->alt_corr = 0.0f;    
    this->pos_base = 0.0f;
}

bool alt_est_3o_init(void)
{
    PARAM_REGISTER(alt3o);
    cli_regist("alt3o", alt_est_3o_shell);
    float time_constant = PARAM_GET(ALT3O_TIME_CONS);
    this->k1 = 3.0f / time_constant;
    this->k2 = 3.0f / (time_constant*time_constant);
    this->k3 = 1.0f / (time_constant*time_constant*time_constant);
    PRINT("k1:%f k2:%f k3:%f\n", this->k1, this->k2, this->k3);
    alt_est_3o_reset();
    this->heir.inited = true;

    return true;
}

bool alt_est_3o_run(float dt)
{
    Vector acc;
    Dcm r;

    imu_get_acc(0, &acc);
    att_get_dcm(r);
    acc = rotation_ef(r, &acc);

    acc.z += CONSTANTS_ONE_G;

    this->heir.acc_neu_z = -acc.z;

    this->alt_err = baro_get_altitude(0) - (this->heir.alt + this->heir.ref_alt);

    this->acc_corr += this->alt_err * this->k3 * dt;
    this->vel_corr += this->alt_err * this->k2 * dt;
    this->alt_corr += this->alt_err * this->k1 * dt;

    float vel_inc = (this->heir.acc_neu_z + this->acc_corr) * dt;
    this->heir.vel = this->vel_corr;
    this->pos_base += (this->heir.vel + vel_inc*0.5f) * dt;
    this->heir.alt = this->pos_base + this->alt_corr;
    this->heir.vel += vel_inc;

    this->heir.valid = true;

    return true;
}

void alt_est_3o_shell(int argc, char *argv[])
{
	if(argc == 2) {
		if(strcmp(argv[1],"reset") == 0) {
			alt_est_3o_reset();
			return;
		}
	} else if(argc == 3) {
 		if(strcmp(argv[1],"t") == 0) {
            float time_constant = atof(argv[2]);
            this->k1 = 3.0f / time_constant;
            this->k2 = 3.0f / (time_constant*time_constant);
            this->k3 = 1.0f / (time_constant*time_constant*time_constant);
			alt_est_3o_reset();
			return;
		}       
	} else if(argc == 5) {
		if(strcmp(argv[1],"w") == 0) {
			this->k1 = atof(argv[2]);
			this->k1 = atof(argv[3]);
			this->k1 = atof(argv[4]);
			alt_est_3o_reset();
			return;
		}
	}

	cli_device_write("missing command: try 'reset' 't tc' 'w k1 k2 k3'");
}
