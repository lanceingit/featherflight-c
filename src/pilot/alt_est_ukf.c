#include "board.h"

#include "alt_est_ukf.h"
#include "mathlib.h"
#include "sensor.h"
#include "timer.h"
#include <math.h>

#define ALT_S           3   // states
#define ALT_M           2   // measurements
#define ALT_V           2   // process noise
#define ALT_N           2   // measurement noise

#define ALT_STATE_POS   0
#define ALT_STATE_VEL   1
#define ALT_STATE_BIAS  2

#define ALT_NOISE_BIAS  0
#define ALT_NOISE_VEL   1

#define ALT_POS_Q_NOISE  5.0f
#define ALT_VEL_Q_NOISE  1e-6f
#define ALT_ACC_Q_NOISE  0.05f

#define ALT_POS_M_NOISE  0.02f
#define ALT_VEL_M_NOISE  0.001f

#define ALT_BIAS_P_NOISE  5e-4f//5e-5f
#define ALT_VEL_P_NOISE   5e-4f

struct alt_est_ukf_s alt_est_ukf = {
	.heir = {
        .heir = {
            .init = &alt_est_ukf_init,
            .run = &alt_est_ukf_run,
        },
	},
};

static struct alt_est_ukf_s* this=&alt_est_ukf;

bool alt_est_ukf_init(void)
{
    float Q[ALT_S];		// state variance
    float V[ALT_V];		// process variance

    this->kf = srcdkfInit(ALT_S, ALT_M, ALT_V, ALT_N, altUkfTimeUpdate);

    this->x = srcdkfGetState(altUkfData.kf);

    Q[ALT_STATE_POS] = ALT_POS_Q_NOISE;
    Q[ALT_STATE_VEL] = ALT_VEL_Q_NOISE;
    Q[ALT_STATE_BIAS] = ALT_ACC_Q_NOISE;

    V[ALT_NOISE_BIAS] = ALT_BIAS_P_NOISE;
    V[ALT_NOISE_VEL] = ALT_VEL_P_NOISE;

    srcdkfSetVariance(altUkfData.kf, Q, V, 0, 0);

    ALT_POS = baro_get_alt();
    ALT_VEL = 0.0f;
    ALT_BIAS = 0.0f;    


    this->heir.inited = true;

    return true;
}

bool alt_est_ukf_run(float dt)
{

}