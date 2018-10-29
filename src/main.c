#include "board.h"

#include "sensor.h"
#ifdef F3_EVO
#include "spi_flash.h"
#include "mtd.h"
#endif
#include "log.h"
#include "link_mavlink.h"
#include "link_wwlink.h"
#include "mavlink_log.h"
#include "timer.h"
#include "perf.h"
#include "vector.h"
#include "est.h"
#include "scheduler.h"
#include "mathlib.h"
#include "commander.h"
#include "cli.h"
#include "debug.h"

struct perf_s main_perf;
struct perf_s att_perf;
struct perf_s att_elapsed;

struct task_s link_task;
struct task_s cli_task;
struct task_s imu_task;
struct task_s compass_task;
struct task_s baro_task;
struct task_s att_task;
struct task_s commander_task;

struct variance_s baro_variance;
float baro_vari;
float baro_vel;


void task_link_send(void)
{
    mavlink_stream();
    wwlink_stream();
}

void task_cli(void)
{
    cli_updata();
}

void task_link_recv(void)
{
	mavlink_message_t msg;
    wwlink_message_t wwmsg;

	if(mavlink_recv(&msg))
	{
        mavlink_log_handle(&msg);
	}

    wwlink_recv(&wwmsg);    
}

void task_imu(void)
{        
    imu_update(0);
}    

void task_compass(void)
{
    compass_update(0);
}

void task_baro(void)
{
    static float baro_alt_f_old=0;
    static float baro_alt_f=0;

    baro_update(0);

    float alt = baro_get_altitude(0);

    baro_alt_f = ((baro_alt_f * 0.7) + (alt * (1.0f - 0.7)));
    baro_vel = (baro_alt_f - baro_alt_f_old) / (0.025000f);
    baro_alt_f_old = baro_alt_f;

    baro_vari = variance_cal(&baro_variance, baro_vel);
}


void task_att(void)
{
	if(imu_is_update(0)) {
		perf_begin(&att_elapsed);
		est_att_run();
		perf_end(&att_elapsed);
		imu_clean_update(0);
		perf_interval(&att_perf);
	}
}

void task_commander(void)
{
    commander_update();
}

// void task_log(void)
// {
// 	if(!mtd_is_full() && log_need_record())
// 	{
// 	    log_write_att(50);
// 	    log_write_imu(500);
// 	    log_write_sens(50);
// 	}
// }


int main() 
{    
//    PRINT("hello feather flight\n");
#ifdef F3_EVO    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    RCC_ClearFlag();
#endif        
    timer_init();    
#ifdef F3_EVO    
    spi_flash_init();
    mtd_init();
    mtd_test();
#endif        
    // log_init();
     mavlink_init();
     wwlink_init();
     cli_init();

#ifdef F3_EVO    
    imu_register(&mpu6050.heir);
    baro_register(&ms5611.heir);
    compass_register(&hmc5883.heir);
#elif LINUX
    imu_register(&mpu6050_linux.heir);
    baro_register(&spl06_linux.heir);
#endif    
    sensor_init();

    att_est_register(&att_est_q.heir);
//    att_est_register(&att_est_cf.heir);
    est_init();
    perf_init(&main_perf);
    perf_init(&att_perf);
    perf_init(&att_elapsed);

    variance_create(&baro_variance, 100);

    task_create(&imu_task, 1000, task_imu);
//    task_create(&compass_task, (10000000 / 150), task_compass);
    task_create(&baro_task, 25000, task_baro);
    task_create(&att_task, 1000, task_att);
    task_create(&commander_task, 1000, task_commander);
    task_create(&link_task, 10*1000, task_link_send);
    task_create(&cli_task, 100*1000, task_cli);

    while(1)
    {
        scheduler_run();

        perf_interval(&main_perf);
    }
    
    return 0;
}
