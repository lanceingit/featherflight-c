#include <stdbool.h>

#include "stm32f30x.h"

#include "sensor.h"
#include "spi_flash.h"
#include "mtd.h"
#include "log.h"
#include "link_mavlink.h"
#include "mavlink_log.h"
#include "att_est_q.h"
#include "timer.h"
#include "pref.h"
#include "vector.h"
#include "att_est_q.h"
#include "est.h"

static uint64_t last_inertial_sensor_update_time = 0;
static uint64_t last_compass_update_time = 0;
static uint64_t last_baro_update_time = 0;
static uint64_t last_heartbeat_update_time = 0;
static uint64_t last_imu_update_time = 0;
static uint64_t last_att_update_time = 0;
//static uint64_t last_att_run_time = 0;



static void led_init(void);
void led_on(void);
void led_off(void);

void linkSendTask(void);
void linkRecvTask(void);
void sensorTask(void);  
void attTask(void);
void logTask(void);

void gyro_cal(void);



struct Pref pref;
struct Pref attPref;
struct Pref attElapsed;


int main() 
{    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    RCC_ClearFlag();
        
    Timer_init();    
    spi_flash_init();
    mtd_init();
    mtd_test();
    log_init();
    link_mavlink_init();

    inertial_sensor_register(&mpu6050.heir);
    compass_register(&hmc5883.heir);
    baro_register(&ms5611.heir);
    sensor_init();

    gyro_cal();

    att_est_register(&att_est_q.heir);
    est_init();
    pref_init(&pref);
    pref_init(&attPref);
    pref_init(&attElapsed);

    while(1)
    {
        led_on();
        led_off();      
        linkSendTask();
        mavlink_log_run();
        linkRecvTask();
        sensorTask();
        attTask();
//        logTask();
//        mtd_sync();
        

    //  Timer::delayUs(1*1000*1000);
        pref_interval(&pref);
        
    }
    
    return 0;
}



void led_init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);			
}

void led_on(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_3); 
}

void led_off(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_3);
}

void linkSendTask(void)
{
    mavlink_message_t msg;
    uint8_t system_id=2;
    uint8_t component_id=1;    

    if(Timer_getTime() - last_heartbeat_update_time > 1000*1000)
    {
        mavlink_msg_heartbeat_pack(system_id, component_id, &msg,
                                       MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 
                                       0, 
                                       0, MAV_STATE_STANDBY);  

        link_mavlink_msg_send(&msg);
        last_heartbeat_update_time = Timer_getTime();
    }
    
    if(Timer_getTime() - last_imu_update_time > 20*1000)
    {
        mavlink_msg_highres_imu_pack(system_id, component_id, &msg,
                               Timer_getTime(),
                               inertial_sensor_get_acc_x(0), inertial_sensor_get_acc_y(0), inertial_sensor_get_acc_z(0),
                               inertial_sensor_get_gyro_x(0),inertial_sensor_get_gyro_y(0),inertial_sensor_get_gyro_z(0),
                               compass_get_mag_x(0), compass_get_mag_y(0), compass_get_mag_z(0),
                               baro_get_press(0), 0, baro_get_altitude(0), baro_get_temp(0),
                               0xFFFF);
        link_mavlink_msg_send(&msg);

        Vector v;
        inertial_sensor_get_acc(0, &v);
        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
                               Timer_getTime(),
                               "acc_len",
                               vector_length(v));
        link_mavlink_msg_send(&msg);

        compass_get_mag(0, &v);
        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
                               Timer_getTime(),
                               "mag_len",
                               vector_length(v));
        link_mavlink_msg_send(&msg);

        last_imu_update_time = Timer_getTime();
    }
    
    if(Timer_getTime() - last_att_update_time > 50*1000)
    {
        mavlink_msg_attitude_pack(system_id, component_id, &msg,
                               Timer_getTime(), 
                               att_get_roll(),
                               att_get_pitch(),
                               att_get_yaw(),
                               0,0,0         
                                 );
        link_mavlink_msg_send(&msg);


//        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
//                               Timer_getTime(),
//                               "bias_x",
//                               att_get_bias_x());
//        link_mavlink_msg_send(&msg);
//        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
//                               Timer_getTime(),
//                               "bias_y",
//                               att_get_bias_y());
//        link_mavlink_msg_send(&msg);

//        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
//                               Timer_getTime(),
//                               "corr_acc_x",
//                               att_get_corr_acc_x());
//        link_mavlink_msg_send(&msg);
//        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
//                               Timer_getTime(),
//                               "corr_acc_y",
//                               att_get_corr_acc_y());
//        link_mavlink_msg_send(&msg);

//        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
//                               Timer_getTime(),
//                               "corr_all_x",
//                               att_get_corr_all_x());
//        link_mavlink_msg_send(&msg);
//        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
//                               Timer_getTime(),
//                               "corr_all_y",
//                               att_get_corr_all_y());
//        link_mavlink_msg_send(&msg);

        last_att_update_time = Timer_getTime();
    }
}


void linkRecvTask(void)
{
	mavlink_message_t msg;

	if(link_mavlink_recv(&msg))
	{
        mavlink_log_handle(&msg);
	}
}

void sensorTask(void)
{        
    if(Timer_getTime() - last_inertial_sensor_update_time > 1000)
    {        
        inertial_sensor_update(0);
        last_inertial_sensor_update_time = Timer_getTime();
    }
    if(Timer_getTime() - last_compass_update_time > (1000000 / 150))
    {        
        compass_update(0);
        last_compass_update_time = Timer_getTime();
    }
    if(Timer_getTime() - last_baro_update_time > 25000)
    {
        baro_update(0);
        last_baro_update_time = Timer_getTime();
    }
}

void attTask(void)
{
	if(inertial_sensor_is_update(0))
	{
		pref_begin(&attElapsed);
		est_att_run();
		pref_end(&attElapsed);
		inertial_sensor_clean_update(0);
		pref_interval(&attPref);
	}
}

void gyro_cal(void)         //TODO:put into sensor
{
	while(1)
	{
		Vector gyro;
		Vector gyro_sum;
		Vector accel_start;
		Vector accel_end;
		Vector accel_diff;

		inertial_sensor_update(0);
		inertial_sensor_get_acc(0,&accel_start);
        gyro_sum = vector_set(0,0,0);
        for(uint8_t i=0; i<50; i++) {
        	inertial_sensor_update(0);
    		inertial_sensor_get_gyro(0,&gyro);
            gyro_sum = vector_add(gyro_sum, gyro);
    		Timer_delayUs(10*1000);
        }
        inertial_sensor_update(0);
		inertial_sensor_get_acc(0,&accel_end);
        accel_diff = vector_sub(accel_start, accel_end);
		if(vector_length(accel_diff) >  0.2f) continue;

		inertial_sensor_set_gyro_offset_x(0, gyro_sum.x/50);   
		inertial_sensor_set_gyro_offset_y(0, gyro_sum.y/50);
		inertial_sensor_set_gyro_offset_z(0, gyro_sum.z/50);
        
        return;
	}
}

void logTask(void)
{
	if(!mtd_is_full() && log_need_record())
	{
	    log_write_att(50);
	    log_write_imu(500);
	    log_write_sens(50);
	}
}

