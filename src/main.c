#include <stdbool.h>

#include "stm32f30x.h"


#include "mpu6050.h"
#include "ms5611.h"
#include "hmc5883.h"
#include "spi_flash.h"
#include "mtd.h"
#include "log.h"
#include "link_mavlink.h"
#include "mavlink_log.h"
#include "att_est_q.h"
#include "timer.h"

static uint64_t last_mpu6050_updata_time = 0;
static uint64_t last_hmc5883_updata_time = 0;
static uint64_t last_ms5611_updata_time = 0;
static uint64_t last_heartbeat_updata_time = 0;
static uint64_t last_imu_updata_time = 0;
static uint64_t last_att_updata_time = 0;
static uint64_t last_att_run_time = 0;



static void led_init(void);
void led_on(void);
void led_off(void);

void linkSendTask(void);
void linkRecvTask(void);
void sensorTask(void);  
void attTask(void);
void logTask(void);

void gyro_cal(void);



struct Pref
{
    uint64_t t0,t1;
    uint64_t cnt,avg,sum,t_max,t_min;
};

void pref_init(struct Pref* pref)
{
    pref->t0=0;
    pref->t1 = 0;
    pref->cnt=0;
    pref->avg=0;
    pref->sum=0;
    pref->t_max=0;
    pref->t_min=0xFFFFFFFF;
}

void pref_interval(struct Pref* pref)
{
    pref.t1 = Timer_getTime() - pref.t0;
    if(pref->t0 != 0)
    {
        pref->sum += pref->t1;
        pref->cnt++;
        pref->avg = pref->sum / pref->cnt;
        if(pref->t1>pref->t_max) pref->t_max=pref->t1;
        if(pref->t1<pref->t_min) pref->t_min=pref->t1;
    }
    pref->t0 = Timer_getTime();
}

void pref_begin(struct Pref* pref)
{
    pref->t0 = Timer_getTime();
}

void pref_end(struct Pref* pref)
{
    pref->t1 = Timer_getTime() - pref->t0;
    if(pref->t0 != 0)
    {
        pref->sum += pref->t1;
        pref->cnt++;
        pref->avg = pref->sum / pref->cnt;
        if(pref->t1>pref->t_max) pref->t_max=pref->t1;
        if(pref->t1<pref->t_min) pref->t_min=pref->t1;
    }
}


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
    logging_init();
    link_mavlink_init();
    mpu6050_init();

    gyro_cal();

    hmc5883_init();
    ms5611_init();
    att_init();
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
        logTask();
        mtd_sync();
        

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

    if(Timer_getTime() - last_heartbeat_updata_time > 1000*1000)
    {
        mavlink_msg_heartbeat_pack(system_id, component_id, &msg,
                                       MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 
                                       0, 
                                       0, MAV_STATE_STANDBY);  

        link_mavlink_msg_send(msg);
        last_heartbeat_updata_time = Timer_getTime();
    }
    
    if(Timer_getTime() - last_imu_updata_time > 20*1000)
    {
        mavlink_msg_highres_imu_pack(system_id, component_id, &msg,
                               Timer_getTime(),
                               mpu6050_get_acc_x(), mpu6050_get_acc_y(), mpu6050_get_acc_z(),
                               mpu6050_get_gyro_x(),mpu6050_get_gyro_y(),mpu6050_get_gyro_z(),
                               hmc5883_get_mag_x(), hmc5883_get_mag_y(), hmc5883_get_mag_z(),
                                ms5611_get_press(), 0, ms5611_get_altitude(), ms5611_get_temp(),
                               0xFFFF);
        link_mavlink_msg_send(msg);

        float x=mpu6050_get_acc_x();
        float y=mpu6050_get_acc_y();
        float z=mpu6050_get_acc_z();
        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
                               Timer_getTime(),
                               "acc_len",
                               sqrt(x*x+y*y+z*z));
        link_mavlink_msg_send(msg);

        x=hmc5883_get_mag_x();
        y=hmc5883_get_mag_y();
        z=hmc5883_get_mag_z();
        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
                               Timer_getTime(),
                               "mag_len",
                               sqrt(x*x+y*y+z*z));
        link_mavlink_msg_send(msg);



        last_imu_updata_time = Timer_getTime();
    }
    
    if(Timer_getTime() - last_att_updata_time > 50*1000)
    {
        mavlink_msg_attitude_pack(system_id, component_id, &msg,
                               Timer_getTime(), 
                               att_get_roll(),
                               att_get_pitch(),
                               att_get_yaw(),
                               0,0,0         
                                 );
        link_mavlink_msg_send(msg);


        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
                               Timer_getTime(),
                               "bias_x",
                               att_get_bias_x());
        link_mavlink_msg_send(msg);
        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
                               Timer_getTime(),
                               "bias_y",
                               att_get_bias_y());
        link_mavlink_msg_send(msg);

        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
                               Timer_getTime(),
                               "corr_acc_x",
                               att_get_corr_acc_x());
        link_mavlink_msg_send(msg);
        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
                               Timer_getTime(),
                               "corr_acc_y",
                               att_get_corr_acc_y());
        link_mavlink_msg_send(msg);

        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
                               Timer_getTime(),
                               "corr_all_x",
                               att_get_corr_all_x());
        link_mavlink_msg_send(msg);
        mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
                               Timer_getTime(),
                               "corr_all_y",
                               att_get_corr_all_y());
        link_mavlink_msg_send(msg);

        last_att_updata_time = Timer_getTime();
    }
}


void linkRecvTask(void)
{
	mavlink_message_t msg;

	if(link_mavlink_recv(msg))
	{
        mavlink_log_handle(msg);
	}
}

void sensorTask(void)
{        
    if(Timer_getTime() - last_mpu6050_updata_time > 1000)
    {        
        mpu6050_read();
        last_mpu6050_updata_time = Timer_getTime();
    }
    if(Timer_getTime() - last_hmc5883_updata_time > (1000000 / 150))
    {        
        hmc5883_read();
        last_hmc5883_updata_time = Timer_getTime();
    }
    if(Timer_getTime() - last_ms5611_updata_time > 25000)
    {
        ms5611_read();
        last_ms5611_updata_time = Timer_getTime();
    }
}

void attTask(void)
{
	if(inertialSensor_update())
	{
		pref_begin(attElapsed);
		att_run();
		pref_end(attElapsed);
		inertialSensor_clean_update();
		pref_interval(attPref);
	}
}

void gyro_cal(void)
{
	while(1)
	{
		math::Vector<3>	gyro;
		math::Vector<3>	gyro_sum;
		math::Vector<3>	accel_start;
		math::Vector<3>	accel_end;
		math::Vector<3>	accel_diff;

		mpu6050.read();
		sensor.inertialSensor->get_accel(accel_start.data);
		gyro_sum.zero();
        for (uint8_t i=0; i<50; i++) {
        	mpu6050.read();
    		sensor.inertialSensor->get_gyro(gyro.data);
    		gyro_sum += gyro;
    		Timer_delayUs(10*1000);
        }
        mpu6050.read();
		sensor.inertialSensor->get_accel(accel_end.data);
		accel_diff = accel_start - accel_end;
		if(accel_diff.length() >  0.2f) continue;

		sensor.inertialSensor->set_gyro_offset_x(gyro_sum(0)/50);
		sensor.inertialSensor->set_gyro_offset_y(gyro_sum(1)/50);
		sensor.inertialSensor->set_gyro_offset_z(gyro_sum(2)/50);
        
        return;
	}
}

void logTask(void)
{
	if(!mtd_is_full() && logging_need_record())
	{
	    logging_write_att(att, 50);
	    logging_write_imu(sensor, 500);
	    logging_write_sens(sensor, 50);
	}
}

