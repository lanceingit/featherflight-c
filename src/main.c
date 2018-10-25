#include "board.h"

#include "sensor.h"
#ifdef F3_EVO
#include "spi_flash.h"
#include "mtd.h"
#endif
#include "log.h"
#include "link_mavlink.h"
#include "mavlink_log.h"
#include "timer.h"
#include "perf.h"
#include "vector.h"
#include "est.h"
#include "scheduler.h"


static void led_init(void);
void led_on(void);
void led_off(void);

void linkSendTask(void);
void linkRecvTask(void);
void sensorTask(void);  
void attTask(void);
void logTask(void);
void imuTask(void);
void compassTask(void);
void baroTask(void);

void gyro_cal(void);


struct perf_s main_perf;
struct perf_s att_perf;
struct perf_s att_elapsed;

struct task_s link_task;
struct task_s imu_task;
struct task_s compass_task;
struct task_s baro_task;
struct task_s att_task;

int main() 
{    
//    printf("hello feather flight\n");
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
     link_mavlink_init();

#ifdef F3_EVO    
    imu_register(&mpu6050.heir);
    baro_register(&ms5611.heir);
    compass_register(&hmc5883.heir);
#elif LINUX
    imu_register(&mpu6050_linux.heir);
    baro_register(&spl06_linux.heir);
#endif    
    sensor_init();

    gyro_cal();

    att_est_register(&att_est_q.heir);
//    att_est_register(&att_est_cf.heir);
    est_init();
    perf_init(&main_perf);
    perf_init(&att_perf);
    perf_init(&att_elapsed);

    task_create(&link_task, 10*1000, linkSendTask);
    task_create(&imu_task, 1000, imuTask);
    task_create(&compass_task, (10000000 / 150), compassTask);
    task_create(&baro_task, 25000, baroTask);
    task_create(&att_task, 1000, attTask);

    while(1)
    {
    #ifdef F3_EVO        
        led_on();
        led_off();      
    #endif        

        scheduler_run();

        perf_interval(&main_perf);
        
    }
    
    return 0;
}


#ifdef F3_EVO
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
#endif        

void linkSendTask(void)
{
    link_mavlink_stream();
}


void linkRecvTask(void)
{
	mavlink_message_t msg;

	if(link_mavlink_recv(&msg))
	{
        mavlink_log_handle(&msg);
	}
}

void imuTask(void)
{        
    imu_update(0);
}    

void compassTask(void)
{
    compass_update(0);
}

void baroTask(void)
{
    baro_update(0);
}


void attTask(void)
{
	if(imu_is_update(0)) {
		perf_begin(&att_elapsed);
		est_att_run();
		perf_end(&att_elapsed);
		imu_clean_update(0);
		perf_interval(&att_perf);
	}
}

void gyro_cal(void)         //TODO:put into sensor
{
	while(1) {
		Vector gyro;
		Vector gyro_sum;
		Vector accel_start;
		Vector accel_end;
		Vector accel_diff;

		imu_update(0);
		imu_get_acc(0,&accel_start);
        gyro_sum = vector_set(0,0,0);
        for(uint8_t i=0; i<50; i++) {
        	imu_update(0);
    		imu_get_gyro(0,&gyro);
            gyro_sum = vector_add(gyro_sum, gyro);
    		delay_ms(10);
        }
        imu_update(0);
		imu_get_acc(0,&accel_end);
        accel_diff = vector_sub(accel_start, accel_end);
		if(vector_length(accel_diff) >  0.2f) continue;

		imu_set_gyro_offset_x(0, gyro_sum.x/50);   
		imu_set_gyro_offset_y(0, gyro_sum.y/50);
		imu_set_gyro_offset_z(0, gyro_sum.z/50);
        
        return;
	}
}

// void logTask(void)
// {
// 	if(!mtd_is_full() && log_need_record())
// 	{
// 	    log_write_att(50);
// 	    log_write_imu(500);
// 	    log_write_sens(50);
// 	}
// }

