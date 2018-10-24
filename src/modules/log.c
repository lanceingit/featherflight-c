#include "board.h"

#include "log.h"
#include "log_messages.h"
#include "timer.h"
#include "mtd.h"
#include "att_est_q.h"
#include "est.h"
#include "sensor.h"

static times_t timer[sizeof(log_formats) / sizeof(log_formats[0])];
static bool record;



void log_init()
{
	record = true;
	/* construct message format packet */
	struct {
		LOG_PACKET_HEADER
		struct log_format_s body;
	} log_msg_format = {
		LOG_PACKET_HEADER_INIT(LOG_FORMAT_MSG),
	};

	/* fill message format packet for each format and write it */
	for (unsigned i = 0; i < log_formats_num; i++) {
		log_msg_format.body = log_formats[i];
		log_write(&log_msg_format, sizeof(log_msg_format));
	}
}

void log_write(void* pkt, uint16_t len)
{
    mtd_write((uint8_t*)pkt, len);
}

uint16_t log_read(uint32_t offset, uint8_t* data, uint16_t len)
{
	return mtd_read(offset, data, len);
}

void log_write_att(uint16_t rate)
{
    struct log_ATT_s pkt = {
    	LOG_PACKET_HEADER_INIT(LOG_ATT_MSG),
		.roll = att_get_roll(),
		.pitch = att_get_pitch(),
		.yaw = att_get_yaw(),
		.roll_sp = 0.0f,
		.pitch_sp = 0.0f,
		.yaw_sp = 0.0f,
//		.roll_rate = att_get_roll_rate(),
//		.pitch_rate = att_get_pitch_rate(),
//		.yaw_rate = att_get_yaw_rate(),
		.roll_rate_sp = 0.0f,
		.pitch_rate_sp = 0.0f,
		.yaw_rate_sp = 0.0f,
    };
    
    if(timer_check(&timer[LOG_ATT_MSG], 1*1000*1000/rate))
    {
        log_write(&pkt, sizeof(pkt));
    }
}

void log_write_imu(uint16_t rate)
{
    struct log_IMU_s pkt = {
    	LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
		.acc_x = imu_get_acc_x(0),
		.acc_y = imu_get_acc_x(0),
		.acc_z = imu_get_acc_z(0),
		.gyro_x = imu_get_gyro_x(0),
		.gyro_y = imu_get_gyro_y(0),
		.gyro_z = imu_get_gyro_z(0),
		.mag_x = compass_get_mag_x(0),
		.mag_y = compass_get_mag_y(0),
		.mag_z = compass_get_mag_z(0),
		.temp_acc = 0.0f,
		.temp_gyro = 0.0f,
		.temp_mag = 0.0f,
    };
    if(timer_check(&timer[LOG_IMU_MSG], 1*1000*1000/rate))
    {
        log_write(&pkt, sizeof(pkt));
    }
}

void log_write_sens(uint16_t rate)
{
    struct log_SENS_s pkt = {
    	LOG_PACKET_HEADER_INIT(LOG_SENS_MSG),
		.baro_pres = baro_get_press(0),
		.baro_alt =  baro_get_altitude(0),
		.baro_temp = baro_get_temp(0),
    };
    if(timer_check(&timer[LOG_SENS_MSG], 1*1000*1000/rate))
    {
        log_write(&pkt, sizeof(pkt));
    }
}

uint32_t log_get_size(void)
{
	return mtd_get_store();
}

void log_stop(void)
{
	record = false;
}

bool log_need_record(void)
{
	return record;
}









