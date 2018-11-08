#include "board.h"

#include "log.h"
#include "timer.h"
#include "mtd.h"
#include "att_est_q.h"
#include "est.h"
#include "sensor.h"
#include <string.h>
#include "debug.h"

#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

typedef uint8_t(*log_func)(uint8_t id, uint16_t rate, void* pkt);

struct log_format_s 
{
	uint8_t type;
	uint8_t length;		// full packet length including header
	char name[5];
	char format[16];
	char labels[64];
};

struct log_s 
{
	struct log_format_s format;
	uint16_t rate;
	log_func pack;
};



static bool record;
static uint8_t buf[200];

void log_shell(int argc, char *argv[]);

uint8_t log_att_write(uint8_t id, uint16_t rate, void* p)
{
    struct log_att_s pkt = {
    	LOG_PACKET_HEADER_INIT(id),
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
	uint8_t len = sizeof(struct log_att_s);
	memcpy(p, &pkt, len);

	return len;
}

uint8_t log_imu_write(uint8_t id, uint16_t rate, void* p)
{
    struct log_imu_s pkt = {
    	LOG_PACKET_HEADER_INIT(id),
		.acc_x = imu_get_acc_x(0),
		.acc_y = imu_get_acc_y(0),
		.acc_z = imu_get_acc_z(0),
		.gyro_x = imu_get_gyro_x(0),
		.gyro_y = imu_get_gyro_y(0),
		.gyro_z = imu_get_gyro_z(0),
		.mag_x = compass_get_mag_x(0),
		.mag_y = compass_get_mag_y(0),
		.mag_z = compass_get_mag_z(0),
		.temp_acc = imu_get_temp(0),
		.temp_gyro = imu_get_temp(0),
		.temp_mag = 0.0f,
    };
	uint8_t len = sizeof(struct log_imu_s);
	memcpy(p, &pkt, len);

	return len;
}

uint8_t log_sens_write(uint8_t id, uint16_t rate, void* p)
{
    struct log_sens_s pkt = {
    	LOG_PACKET_HEADER_INIT(id),
		.baro_pres = baro_get_press(0),
		.baro_alt =  baro_get_altitude(0),
		.baro_temp = baro_get_temp(0),
    };
	uint8_t len = sizeof(struct log_sens_s);
	memcpy(p, &pkt, len);

	return len;
}

uint8_t log_alt_write(uint8_t id, uint16_t rate, void* p)
{
    struct log_alt_s pkt = {
    	LOG_PACKET_HEADER_INIT(id),
		.pos = alt_est_3o.heir.alt,
		.vel= alt_est_3o.heir.vel,
		// .baro_alt;
		// .baro_vel;
		.baro_corr = alt_est_3o.alt_err,
		// .acc_alt;
		// .acc_vel;
		.acc = alt_est_3o.heir.acc_neu_z,
		.bias = alt_est_3o.acc_corr,
    };
	uint8_t len = sizeof(struct log_alt_s);
	memcpy(p, &pkt, len);

	return len;
}

#include "log_messages.h"

static struct log_s log[] = {
	LOG_DEF(att, 250, "ffffffffffff",	"r,p,y,rsp,psp,ysp,rr,pr,yr,rrs,prs,yrs"),
	LOG_DEF(imu, 250, "ffffffffffff", "AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,tA,tG,tM"),
	LOG_DEF(sens, 250, "fff", "BaroPres,BaroAlt,BaroTemp"),
	LOG_DEF(alt, 250, "fffffffff","alt,vel,bp,bv,bc,ap,av,a,b"),
};

static const unsigned log_num = sizeof(log) / sizeof(log[0]);
static times_t timer[sizeof(log) / sizeof(log[0])];

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

void log_init()
{
	cli_regist("log", log_shell);
	record = true;

	struct {
		LOG_PACKET_HEADER
		struct log_format_s body;
	} log_msg_format = {
		LOG_PACKET_HEADER_INIT(LOG_FORMAT_MSG),
	};

	/* fill message format packet for each format and write it */
	for (unsigned i = 0; i < log_num; i++) {
		log[i].format.type = i;
		log_msg_format.body = log[i].format;
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

void log_run(void)
{
	if(!mtd_is_full() && log_need_record()) {
		for(uint8_t i=0; i<log_num; i++) {
			if(log[i].rate != 0) {
				if(timer_check(&timer[log[i].format.type], 1*1000*1000/log[i].rate)) {
					uint8_t len = log[i].pack(log[i].format.type, log[i].rate, (void*)buf);    
					// PRINT("log_write len:%d\n", len);
					log_write(buf, len);
				}
			}
		}
	}
	mtd_sync();
}


void log_shell(int argc, char *argv[])
{
	if(argc == 2) {
		if(strcmp(argv[1],"list") == 0) {
			PRINT("NAME\t\tID\t\tRATE/hz\t\tlen\n");
			for(uint8_t i=0; i<log_num; i++) {
				PRINT("[%s]\t\t%d\t\t%d\t\t%d\n", log[i].format.name, log[i].format.type, log[i].rate, log[i].format.length);
			}
			return;
		}
	}
	cli_device_write("missing command: try 'list' ");
}







