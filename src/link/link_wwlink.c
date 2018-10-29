#include "board.h"
#include "link_wwlink.h"
#include "debug.h"
#include "timer.h"

#if LINUX 
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#endif

#define WWLINK_PARSE_STEP_MAGIC		1
#define WWLINK_PARSE_STEP_LENGTH	2
#define WWLINK_PARSE_STEP_ID_SRC	3
#define WWLINK_PARSE_STEP_I_ID		4
#define WWLINK_PARSE_STEP_SI_ID		5
#define WWLINK_PARSE_STEP_DATA		6
#define WWLINK_PARSE_STEP_CHECKSUM1	7
#define WWLINK_PARSE_STEP_CHECKSUM2	8
#define WWLINK_PARSE_STEP_OK		9
#define WWLINK_PARSE_STEP_FAIL		10

#if LINUX 
#define UDP_PORT  9696
struct sockaddr_in addr;
static int socket_fd = -1;
static int addr_len = 0;
#endif

static uint8_t type;
static uint16_t wwlink_parse_error;
static uint8_t  wwlink_parse_step;
static uint8_t  wwlink_parse_data_count;
static uint16_t wwlink_parse_checksum;
static wwlink_message_t wwlink_package;
static uint8_t data[100];

void wwlink_init(void)
{
#ifdef F3_EVO
#elif LINUX    
	int flag = 0;

	if((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("socket failed\n");
		exit(1);
	}
	
	bzero(&addr, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(UDP_PORT);
	addr.sin_addr.s_addr = htonl(INADDR_ANY) ;
	flag = fcntl(socket_fd , F_GETFL , 0);
	fcntl(socket_fd,F_SETFL,flag | O_NONBLOCK);
	
	if(bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr))<0){
		perror("connect failed\n");
		exit(1);
	}else{
		PRINT("wwlink init ok\n");	
	}
#endif    
}

void wwlink_handle_message(wwlink_message_t* msg)
{
    switch(msg->id_src)
    {
        case WWLINK_ITEM_SYSTEM:
            wwlink_sys_handle(msg);
            break;
        case WWLINK_ITEM_STATUS:
            wwlink_status_handle(msg);
            break;
        case WWLINK_ITEM_CONTROL:
            wwlink_control_handle(msg);
            break;
        default:break;    
    }
}

bool wwlink_parse_char(uint8_t ch, wwlink_message_t* msg)
{
	bool ret = false;
	
	switch(wwlink_parse_step){
		case WWLINK_PARSE_STEP_MAGIC:
			if(ch == WWLINK_MAGIC){
				wwlink_parse_checksum = wwlink_crc16_init();
				wwlink_parse_step = WWLINK_PARSE_STEP_LENGTH;
				wwlink_package.checksum = ch;
			}
			break;
		case WWLINK_PARSE_STEP_LENGTH:
			wwlink_parse_step = WWLINK_PARSE_STEP_ID_SRC;
			wwlink_parse_checksum = wwlink_crc16_update(ch,wwlink_parse_checksum);
			wwlink_package.length = ch;
			break;
		case WWLINK_PARSE_STEP_ID_SRC:
			wwlink_parse_step = WWLINK_PARSE_STEP_I_ID;
			wwlink_parse_checksum = wwlink_crc16_update(ch,wwlink_parse_checksum);
			wwlink_package.id_src = ch;
			break;
		case WWLINK_PARSE_STEP_I_ID:
			wwlink_parse_step = WWLINK_PARSE_STEP_SI_ID;
			wwlink_parse_checksum = wwlink_crc16_update(ch,wwlink_parse_checksum);
			wwlink_package.item_id = ch;
			break;
		case WWLINK_PARSE_STEP_SI_ID:
			wwlink_parse_step = WWLINK_PARSE_STEP_DATA;
			wwlink_parse_checksum = wwlink_crc16_update(ch,wwlink_parse_checksum);
			wwlink_package.subitem_id = ch;
			if(wwlink_package.length == 0){				
				wwlink_parse_step = WWLINK_PARSE_STEP_CHECKSUM1;
			}	
			break;
		case WWLINK_PARSE_STEP_DATA:
			wwlink_parse_checksum = wwlink_crc16_update(ch,wwlink_parse_checksum);
            data[wwlink_parse_data_count] = ch;
			
			wwlink_parse_data_count++;
			if(wwlink_parse_data_count >= wwlink_package.length){
				wwlink_parse_step = WWLINK_PARSE_STEP_CHECKSUM1;
			}
			break;
		case WWLINK_PARSE_STEP_CHECKSUM1:
			if(ch == (wwlink_parse_checksum & 0xFF)){
				wwlink_parse_step = WWLINK_PARSE_STEP_CHECKSUM2;
			}
			break;
		case WWLINK_PARSE_STEP_CHECKSUM2:
			if(ch == ((wwlink_parse_checksum >> 8) & 0xFF)){
				ret = true;
			}
			break;
	}

	return ret;
}

bool wwlink_recv(wwlink_message_t* msg)
{
#ifdef LINUX
    int len = 0;
    uint16_t check_len = 0;
    uint8_t buffer[300] = {};
    bzero(buffer, sizeof(buffer));
    len = recvfrom(socket_fd, buffer, sizeof(buffer), 0,(struct sockaddr *)&addr, (socklen_t*)&addr_len);

    if(len > 0) {
        for(check_len = 0; check_len < len; check_len++) {
            if(wwlink_parse_char(buffer[check_len], msg)) {
                wwlink_handle_message(msg);
            }
        }
    }
#else    
	return false;
#endif
}

void wwlink_send(uint8_t* data, uint16_t len)
{
#ifdef LINUX
    sendto(socket_fd, data, len, 0, (struct sockaddr *)&addr,addr_len);
#endif
}


static times_t last_heartbeat_update_time = 0;

void wwlink_stream(void)
{
    if(timer_check(&last_heartbeat_update_time, 1000*1000))
    {
		awlink_encode_system_heart(0);
		PRINT("send heart beat\n");
    }
}
