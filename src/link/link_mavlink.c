#include "board.h"

#ifdef F3_EVO
#include "serial.h"
#elif LINUX
#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#endif
#include "mavlink.h"
#include "link_mavlink.h"
#include "mavlink.h"

#define RX_BUF_SIZE 300    
#define TX_BUF_SIZE 300    

uint8_t sendbuf[300];
#ifdef F3_EVO
struct serial_s * _port;
#elif LINUX
#define UDP_PORT  14550
#define UDP_IP    "192.168.100.255"
static struct sockaddr_in addr;
static struct sockaddr_in bcast_addr;
static struct sockaddr_in remote_addr;
static int _fd = -1;
static int addr_len = 0;
#endif
uint8_t _rxBuf[RX_BUF_SIZE]; 
uint8_t _txBuf[TX_BUF_SIZE];

mavlink_status_t _r_mavlink_status;
mavlink_message_t msg;

void handle_log_request_list(mavlink_message_t *msg);

void link_mavlink_init()
{   
#ifdef F3_EVO   
    _port = serial_open(USART1, 921600, _rxBuf, RX_BUF_SIZE, _txBuf, TX_BUF_SIZE);
#elif LINUX
	int flag = 0;

	if((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("socket failed\n");
		exit(1);
	}

	bzero(&addr, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(14556);
	addr.sin_addr.s_addr = htonl(INADDR_ANY) ;
	flag = fcntl(_fd , F_GETFL , 0);
	fcntl(_fd,F_SETFL,flag | O_NONBLOCK);

	if(bind(_fd, (struct sockaddr *)&addr, sizeof(addr))<0){
		perror("connect failed\n");
		exit(1);
	}else{
	}

	bzero(&bcast_addr, sizeof(bcast_addr));
	bcast_addr.sin_family = AF_INET;
	bcast_addr.sin_port = htons(UDP_PORT);
	bcast_addr.sin_addr.s_addr = inet_addr("192.168.100.255") ;

	addr_len = sizeof(bcast_addr);

	int broadcast_opt = 1;

	if (setsockopt(_fd, SOL_SOCKET, SO_BROADCAST, &broadcast_opt, sizeof(broadcast_opt)) < 0) {
		perror("setting broadcast permission failed");
	}    
#endif    
}    


//void link_mavlink_send(mavlink_message_t &msg)
//{
//    mavlink_message_t msg;
//    uint16_t len;
//    
//    mavlink_msg_heartbeat_pack(0, 0, &msg,
//                               MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 
//                               MAV_MODE_FLAG_TEST_ENABLED, 0, MAV_STATE_STANDBY);
//    
//    len = mavlink_msg_to_send_buffer(sendbuf, &msg);
//    
//    for(uint16_t i=0; i<len; i++)
//    {
//        USART_SendData(USART1, sendbuf[i]);
//        while (!USART_GetFlagStatus(USART1,USART_FLAG_TC));
//    }
//    
//    //vTaskDelay(M2T(200));
//    
//    mavlink_msg_sys_status_pack(0, 0, &msg,
//                               0, 0, 0,10, 0, 0, 0, 0, 0, 0,0, 0, 0)    ;

//    len = mavlink_msg_to_send_buffer(sendbuf, &msg);
//    
//    for(uint16_t i=0; i<len; i++)
//    {
//        USART_SendData(USART1, sendbuf[i]);
//        while (!USART_GetFlagStatus(USART1,USART_FLAG_TC));
//    }    
//    
//}

void link_mavlink_msg_send(mavlink_message_t* msg)
{
    uint16_t len;
       
    len = mavlink_msg_to_send_buffer(sendbuf, msg);
#ifdef F3_EVO
    serial_write(_port, sendbuf, len);    
#elif LINUX
    sendto(_fd, sendbuf, len, 0, (struct sockaddr *)&bcast_addr, addr_len);    
#endif
}

bool link_mavlink_recv(mavlink_message_t* msg)
{
    uint8_t c;

#ifdef F3_EVO
    if(serial_read(_port, &c) == 0) {
        if(mavlink_parse_char(0, c, msg, &_r_mavlink_status))
        {
            return true;
        }
    }
#endif

	return false;
}





