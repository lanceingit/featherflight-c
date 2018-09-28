#include "board.h"

#include "serial.h"
#include "mavlink.h"
#include "link_mavlink.h"
#include "mavlink.h"

#define RX_BUF_SIZE 300    
#define TX_BUF_SIZE 300    

uint8_t sendbuf[300];
struct serial_s * _port;
uint8_t _rxBuf[RX_BUF_SIZE]; 
uint8_t _txBuf[TX_BUF_SIZE];

mavlink_status_t _r_mavlink_status;
mavlink_message_t msg;

void handle_log_request_list(mavlink_message_t *msg);

void link_mavlink_init()
{   
    _port = serial_open(USART1, 921600, _rxBuf, RX_BUF_SIZE, _txBuf, TX_BUF_SIZE);
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
    serial_write(_port, sendbuf, len);    
}

bool link_mavlink_recv(mavlink_message_t* msg)
{
    uint8_t c;

    if(serial_read(_port, &c) == 0) {
        if(mavlink_parse_char(0, c, msg, &_r_mavlink_status))
        {
            return true;
        }
    }

	return false;
}


//void link_mavlink_recv()
//{
//    uint8_t c;
//    //mavlink_message_t msg={};
//
//    if(serialAvailable(_port))
//    {
//        c = serialRead(_port);
//    	//send_text("recv usb");
//        if(mavlink_parse_char(0, c, &msg, &_r_mavlink_status))
//        {
//            //DEBUG_PRINTF("msg id:%d  %x %x %x %x \r\n", msg.msgid, msg.payload64[0], msg.payload64[1], msg.payload64[2], msg.payload64[3]);
////        	send_text("recv msg");
//            switch(msg.msgid)
//            {
//                case MAVLINK_MSG_ID_COMMAND_LONG:
//				{
//                    uint16_t cmd = mavlink_msg_command_long_get_command(&msg);
//                    //DEBUG_PRINTF("cmd:%d \r\n", mavlink_msg_command_long_get_command(&msg));
//                    if(cmd == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)
//                    {
//                        NVIC_SystemReset();
//                    }
//                    break;
//				}
//
//                case MAVLINK_MSG_ID_HIL_SENSOR:
//                {
////                	send_text("recv hil");
//                	break;
//                }
//
//
//                case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
//                    handle_log_request_list(&msg);
//                    break;
//                case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
////                    handle_log_request_data(msg, dataflash);
//                    break;
//                case MAVLINK_MSG_ID_LOG_ERASE:
////                    handle_log_request_erase(msg, dataflash);
//                    break;
//                case MAVLINK_MSG_ID_LOG_REQUEST_END:
////                    handle_log_request_end(msg, dataflash);
//                    break;
//
//                default:break;
//            }
//        }
//    }
//}


//void link_mavlink_handle_log_request_list(mavlink_message_t *msg)
//{
////    mavlink_log_request_list_t packet;
////    mavlink_msg_log_request_list_decode(msg, &packet);
//
//    mavlink_msg_named_value_float_pack(system_id, component_id, &msg,
//                           Timer_getTime(),
//                           "mag_len",
//                           sqrt(x*x+y*y+z*z));
//
//    mavlink_message_t msg_send;
//    uint8_t system_id=2;
//    uint8_t component_id=1;
//
//    mavlink_msg_log_entry_encode(system_id, component_id, &msg_send,
//    						     0, 1, 1, 0, uint32_t size);
//    msg_send(msg_send);
//}
