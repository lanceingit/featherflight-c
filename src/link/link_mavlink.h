#pragma once

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#define MAVLINK_SEND_UART_BYTES mavlink_send

#include "mavlink_types.h"
extern mavlink_system_t mavlink_system;



#define MAV_CH      MAVLINK_COMM_0
#define MAV_SYS     2
#define MAV_COMP    1



void link_mavlink_init(void);
void link_mavlink_msg_send(mavlink_message_t* msg);
bool link_mavlink_recv(mavlink_message_t* msg);
void mavlink_send(mavlink_channel_t chan, const uint8_t *ch, uint16_t length);

#include "mavlink.h"

