#pragma once

#include "mavlink.h"

void link_mavlink_init(void);
void link_mavlink_msg_send(mavlink_message_t* msg);
bool link_mavlink_recv(mavlink_message_t* msg);


