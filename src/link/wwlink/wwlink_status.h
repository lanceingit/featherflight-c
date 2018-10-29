#pragma once

#define WWLINK_ITEM_STATUS_BASIC_INFO			0
#define WWLINK_ITEM_STATUS_NUM					1

typedef struct PACKED {
	float att[3];
	float vel_ned[3];
	float pos_ned[3];
	uint8_t status;
	uint8_t mode;
	uint8_t capacity;
	uint8_t voltage;
	bool  temp;
	bool  charge;
	bool  armed;
}wwlink_status_base_info_s;

static inline void wwlink_encode_status_base_info()
{
	uint8_t mode = 0;
	wwlink_message_t msg;
	wwlink_status_base_info_s data;	

    //data =   //TODO:

	msg.item_id = WWLINK_ITEM_STATUS;
	msg.subitem_id = WWLINK_ITEM_STATUS_BASIC_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	wwlink_encode(&msg);
}

static inline void wwlink_status_handle(wwlink_message_t* msg)
{
}
