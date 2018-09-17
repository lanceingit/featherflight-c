#pragma once

#include "log_messages.h"




void log_init(void);
void log_write(void* pkt, uint16_t len);
uint16_t log_read(uint32_t offset, uint8_t* data, uint16_t len);

void log_write_att(uint16_t rate);
void log_write_imu(uint16_t rate);
void log_write_sens(uint16_t rate);

uint32_t log_get_size(void);

void log_stop(void);
bool log_need_recordvoid();

