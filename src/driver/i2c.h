#pragma once


void i2c_init(void);
bool i2c_read(uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);
bool i2c_write(uint8_t addr_, uint8_t reg, uint8_t data);



