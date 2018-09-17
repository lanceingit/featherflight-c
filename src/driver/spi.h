#pragma once


void spi_init(void);
bool spi_transferByte(uint8_t* out, uint8_t in);
bool spi_transfer(uint8_t *out, const uint8_t *in, int len);
void spi_setDivisor(uint16_t divisor);
bool spi_readIdentification();


