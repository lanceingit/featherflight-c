#pragma once


#include "fifo.h"


typedef struct {
    uint32_t baudRate;

    uint16_t txBufSize;
    uint8_t *txBuf;
    Fifo txFifo;

    uint16_t rxBufSize;
    uint8_t *rxBuf;
    Fifo rxFifo;

    USART_TypeDef *USARTx;
} serialPort_t;

serialPort_t * serialOpen(USART_TypeDef *USARTx, uint32_t baud, uint8_t* rxBuf, uint16_t rxBufSize, uint8_t* txBuf, uint16_t txBufSize);
void serialWrite(serialPort_t *s, unsigned char ch);
void serialWriteMass(serialPort_t *s, unsigned char* buf, uint16_t len);
bool serialAvailable(serialPort_t *s);
uint8_t serialRead(serialPort_t *s);






