#ifndef __USART2_H
#define __USART2_H
#include "stm32f10x.h"
#include "ringbuffer.h"
extern ringbuffer_t uart2RxFifo;
void usart2_init(uint32_t bound);
void usart2_sendData(uint8_t *data);
uint16_t usart2_getRxData(uint8_t *buf, uint16_t len);
void usart2_poll_rx(void); // 新增的轮询函数声明

#endif
