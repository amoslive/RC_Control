#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"

#define USART_REC_LEN  	 200
#define EN_USART1_RX 			1
#define EN_USART2_RX      1   // 关键：使能USART2接收

extern u8  USART_RX_BUF[USART_REC_LEN];
extern u16 USART_RX_STA;
extern UART_HandleTypeDef UART2_Handler;

#define RXBUFFERSIZE   1
extern u8 aRxBuffer[RXBUFFERSIZE];

void uart_init(u32 bound);

#endif
