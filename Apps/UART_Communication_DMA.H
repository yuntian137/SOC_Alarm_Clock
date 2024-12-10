#ifndef _UART_Communication_DMA_H_
#define _UART_Communication_DMA_H_

#include "sc32_conf.h"

extern uint8_t UART_Communication_DMA_Data[];
extern uint8_t UART_Communication_DMA_Buffer[];

void UART_Communication_DMA(void);
void UART_Communication_DMA_DAM0Handler(void);
void UART_Communication_DMA_DAM1Handler(void);
void UART_DMA_Send(uint8_t* data, uint16_t length);
void UART1_IRQHandler_DMA(void);
void UART_DMA_Receive(void);
#endif
