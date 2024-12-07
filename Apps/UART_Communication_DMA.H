#ifndef _UART_Communication_DMA_H_
#define _UART_Communication_DMA_H_

#include "sc32_conf.h"

extern uint8_t UART_Communication_DMA_Data[];

void UART_Communication_DMA(void);
void UART_Communication_DMA_DAM0Handler(void);
void UART_Communication_DMA_DAM1Handler(void);
#endif
