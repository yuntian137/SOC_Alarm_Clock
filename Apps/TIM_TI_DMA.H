#ifndef _TIM_TI_DMA_H_
#define _TIM_TI_DMA_H_

#include "sc32_conf.h"

extern uint8_t TIM_TI_DMA_SrcData[], TIM_TI_DMA_DstData[];

void TIM_TI_DMA(void);
void TIM_TI_DMA_DMA0Handler(void);
#endif
