#ifndef _ADC_Conversion_H_
#define _ADC_Conversion_H_

#include "sc32_conf.h"

void ADC_Conversion(void);
void ADC_Conversion_IRQ(void);
uint16_t ADC_GetAverageValue(void);
#endif
