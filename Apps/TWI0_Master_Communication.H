#ifndef _TWI0_Master_Communication_H_
#define _TWI0_Master_Communication_H_

#include "sc32_conf.h"

extern uint8_t TWI0_SendDataArray[];
extern uint8_t TWI0_ReceiveDataArray[];

void TWI0_Master_Communication(void);
void TWI0_Master_Communication_TWI0Handler(void);
#endif
