#ifndef ADC_IAP_H
#define ADC_IAP_H

#include <stdint.h>
#include "sc32f1xxx_adc.h"
// ADC相关函数
void initADC(void);
uint16_t readADC(ADC_ChannelTypedef channel);
float adcToVoltage(uint16_t adcValue);
void checkPowerLoss(void);

// IAP相关函数
void initIAP(void);
void eraseFlashPage(uint32_t address);
void writeFlash(uint32_t address, uint8_t* data, uint32_t length);
void readFlash(uint32_t address, uint8_t* data, uint32_t length);

// 闹钟设置相关函数
typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t enabled;  // 是否启用闹钟
} AlarmSettings;

void saveAlarmSettings(void);
void loadAlarmSettings(void);
void setAlarm(uint8_t hour, uint8_t minute, uint8_t enabled);

#endif // ADC_IAP_H
