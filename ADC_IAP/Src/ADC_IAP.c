#include "adc_iap.h"
#include "sc32f1xxx_iap.h"
#include "sc32f1xxx_adc.h"

// ADC配置
#define ADC_CHANNEL 0  // 假设使用ADC通道0
#define VOLTAGE_THRESHOLD 4.5  // 掉电检测阈值电压
#define ADC_RESOLUTION 16383  // 12位ADC的满量程值
#define REFERENCE_VOLTAGE 5.0  // 参考电压为5V

// IAP配置
#define ALARM_SETTINGS_ADDRESS 0x0800F000  // 假设Flash地址为0x0800F000

// 读取ADC值
uint16_t readADC(uint8_t channel) {
    ADC_SetChannel(ADC, channel);
    ADC_SoftwareStartConv(ADC);
    while (ADC_GetFlagStatus(ADC, ADC_Flag_ADCIF) == RESET);
    ADC_ClearFlag(ADC, ADC_Flag_ADCIF);
    return ADC_GetConversionValue(ADC);
}

// 将ADC值转换为电压
float adcToVoltage(uint16_t adcValue) {
    return (adcValue * REFERENCE_VOLTAGE) / ADC_RESOLUTION;
}

// 检查电源电压是否低于阈值
void checkPowerLoss(void) {
    uint16_t adcValue = readADC(ADC_CHANNEL);
    float voltage = adcToVoltage(adcValue);

    if (voltage < VOLTAGE_THRESHOLD) {
        saveAlarmSettings();
    }
}

// IAP初始化
void initIAP(void) {
    // 配置IAP模块
    // 例如：设置Flash的起始地址和大小
    IAP_Unlock();
}

// 擦除Flash页
void eraseFlashPage(uint32_t address) {
    IAP_EraseSector(address);
}

// 写入Flash
void writeFlash(uint32_t address, uint8_t* data, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        IAP_ProgramByte(address + i, data[i]);
    }
}

// 读取Flash
void readFlash(uint32_t address, uint8_t* data, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        data[i] = IAP_ReadByte(address + i);
    }
}

// 保存闹钟设置
void saveAlarmSettings(void) {
    AlarmSettings settings = {
        .hour = 7,
        .minute = 30,
        .enabled = 1
    };

    eraseFlashPage(ALARM_SETTINGS_ADDRESS);
    writeFlash(ALARM_SETTINGS_ADDRESS, (uint8_t*)&settings, sizeof(settings));
}

// 读取闹钟设置
void loadAlarmSettings(void) {
    AlarmSettings settings;
    readFlash(ALARM_SETTINGS_ADDRESS, (uint8_t*)&settings, sizeof(settings));

    setAlarm(settings.hour, settings.minute, settings.enabled);
}

// 设置闹钟
void setAlarm(uint8_t hour, uint8_t minute, uint8_t enabled) {
    // 实现闹钟设置逻辑
    
}