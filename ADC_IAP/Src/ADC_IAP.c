#include "adc_iap.h"
#include "sc32f1xxx_iap.h"
#include "sc32f1xxx_adc.h"

// ADC配置
#define ADC_CHANNEL ADC_Channel_13  // 假设使用ADC通道0
#define VOLTAGE_THRESHOLD 4.5  // 掉电检测阈值电压
#define ADC_RESOLUTION 16383  // 14位ADC的满量程值
#define REFERENCE_VOLTAGE 5.0  // 参考电压为5V

// IAP配置
#define FLASH_USER_ADDR  0x801FE00//定义IAP操作目标地址
// 读取ADC值
uint16_t readADC(ADC_ChannelTypedef channel) {
    if (channel != 0)
    {
        ADC_SetChannel(ADC, channel);
    } 
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
    IAP_Unlock();//解锁IAP操作
    eraseFlashPage(FLASH_USER_ADDR);
    IAP_WriteCmd(ENABLE);//开启写使能
}

// 擦除Flash页
void eraseFlashPage(uint32_t address) {
    IAP_EraseSector((address - FLASH_BASE) / 512);//擦除地址所在扇区，每个扇区大小为512Byte
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
    initIAP();//准备IAP
    writeFlash(FLASH_USER_ADDR, (uint8_t*)&settings, sizeof(settings));//开写
    IAP_Lock();//上锁IAP操作，并且复位IAP操作寄存器
}

// 读取闹钟设置
void loadAlarmSettings(void) {
    AlarmSettings settings;
    readFlash(FLASH_USER_ADDR, (uint8_t*)&settings, sizeof(settings));

    setAlarm(settings.hour, settings.minute, settings.enabled);
}

// 设置闹钟
void setAlarm(uint8_t hour, uint8_t minute, uint8_t enabled) {
    // 实现闹钟设置逻辑
    
}
