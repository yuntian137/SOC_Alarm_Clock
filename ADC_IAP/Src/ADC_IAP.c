#include "adc_iap.h"
#include "sc32f1xxx_iap.h"
#include "sc32f1xxx_adc.h"

// ADC配置
#define ADC_CHANNEL ADC_Channel_VDD_D4  // 假设使用ADC通道0
#define VOLTAGE_THRESHOLD 4.0  // 掉电检测阈值电压
#define ADC_RESOLUTION 16383  // 14位ADC的满量程值
#define REFERENCE_VOLTAGE 2.4  // 参考电压为5V

// IAP配置
#define FLASH_USER_ADDR  0x801FE00//定义IAP操作目标地址
// 读取ADC值
uint16_t readADC(ADC_ChannelTypedef channel) {
    if (channel != 0)
    {
        ADC_SetChannel(ADC, channel);
    } 
    while (ADC_GetFlagStatus(ADC, ADC_Flag_ADCIF) == RESET);
    ADC_ClearFlag(ADC, ADC_Flag_ADCIF);
    return ADC_GetConversionValue(ADC);
}

// 将ADC值转换为电压
float adcToVoltage(uint16_t adcValue) {
    return (adcValue * REFERENCE_VOLTAGE) / ADC_RESOLUTION;
}

// 检查电源电压是否低于阈值
void checkPowerLoss(uint8_t cnt) {
    uint16_t adcValue = readADC(ADC_CHANNEL);
    float voltage = adcToVoltage(adcValue);
    
    if (voltage < VOLTAGE_THRESHOLD) //如果电压低于1.0V
    {
        //saveAlarmSettings(cnt);//存数据
        initIAP();
        while(1);
    }
    //saveAlarmSettings(cnt);
    //initIAP();
    
}

// IAP初始化
void initIAP(void) {
    // 配置IAP模块
    IAP_Unlock();//解锁IAP操作
    IAP_EraseSector((FLASH_USER_ADDR - FLASH_BASE) / 512);//擦除地址所在扇区，每个扇区大小为512Byte
   
    //eraseFlashPage(FLASH_USER_ADDR);
    IAP_WriteCmd(ENABLE);//开启写使能
    IAP_ProgramWord(FLASH_USER_ADDR, 0x12345678);//字写入，目标地址FLASH_USER_ADDR
    IAP_Lock();//上锁IAP操作，并且复位IAP操作寄存器
}

// 擦除Flash页
void eraseFlashPage(uint32_t address) {
    IAP_EraseSector((address - FLASH_BASE) / 512);//擦除地址所在扇区，每个扇区大小为512Byte
    //IAP_EraseSector(255);
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
void saveAlarmSettings(uint8_t cnt) {
    AlarmSettings settings_IN = {
        .hour = 7,
        .minute = 30,
        .enabled = 1
    };
    initIAP();//准备IAP
    writeFlash(FLASH_USER_ADDR, (uint8_t*)&settings_IN, sizeof(settings_IN));//开写
    //IAP_ProgramByte(FLASH_USER_ADDR, 0x12233);
    IAP_Lock();//上锁IAP操作，并且复位IAP操作寄存器
    //loadAlarmSettings();
}

// 读取闹钟设置
AlarmSettings loadAlarmSettings(void) {
    
    uint32_t ReadWord = 0;//定义变量存放单字读数据
    AlarmSettings settings;
    //readFlash(FLASH_USER_ADDR, (uint8_t*)&settings, sizeof(settings));
    ReadWord = IAP_ReadWord(FLASH_USER_ADDR);//字读，目标地址（0x803FE00）
        // 实现闹钟设置逻辑
        // 先关灯
        GPIO_SetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11); // 先关闭所有 LED
        GPIO_SetBits(GPIOA, GPIO_Pin_7 | GPIO_Pin_8);                             // 先关闭所有 LED
  
    // 根据 cnt 的值点亮相应数量的 LED
    if (ReadWord >= 1) 
        GPIO_ResetBits(GPIOC, GPIO_Pin_4); // LED1
    if (ReadWord >= 2)
        GPIO_ResetBits(GPIOC, GPIO_Pin_5); // LED2
    if (ReadWord >= 3)
        GPIO_ResetBits(GPIOC, GPIO_Pin_10); // LED3
    if (ReadWord >= 4)
        GPIO_ResetBits(GPIOC, GPIO_Pin_11); // LED4
    if (ReadWord >= 5)
        GPIO_ResetBits(GPIOA, GPIO_Pin_7); // LED5
    if (ReadWord >= 6)
        GPIO_ResetBits(GPIOA, GPIO_Pin_8); // LED6
    //setAlarm((uint8_t)ReadWord, settings.minute, settings.enabled);
    return settings;
}

uint32_t cnt01;
// 设置闹钟
void setAlarm(uint8_t hour, uint8_t minute, uint8_t enabled) {
    // 实现闹钟设置逻辑
        // 先关灯
        GPIO_SetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11); // 先关闭所有 LED
        GPIO_SetBits(GPIOA, GPIO_Pin_7 | GPIO_Pin_8);                             // 先关闭所有 LED
  
    // 根据 cnt 的值点亮相应数量的 LED
    if (hour >= 1 || minute >= 1|| enabled >= 1) 
        GPIO_ResetBits(GPIOC, GPIO_Pin_4); // LED1
    if (hour >= 2)
        GPIO_ResetBits(GPIOC, GPIO_Pin_5); // LED2
    if (hour >= 3)
        GPIO_ResetBits(GPIOC, GPIO_Pin_10); // LED3
    if (hour >= 4)
        GPIO_ResetBits(GPIOC, GPIO_Pin_11); // LED4
    if (hour >= 5)
        GPIO_ResetBits(GPIOA, GPIO_Pin_7); // LED5
    if (hour >= 6)
        GPIO_ResetBits(GPIOA, GPIO_Pin_8); // LED6
}
