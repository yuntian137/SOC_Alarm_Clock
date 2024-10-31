#include "ADC_Conversion.h"
static volatile FlagStatus ADC_Flag;//自定义标志位

void ADC_Conversion(void)
{
  //ADC示例1：ADC单次采集数据示例

  static volatile uint16_t ADC_Value;//定义变量存储ADC值
  //ADC单次转化
  ADC_SoftwareStartConv(ADC);//软件触发ADC
  while(ADC_Flag == RESET);//等待ADC转换结束
  ADC_Flag = RESET;
  ADC_Value = ADC_GetConversionValue(ADC);//获取一次ADC转换值
  if((ADC_Value > 100) && (ADC_Value <= 500))
  {
    //对采集ADC数据进行处理
  }

  //ADC示例2：ADC采集十次数据滤波取平均值
  /*
    static volatile uint16_t ADC_AverageValue;//定义变量存储ADC平均值
    //ADC多次转化求10次平均值
    ADC_AverageValue = ADC_GetAverageValue();
  */
}

void ADC_Conversion_IRQ(void)
{
  if(ADC_GetFlagStatus(ADC, ADC_Flag_ADCIF))//判断ADC标志位
  {
    ADC_ClearFlag(ADC, ADC_Flag_ADCIF);//清除ADC标志位
    ADC_Flag = SET;//自定义标志位置起
  }
}

//ADC求10次转化平均值
uint16_t ADC_GetAverageValue(void)
{
  uint16_t i;
  uint16_t ADC_ValueMax = 0, ADC_ValueMin = 0X01 << 14, ADC_ValueMean = 0, ADC_ValueTad = 0;//定义变量ADC最大值、最小值、平均值、单次临时采样值
  uint32_t ADC_ValueSum = 0;//定义变量ADC多次采样累加值
  for(i = 0; i < 10; i++)
  {
    ADC_SoftwareStartConv(ADC);//软件触发ADC
    while(ADC_Flag == RESET);//等待ADC中断触发
    ADC_Flag = RESET;
    ADC_ValueTad = ADC_GetConversionValue(ADC);//获取一次ADC采样值
    if(ADC_ValueTad > ADC_ValueMax)
    {
      ADC_ValueMax = ADC_ValueTad;//获取ADC最大值
    }
    if(ADC_ValueTad < ADC_ValueMin)
    {
      ADC_ValueMin = ADC_ValueTad;//获取ADC最小值
    }
    ADC_ValueSum += ADC_ValueTad;//ADC采样值累加
  }
  ADC_ValueMean = (uint16_t)((ADC_ValueSum - ADC_ValueMax - ADC_ValueMin) / 8);//ADC滤波取平均值
  return ADC_ValueMean;
}

