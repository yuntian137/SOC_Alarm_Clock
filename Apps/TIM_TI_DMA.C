#include "TIM_TI_DMA.h"
#include "string.h"

static FlagStatus TIM_DMA_Flag = RESET;//自定义TIM_DMA标志位

uint8_t TIM_TI_DMA_SrcData[200], TIM_TI_DMA_DstData[200];//自定义源数据数组及目标数据数组存放相应数据

void TIM_TI_DMA(void)
{
  uint8_t i;

  for(i = 0; i < 200; i++)
  {
    TIM_TI_DMA_SrcData[i] = i;//被搬运数组写入数据
  }
  /* 源数据数组写入数据完毕 */

  /* 准备开始搬运源数组中的数据*/
  while(1)
  {
    if(TIM_DMA_Flag == SET)//自定义TIM_DMA标志位置起
    {
      /* TIM2定时器溢出触发DMA0搬运源数组“TIM_TI_DMA_SrcData[]”中的数据至目标搬运数组”TIM_TI_DMA_DstData[]“已搬运完成 */

      memset(TIM_TI_DMA_DstData, 0x00, 200);//将目标数据数组中的数据清零

      /* 开启下一轮DMA搬运 */
      DMA_SetCurrDataCounter(DMA0, 200);//重新写入DMA搬运数量

      /* 重新使能DMA */
      TIM_DMACmd(TIM2, TIM_DMAReq_TI, DISABLE);
      TIM_DMACmd(TIM2, TIM_DMAReq_TI, ENABLE);

      TIM_DMA_Flag = RESET;//自定义TIM_DMA标志位清零
    }
  }
}

void TIM_TI_DMA_DMA0Handler(void)
{
  if(DMA_GetFlagStatus(DMA0, DMA_FLAG_TCIF) == SET)//DMA0传输完成中断置起
  {
    DMA_ClearFlag(DMA0, DMA_FLAG_TCIF);//清除DMA0中断
    TIM_DMA_Flag = SET;//自定义TIM_DMA标志位置起
  }
}


