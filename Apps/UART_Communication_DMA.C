#include "UART_Communication_DMA.h"

static volatile FlagStatus UART_RxFlag = RESET, UART_TxFlag = RESET;//自定义DMA1、DMA0搬运标志位
uint8_t UART_Communication_DMA_Data[200];//定义变量数组存放发送200个8位自定义数据（若要修改数据个数需修改相应数组大小和DMA搬运个数，DMA0、DMA1初始化配置窗口配置项“缓存大小”应与此项大小一致）

void UART_Communication_DMA(void)
{
  while(1)
  {
    if(UART_RxFlag == SET)//DMA1搬运完成
    {
      UART_RxFlag = RESET;//清标志
      /* DMA1重新使能 */
      DMA_SetCurrDataCounter(DMA0, 200);//重新写入搬运个数（重新写入搬运个数大小应与修改的数据个数大小一致）
      UART_DMACmd(UART1, UART_DMAReq_TX, ENABLE);//重新使能UART1 TX触发DMA1
      /* 软件使能DMA0搬运 */
      DMA_SoftwareTrigger(DMA0);
      while(UART_TxFlag == RESET);//等待DMA0搬运完成
      UART_TxFlag = RESET;//清标志
    }
  }
}

//DMA0中断
void UART_Communication_DMA_DAM0Handler(void)
{
  if(DMA_GetFlagStatus(DMA0, DMA_FLAG_TCIF))//DMA0搬运完成
  {
    DMA_ClearFlag(DMA0, DMA_FLAG_TCIF);//DMA0标志位清零
    UART_TxFlag = SET;//DMA0搬运标志位置起，准备下一次搬运
  }
}

//DMA1中断
void UART_Communication_DMA_DAM1Handler(void)
{
  if(DMA_GetFlagStatus(DMA1, DMA_FLAG_TCIF))//DMA1搬运完成
  {
    DMA_ClearFlag(DMA1, DMA_FLAG_TCIF);//DMA1标志位清零
    UART_RxFlag = SET;//DMA1搬运标志位置起，准备下一次搬运
  }
}

