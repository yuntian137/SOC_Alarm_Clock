#include "UART_Communication_DMA.h"

static volatile FlagStatus UART_RxFlag = RESET, UART_TxFlag = RESET;
uint8_t UART_Communication_DMA_Data[200]; // 定义变量数组存放发送200个8位自定义数据
uint8_t UART_Communication_DMA_Buffer[200]; // 定义变量数组存放接收数据
static uint16_t UART_Communication_DMA_RxCount = 0; // 接收数据计数器

void UART_Communication_DMA(void)
{
    if (UART_RxFlag == SET) // 空闲中断触发
    {
        UART_RxFlag = RESET; // 清标志
        // 处理接收到的数据
        // 将接收到的数据复制到发送缓冲区
        memcpy(UART_Communication_DMA_Data, UART_Communication_DMA_Buffer, UART_Communication_DMA_RxCount);

        // 设置DMA0的当前数据计数器
        DMA_SetCurrDataCounter(DMA0, UART_Communication_DMA_RxCount);

        // 软件使能DMA0搬运
        // 检查接收到的数据长度是否为零
        if (UART_Communication_DMA_RxCount > 0)
        {
            // 设置DMA0的当前数据计数器
            DMA_SetCurrDataCounter(DMA0, UART_Communication_DMA_RxCount);

            // 软件使能DMA0搬运
            DMA_SoftwareTrigger(DMA0);
            while (DMA_GetFlagStatus(DMA0, DMA_FLAG_TCIF) == RESET); // 等待DMA0搬运完成
            DMA_ClearFlag(DMA0, DMA_FLAG_TCIF); // DMA0标志位清零
        }


        // DMA1重新使能
        DMA_SetCurrDataCounter(DMA1, 200); // 重新写入搬运个数（重新写入搬运个数大小应与修改的数据个数大小一致）
        UART_DMACmd(UART1, UART_DMAReq_RX, ENABLE); // 重新使能UART1 RX触发DMA1
    }
}
// UART接收中断处理函数
void UART1_IRQHandler_DMA(void)
{
    if (UART_GetFlagStatus(UART1, UART_Flag_RX) == SET)
    {
        UART_ClearFlag(UART1, UART_Flag_RX); // 清除UART接收标志
        // 重置定时器以检测空闲时间
        TIM_SetCounter(TIM3, 0);
    }
}


// 定时器中断处理函数
void TIM3_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM3, TIM_Flag_TI) == SET)
    {
        TIM_ClearFlag(TIM3, TIM_Flag_TI); // 清除定时器中断标志  
        if (UART_RxFlag == RESET) // 如果DMA1没有搬运完成
        {
            UART_RxFlag = SET; // 设置接收完成标志
            UART_Communication_DMA_RxCount = 200 - DMA_GetCurrDataCounter(DMA1); // 获取接收到的数据长度
        }
    }
}
/*
//DMA0中断 发送出去
void UART_Communication_DMA_DAM0Handler(void)
{
  if(DMA_GetFlagStatus(DMA0, DMA_FLAG_TCIF))//DMA0搬运完成
  {
    DMA_ClearFlag(DMA0, DMA_FLAG_TCIF);//DMA0标志位清零
    UART_TxFlag = SET;//DMA0搬运标志位置起，准备下一次搬运
  }
}

//DMA1中断 接收到
void UART_Communication_DMA_DAM1Handler(void)
{
  if(DMA_GetFlagStatus(DMA1, DMA_FLAG_TCIF))//DMA1搬运完成
  {
    DMA_ClearFlag(DMA1, DMA_FLAG_TCIF);//DMA1标志位清零
    UART_RxFlag = SET;//DMA1搬运标志位置起，准备下一次搬运
  }
}
*/
