#include "UART_Communication_DMA.h"
#include "string.h"

static volatile FlagStatus UART_RxFlag = RESET, UART_TxFlag = RESET;
/*如果这俩用一个数组，DMA0会一直搬不完*/
uint8_t UART_Communication_DMA_Data[200]; // DMA0要发送回去给到UART用来发回上位机
uint8_t UART_Communication_DMA_Buffer[200]; // DMA1收到之后搬运到这里
static uint16_t UART_Communication_DMA_RxCount = 0; // 接收数据计数器

uint8_t a = 0;
uint8_t b = 0;
void UART_Communication_DMA(void)
{
  if (UART_RxFlag == SET) // 空闲中断触发
  {
    UART_RxFlag = RESET; // 清标志
    // 处理接收到的数据
    // 将接收到的数据复制到发送缓冲区
    memcpy(UART_Communication_DMA_Data, UART_Communication_DMA_Buffer, UART_Communication_DMA_RxCount);

    // 软件使能DMA0搬运
    // 检查接收到的数据长度是否为零
    if (UART_Communication_DMA_RxCount > 0)
    {
      UART_DMA_Send(UART_Communication_DMA_Data, UART_Communication_DMA_RxCount);
    }
    // 清空接收缓冲区
    memset(UART_Communication_DMA_Buffer, 0, sizeof(UART_Communication_DMA_Buffer));

    // DMA1重新使能
    DMA_SetCurrDataCounter(DMA1, 200); // 重新写入搬运个数 !!!!!!!!无效，必须重启

    DMA_Cmd(DMA1, DISABLE);
    DMA_Cmd(DMA1, ENABLE);
    a = DMA_GetCurrDataCounter(DMA1);           // 获取接收到的数据长度;
    UART_DMACmd(UART1, UART_DMAReq_RX, ENABLE); // 重新使能UART1 RX触发DMA1
  }
}

void UART_DMA_Send(uint8_t* data, uint16_t length)
{
    memcpy(UART_Communication_DMA_Data, data, length);
    // 设置DMA0的当前数据计数器
    DMA_SetCurrDataCounter(DMA0, length);

    // 软件使能DMA0搬运
    DMA_SoftwareTrigger(DMA0);

    // 等待DMA0搬运完成
    while (DMA_GetFlagStatus(DMA0, DMA_FLAG_TCIF) == RESET);

    // DMA0标志位清零
    DMA_ClearFlag(DMA0, DMA_FLAG_TCIF);
}

void UART_DMA_Receive(void)
{
    if (UART_RxFlag == SET) // 空闲中断触发
  {
    UART_RxFlag = RESET; // 清标志
    // 获取接收到的数据长度
    UART_Communication_DMA_RxCount = 200 - DMA_GetCurrDataCounter(DMA1);

    // 处理接收到的数据
    if (UART_Communication_DMA_RxCount > 0)
    {
        // 这里可以添加处理接收数据的代码
        // 例如：将接收到的数据复制到另一个缓冲区，或者进行其他处理
            // 将接收到的数据复制到发送缓冲区
        // memcpy(UART_Communication_DMA_Data, UART_Communication_DMA_Buffer, UART_Communication_DMA_RxCount);
        // UART_DMA_Send(UART_Communication_DMA_Data, UART_Communication_DMA_RxCount);
    }

    // 清空接收缓冲区
    memset(UART_Communication_DMA_Buffer, 0, sizeof(UART_Communication_DMA_Buffer));

    // DMA1重新使能
    DMA_SetCurrDataCounter(DMA1, 200); // 重新写入搬运个数
    /*重启DMA1，不然搬运个数写入不生效*/
    DMA_Cmd(DMA1, DISABLE);
    DMA_Cmd(DMA1, ENABLE);

    // 重新使能UART1 RX触发DMA1
    UART_DMACmd(UART1, UART_DMAReq_RX, ENABLE);
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
    TIM_Cmd(TIM3, ENABLE); // 开启定时器
  }
}
// 定时器中断处理函数
void TIM3_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM3, TIM_Flag_TI) == SET)
    {
        TIM_ClearFlag(TIM3, TIM_Flag_TI); // 清除定时器中断标志  
        TIM_Cmd(TIM3, DISABLE); // 关闭定时器
        if (UART_RxFlag == RESET) // 如果DMA1没有搬运完成
        {
            UART_RxFlag = SET; // 设置接收完成标志
            UART_DMA_Receive();//调用接收处理函数
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
