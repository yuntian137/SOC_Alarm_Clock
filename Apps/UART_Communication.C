#include "UART_Communication.h"
#include "string.h"

#define MAX_BUFFER_SIZE 100 // 定义最大接收和发送的数据大小

static volatile uint8_t UART_RxBuffer[MAX_BUFFER_SIZE]; // 接收数据缓冲区
static volatile uint8_t UART_TxBuffer[MAX_BUFFER_SIZE]; // 发送数据缓冲区
static volatile uint8_t RxIndex = 0; // 接收数据索引
static volatile FlagStatus UART_RxComplete = RESET; // 接收完成标志位

void UART_Communication_UART2Handler(void)
{
  while (UART_GetFlagStatus(UART2, UART_Flag_RX) == SET) // 接收中断
  {
    UART_ClearFlag(UART2, UART_Flag_RX); // 清除接收标志
    uint8_t receivedChar = UART_ReceiveData(UART2); // 接收数据

    UART_RxBuffer[RxIndex++] = receivedChar; // 接收数据并存入缓冲区

    if (RxIndex >= MAX_BUFFER_SIZE) // 防止缓冲区溢出
    {
      RxIndex = 0; // 重置接收索引
    }

    // 将接收到的数据复制到发送缓冲区
    memcpy((void *)UART_TxBuffer, (void *)UART_RxBuffer, RxIndex);

    // 发送数据
    for (uint8_t i = 0; i < RxIndex; i++)
    {
      UART_SendData(UART2, UART_TxBuffer[i]); // 发送数据
      while (UART_GetFlagStatus(UART2, UART_Flag_TX) == RESET); // 等待发送完成
      UART_ClearFlag(UART2, UART_Flag_TX); // 清除发送标志
    }

    RxIndex = 0; // 重置接收索引
  }
}

void UART_Handler(void)
{
    if (UART_GetFlagStatus(UART2, UART_Flag_RX) == SET) // 接收中断
    {
        UART_ClearFlag(UART2, UART_Flag_RX); // 清除接收标志
        uint8_t receivedChar = UART_ReceiveData(UART2); // 接收数据

        UART_RxBuffer[RxIndex++] = receivedChar; // 接收数据并存入缓冲区

        if (RxIndex >= MAX_BUFFER_SIZE) // 防止缓冲区溢出
        {
            RxIndex = 0; // 重置接收索引
        }

        // 检查是否接收到特定消息 "0xF1"
        if (RxIndex >= 1 && UART_RxBuffer[RxIndex - 1] == 0xF1)
        {
            // 发送特定格式的消息回去
            uint8_t response[] = {0xAA, 0x55, 0x01, 0x00, 0x28, 0x55, 0xAA};//播报是十进制
            for (uint8_t i = 0; i < sizeof(response); i++)
            {
                UART_SendData(UART2, response[i]); // 发送数据
                while (UART_GetFlagStatus(UART2, UART_Flag_TX) == RESET); // 等待发送完成
                UART_ClearFlag(UART2, UART_Flag_TX); // 清除发送标志
            }

            RxIndex = 0; // 重置接收索引
        }
    }
}
