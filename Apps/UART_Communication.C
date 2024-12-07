#include "UART_Communication.h"
#include "string.h"

#define MAX_BUFFER_SIZE 100 // 定义最大接收和发送的数据大小

static volatile uint8_t UART_RxBuffer[MAX_BUFFER_SIZE]; // 接收数据缓冲区
static volatile uint8_t UART_TxBuffer[MAX_BUFFER_SIZE]; // 发送数据缓冲区
static volatile uint8_t RxIndex = 0; // 接收数据索引
static volatile FlagStatus UART_RxComplete = RESET; // 接收完成标志位

/**
 * @brief 上位机发送返回
 * 
 */
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


/**
 * @brief 语音模块识别
 * @note 只有0xF1问时间的消息需要发回给语音模块
 */
void UART2_Handler(void)
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

        // 检查是否接收到特定消息
        if (RxIndex >= 1)
        {
          uint8_t response[7]; // 定义响应数组
          response[0] = 0xAA;  // 固定头帧
          response[1] = 0x55;  // 固定头帧

          switch (UART_RxBuffer[RxIndex - 1])
          {
          case 0xF1://返回时间播报
            response[2] = 0x01;
            response[3] = 0x00;
            response[4] = 0x28; // F1对应的中间数据
            break;
          case 0xF2://开灯
            GPIO_ResetBits(GPIOC, GPIO_Pin_4);  // LED1

            break;
          case 0xF3:                            // 关灯
            GPIO_ResetBits(GPIOC, GPIO_Pin_10); // LED3
            break;
          case 0xF4:                            // 提高亮度
            GPIO_ResetBits(GPIOC, GPIO_Pin_11); // LED4
            break;
          case 0xF5:                           // 降低亮度
            GPIO_ResetBits(GPIOA, GPIO_Pin_7); // LED5
            break;
          case 0xF6:                           // 设置闹钟时间
            GPIO_ResetBits(GPIOA, GPIO_Pin_8); // LED6
            break;
          case 0xF7:                           // 关闹钟/停止响铃
            GPIO_ResetBits(GPIOC, GPIO_Pin_5); // LED2
            break;
          case 0xF8: // 切歌（下一首上一首均可）

            break;
          default:
            return; // 如果没有匹配的消息，直接返回
          }

          response[5] = 0x55; // 固定尾帧
          response[6] = 0xAA; // 固定尾帧

          // 发送响应
          for (uint8_t i = 0; i < sizeof(response); i++)
          {
            UART_SendData(UART2, response[i]); // 发送数据
            while (UART_GetFlagStatus(UART2, UART_Flag_TX) == RESET)
              ;                                  // 等待发送完成
            UART_ClearFlag(UART2, UART_Flag_TX); // 清除发送标志
          }

          RxIndex = 0; // 重置接收索引
        }
    }
}
