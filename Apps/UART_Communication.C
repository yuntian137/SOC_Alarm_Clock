#include "UART_Communication.h"
static volatile FlagStatus UART_RxFlag = RESET;

void UART_Communication(void)
{
  uint8_t i;
  volatile uint16_t UART_RxData[5];//发送5个8位自定义数据（若要修改数据个数需修改相应数组大小和发送/接收for循环大小）
   
  while(1)
  {
    /* UART2接收到5个8位数据 */
    for(i=0;i<5;i++)
    {
      while(UART_RxFlag == RESET);//等待接收完成
      UART_RxFlag = RESET;//接收标志位清零
      UART_RxData[i] = UART_ReceiveData(UART2);//接收到的数据存入数组
    }
    /* UART2将接收到的数据发送出去 */
    for(i=0;i<5;i++)
    {
      UART_SendData(UART2,UART_RxData[i]);//发送接收到的数据
      while(UART_GetFlagStatus(UART2,UART_Flag_TX) == RESET);//等待发送完成
      UART_ClearFlag(UART2,UART_Flag_TX);//发送标志位清零
    }
  }
}

//UART2中断
void UART_Communication_UART2Handler(void)
{
  if(UART_GetFlagStatus(UART2,UART_Flag_RX))//处于接收状态
  {
    UART_ClearFlag(UART2,UART_Flag_RX);//中断接收标志位清零
    UART_RxFlag = SET;//接收标志位置起
  }
}


