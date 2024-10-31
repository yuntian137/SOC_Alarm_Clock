#include "TWI0_Master_Communication.h"

static volatile FlagStatus TWI0_Flag1 = RESET, TWI0_Flag2 = RESET;//自定义TWI0标志位1，2

uint8_t TWI0_SendDataArray[4] = {0x12, 0x34, 0x56, 0x78};//定义主机发送数据数组
uint8_t TWI0_ReceiveDataArray[4];//定义主机接收数据数组

void TWI0_Master_Communication(void)
{
  uint8_t i;

  while(1)
  {
    /* 主机发送 */
    /* 发送起始信号 */
    TWI_GenerateSTART(TWI0, ENABLE);

    //延时
    {
      uint32_t delay_time = 100;
      while(delay_time--);
    }

    while(TWI0_Flag1 == RESET);//等待起始信号发送完成
    TWI0_Flag1 = RESET;//自定义TWI0标志位1清零

    /* 发送地址 */
    TWI_Send7bitAddress(TWI0, 0x50, TWI_Command_Write);//从机地址为0x50
    while(TWI0_Flag1 == RESET);//等待地址发送完成
    TWI0_Flag1 = RESET;//自定义TWI0标志位1清零

    //延时
    {
      uint32_t delay_time = 100;
      while(delay_time--);
    }

    /* 主机发送地址帧（写），且收到从机的ACK */
    while(TWI0_Flag2 != SET);
    TWI0_Flag2 = RESET;//自定义TWI0标志位2清零

    /* 主机发送数据 */
    for(i = 0; i < 4; i++)
    {
      TWI_SendData(TWI0, TWI0_SendDataArray[i]);//对TWIDAT写数据，同时清TXnE/RXnE标志
      while(TWI0_Flag1 == RESET);//等待数据发送完成
      TWI0_Flag1 = RESET;//自定义TWI0标志位1清零
      //延时
      {
        uint32_t delay_time = 100;
        while(delay_time--);
      }
    }

    /* 发送停止信号 */
    TWI_GenerateSTOP(TWI0, ENABLE);

    /*TWI0主机发送数据成功！（主机向从机发送的数据为"TWI0_SendDataArray[]"数组中的数据）*/

    while(TWI_GetStateMachine(TWI0) != TWI_Master_Idle)
      ;//等待主机恢复为空闲状态

    //延时
    {
      uint32_t delay_time = 100;
      while(delay_time--);
    }

    /* 主机接收 */
    TWI_AcknowledgeConfig(TWI0, ENABLE);//使能Ack应答

    /* 发送起始信号 */
    TWI_GenerateSTART(TWI0, ENABLE);
    //延时
    {
      uint32_t delay_time = 100;
      while(delay_time--);
    }

    while(TWI0_Flag1 == RESET);//等待起始信号发送完成
    TWI0_Flag1 = RESET;//自定义TWI0标志位1清零

    /* 发送地址 */
    TWI_Send7bitAddress(TWI0, 0x50, TWI_Command_Read);//从机地址为0x50
    while(TWI0_Flag1 == RESET);//等待地址发送完成
    TWI0_Flag1 = RESET;//自定义TWI0标志位1清零

    //延时
    {
      uint32_t delay_time = 100;
      while(delay_time--);
    }

    /* 主机发送地址帧（读），且收到从机的ACK */
    while((TWI0_Flag2 != SET));
    TWI0_Flag2 = RESET;//自定义TWI0标志位2清零

    /* 主机接收数据 */
    for(i = 0; i < 4; i++)
    {
      while(TWI0_Flag1 != SET);//等待1byte接收完成
      TWI0_Flag1 = RESET;//自定义TWI0标志位1清零
      TWI0_ReceiveDataArray[i] = (unsigned char)TWI_ReceiveData(TWI0);
      if(i >= 2)
      {
        TWI_AcknowledgeConfig(TWI0, DISABLE);//倒数第二byte关闭Ack应答
      }
    }

    /* 发送停止信号 */
    TWI_GenerateSTOP(TWI0, ENABLE);

    /*TWI0主机接收数据成功！（接收到的从机数据会存入"TWI0_ReceiveDataArray[]"数组中）*/

    while(TWI_GetStateMachine(TWI0) != TWI_Master_Idle)
      ;//等待主机恢复为空闲状态

    //延时
    {
      uint32_t delay_time = 1000;
      while(delay_time--);
    }
  }
}

void TWI0_Master_Communication_TWI0Handler(void)
{
  if(TWI_GetFlagStatus(TWI0, TWI_FLAG_TWIF))//主机发送启动信号/发送完地址帧/接收或发送完数据帧
  {
    TWI_ClearFlag(TWI0, TWI_FLAG_TWIF);
    TWI0_Flag1 = SET;//自定义TWI0标志位1置起
    if(TWI_GetFlagStatus(TWI0, TWI_FLAG_TXRXnE))//主机发送地址帧（写），且收到从机的ACK；主机发送完数据，且接收到从机ACK；主机接收到数据，且主机回从机ACK
    {
      TWI0_Flag2 = SET;//自定义TWI0标志位2置起
    }
  }
}

