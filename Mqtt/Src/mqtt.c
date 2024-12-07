#include "mqtt.h"

uint8_t wait_timeout;//超时判断标志位，等待OK超过五秒置1
void WIFI_Init(void)
{
    WIFI_Rst();
    Setting_Connect_Work("1");
    wait_OK();

    Login_URL();
    wait_OK();

    WIFI_Connect("\"scy1\"", "\"123456789\"");
    wait_OK();

    USER_Connect();
    wait_OK();

    Client_Connect();
    wait_OK();

    Connect_Aliyun_Server();
    wait_OK();

    Client_Subscribe();
    wait_OK();
}
/*********************重启WIFI模块*****************************/
/*********************重启WIFI模块*****************************/
void WIFI_Rst(void)
{
    char rst_buff[11] = {"AT+RST\r\n\r\n"};
    USART_TxStr(UART2,"重启模块...\r\n");
    USART_TxStr(UART1,rst_buff);  
    Delay_ms(1000); // WIFI重启需要时间
    Delay_ms(1000);
    Delay_ms(1000);
    Delay_ms(1000);
    Delay_ms(1000);
    Delay_ms(1000);
    CLR_Buf2(); // 清除串口接收缓存
    Flag_usart2_receive_OK = RESET;
}

/*******************设置WIFI工作模式****************************
/
/			1 STA模式
/			2 AP模式
/			3 STA和AP模式
/
**************************************************************/
void Setting_Connect_Work(char *type)
{
    char wifi_mode_buf[13] = {"AT+CWMODE=x\r\n"};
    wifi_mode_buf[10] = *type;
    USART_TxStr(UART2,"设置工作模式...\r\n");
    USART_TxStr(UART1,wifi_mode_buf);
}

/*********************登录网址******************************/
void Login_URL(void)
{
    char login_url[100] = {"AT+CIPSNTPCFG=1,8,\"iot.console.aliyun.com\"\r\n"};
    USART_TxStr(UART2,"连接阿里云服务器...\r\n");
    USART_TxStr(UART1,login_url);
}

/*********************连接WIFI******************************/
void WIFI_Connect(char *name, char *password)
{
    char wifi_connect_buf[100] = {"AT+CWJAP="};
    strcat(wifi_connect_buf, name);
    strcat(wifi_connect_buf, ",");
    strcat(wifi_connect_buf, password);
    strcat(wifi_connect_buf, "\r\n");
    USART_TxStr(UART2,"连接已知WiFi...\r\n");
    USART_TxStr(UART1,wifi_connect_buf);
}

/*********************用户设备连接*******************************/
void USER_Connect(void)
{
    char user_connect_buf[200] = {"AT+MQTTUSERCFG=0,1,\"NULL\",\"SC32F10TS8&k23m6LAJ0V9\",\"C2B81F691F8AFC60347A715F0155D0D97FAFE2FC\",0,0,\"\"\r\n"};
    USART_TxStr(UART2,"用户设备连接...\r\n");
    USART_TxStr(UART1,user_connect_buf);
}

/*********************连接客户端*******************************/
void Client_Connect(void)
{
    char client_connect_buf[100] = {"AT+MQTTCLIENTID=0,\"12345|securemode=3\\,signmethod=hmacsha1|\"\r\n"};
    USART_TxStr(UART2,"连接客户端...\r\n");
    USART_TxStr(UART1,client_connect_buf);
}

/*********************连接阿里云服务器*************************/
void Connect_Aliyun_Server(void)
{
    char connect_server_buf[100] = {"AT+MQTTCONN=0,\"k23m6LAJ0V9.iot-as-mqtt.cn-shanghai.aliyuncs.com\",1883,1\r\n"};
    USART_TxStr(UART2,"连接阿里云服务器...\r\n");
    USART_TxStr(UART1,connect_server_buf);
}

/*********************客户端订阅*************************/
void Client_Subscribe(void)
{
    char client_subscribe_buf[100] = {"AT+MQTTSUB=0,\"/k23m6LAJ0V9/SC32F10TS8/user/Test\",1\r\n"};
    USART_TxStr(UART2,"客户端订阅消息...\r\n");
    USART_TxStr(UART1,client_subscribe_buf);
}


void USART_TxStr(UART_TypeDef* uart, const char *str)
{
    while (*str)
    {
        // 根据传入的串口类型选择发送的串口
        if (uart == UART1) {
            UART_SendData(UART1, (uint8_t)*str); // 发送字符到UART1
            while (UART_GetFlagStatus(UART1, UART_Flag_TX) == RESET); // 等待发送完成
            UART_ClearFlag(UART1, UART_Flag_TX); // 清除发送完成标志
        } else if (uart == UART2) {
            UART_SendData(UART2, (uint8_t)*str); // 发送字符到UART2
            while (UART_GetFlagStatus(UART2, UART_Flag_TX) == RESET); // 等待发送完成
            UART_ClearFlag(UART2, UART_Flag_TX); // 清除发送完成标志
        }
        str++;
    }
}


void CLR_Buf2(void)
{
    memset((void *)UART_RxBuffer, 0, MAX_BUFFER_SIZE); // 清除接收缓存
    RxIndex = 0; // 重置接收索引
}

void UART1_Handler(void)
{
    while (UART_GetFlagStatus(UART1, UART_Flag_RX) == SET) // 接收中断收完再处理
    {
        UART_ClearFlag(UART1, UART_Flag_RX);            // 清除接收标志
        uint8_t receivedChar = UART_ReceiveData(UART1); // 接收数据

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
            while (UART_GetFlagStatus(UART2, UART_Flag_TX) == RESET)
                ;                                // 等待发送完成
            UART_ClearFlag(UART2, UART_Flag_TX); // 清除发送标志
        }

        // 检查是否接收到 "OK"
        if (RxIndex >= 2 && UART_RxBuffer[RxIndex - 2] == 'O' && UART_RxBuffer[RxIndex - 1] == 'K')
        {
            Flag_usart2_receive_OK = SET; // 设置接收OK标志位
        }
    }
}

void wait_OK(void)
{
    uint32_t timeout = 5000000; // 设置超时时间为5000ms
    uint32_t elapsed_time = 0; // 已经过的时间
    while (!Flag_usart2_receive_OK) // 等待接收到OK字符串
    {
        Delay_us(100); // 每次循环延时100微秒
        elapsed_time += 100; // 更新已过时间

        if (elapsed_time >= timeout) // 检查是否超时
        {
            // 超时处理
            wait_timeout = 1;
            USART_TxStr(UART2,"超时！！！\r\n");
            break; // 跳出循环
        }
    }
    Flag_usart2_receive_OK = RESET; // 清除接收OK标志位
    CLR_Buf2(); // 清除串口接收缓存
}

