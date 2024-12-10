#include "mqtt.h"
#include "UART_Communication_DMA.h"
#include <stdlib.h>
uint8_t wait_timeout;                       // 超时判断标志位，等待OK超过五秒置1
char dateTime[64];                          // 存储提取的日期和时间
int year, month, day, hour, minute, second; // 存储年、月、日、时、分、秒

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
    Get_Ntp_Time();
    if(Wifi_Init_OK == RESET) Wifi_Init_OK = SET;//初始化完成
}
FlagStatus Get_Init_Flag()
{
    return Wifi_Init_OK;
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
    char login_url[100] = {"AT+CIPSNTPCFG=1,8,\"iot.console.aliyun.com\"\r\n\r\n"};
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
    char user_connect_buf[200] = {"AT+MQTTUSERCFG=0,1,\"NULL\",\"SC32F10TS8&k23m6LAJ0V9\",\"C2B81F691F8AFC60347A715F0155D0D97FAFE2FC\",0,0,\"\"\r\n\r\n"};
    USART_TxStr(UART2,"用户设备连接...\r\n");
    USART_TxStr(UART1,user_connect_buf);
}

/*********************连接客户端*******************************/
void Client_Connect(void)
{
    char client_connect_buf[100] = {"AT+MQTTCLIENTID=0,\"12345|securemode=3\\,signmethod=hmacsha1|\"\r\n\r\n"};
    USART_TxStr(UART2,"连接客户端...\r\n");
    USART_TxStr(UART1,client_connect_buf);
}

/*********************连接阿里云服务器*************************/
void Connect_Aliyun_Server(void)
{
    char connect_server_buf[100] = {"AT+MQTTCONN=0,\"k23m6LAJ0V9.iot-as-mqtt.cn-shanghai.aliyuncs.com\",1883,1\r\n\r\n"};
    USART_TxStr(UART2,"连接阿里云服务器...\r\n");
    USART_TxStr(UART1,connect_server_buf);
}

/*********************客户端订阅*************************/
void Client_Subscribe(void)
{
    char client_subscribe_buf[100] = {"AT+MQTTSUB=0,\"/k23m6LAJ0V9/SC32F10TS8/user/Test\",1\r\n\r\n"};
    USART_TxStr(UART2,"客户端订阅消息...\r\n");
    USART_TxStr(UART1,client_subscribe_buf);
}

/*连接到NTP服务器*/
int Get_Ntp_Time()
{
    int time;
    //char ntp_connect_buf[100] = {"AT+CIPSNTPCFG=0,1,\"pool.ntp.org\"\r\n\r\n"};
    char ntp_connect_buf_CN[100] = {"AT+CIPSNTPCFG=1,8,\"cn.ntp.org.cn\",\"ntp.sjtu.edu.cn\"\r\n\r\n\r\n"};
    USART_TxStr(UART2, "连接NTP服务器...\r\n");
    USART_TxStr(UART1, ntp_connect_buf_CN);
    wait_OK();
    //Delay_ms(25000);//获取中国地区时间要在连接NTP服务器后等二十秒以上
    //Get_Time();//获取时间
    //wait_OK();
    return 0;
}
/*获取时间*/
int Get_Time(void)
{
    char ntp_time_buf[100] = {"AT+CIPSNTPTIME?\r\n\r\n"};
    USART_TxStr(UART2, "获取时间...\r\n");
    USART_TxStr(UART1, ntp_time_buf);
    Delay_ms(500);//调试用延时，等待搬运完
    // 提取日期和时间
    ExtractDateTime(UART_RxBuffer, dateTime, sizeof(dateTime));

    // 解析日期和时间
    ParseDateTime(dateTime, &year, &month, &day, &hour, &minute, &second);

    // 输出结果
    printf("Extracted DateTime: %s\n", dateTime);
    printf("Year: %d, Month: %d, Day: %d, Hour: %d, Minute: %d, Second: %d\n",
           year, month, day, hour, minute, second);
    return 0;
}
// 提取日期和时间的函数
void ExtractDateTime(uint8_t* buffer, char* dateTime, size_t maxLen) 
{
    const char* prefix = "+CIPSNTPTIME:";  // 前缀字符串
    size_t prefixLen = strlen(prefix);     // 前缀长度
    size_t bufferLen = sizeof(UART_RxBuffer) / sizeof(UART_RxBuffer[0]);  // 缓冲区长度

    // 遍历缓冲区，查找前缀
    for (size_t i = 0; i < bufferLen - prefixLen; i++) {
        if (memcmp(&buffer[i], prefix, prefixLen) == 0) {
            // 找到前缀，提取日期和时间
            size_t j = 0;
            while (i + prefixLen + j < bufferLen && buffer[i + prefixLen + j] != '\r' && j < maxLen - 1) {
                dateTime[j] = buffer[i + prefixLen + j];
                j++;
            }
            dateTime[j] = '\0';  // 字符串结尾
            return;
        }
    }

    // 如果未找到前缀，返回空字符串
    dateTime[0] = '\0';
}

// 解析日期和时间字符串，生成年、月、日、时、分、秒
void ParseDateTime(const char* dateTime, int* year, int* month, int* day, int* hour, int* minute, int* second) 
{
    // 使用 sscanf 解析日期和时间
    sscanf(dateTime, "%*s %*s %d %d:%d:%d %d", day, hour, minute, second, year);

    // 将月份字符串转换为数字
    char monthStr[4];  // 用于存储月份字符串（如 "Dec"）
    strncpy(monthStr, dateTime + 4, 3);  // 从第 5 个字符开始提取月份字符串
    monthStr[3] = '\0';  // 确保字符串以 '\0' 结尾

    if (strcmp(monthStr, "Jan") == 0) *month = 1;
    else if (strcmp(monthStr, "Feb") == 0) *month = 2;
    else if (strcmp(monthStr, "Mar") == 0) *month = 3;
    else if (strcmp(monthStr, "Apr") == 0) *month = 4;
    else if (strcmp(monthStr, "May") == 0) *month = 5;
    else if (strcmp(monthStr, "Jun") == 0) *month = 6;
    else if (strcmp(monthStr, "Jul") == 0) *month = 7;
    else if (strcmp(monthStr, "Aug") == 0) *month = 8;
    else if (strcmp(monthStr, "Sep") == 0) *month = 9;
    else if (strcmp(monthStr, "Oct") == 0) *month = 10;
    else if (strcmp(monthStr, "Nov") == 0) *month = 11;
    else if (strcmp(monthStr, "Dec") == 0) *month = 12;

    // 将星期几字符串转换为数字
    char weekDayStr[4];  // 用于存储星期几字符串（如 "Tue"）
    strncpy(weekDayStr, dateTime, 3);  // 提取前 3 个字符作为星期几
    weekDayStr[3] = '\0';  // 确保字符串以 '\0' 结尾

    if (strcmp(weekDayStr, "Mon") == 0) *day = 1;
    else if (strcmp(weekDayStr, "Tue") == 0) *day = 2;
    else if (strcmp(weekDayStr, "Wed") == 0) *day = 3;
    else if (strcmp(weekDayStr, "Thu") == 0) *day = 4;
    else if (strcmp(weekDayStr, "Fri") == 0) *day = 5;
    else if (strcmp(weekDayStr, "Sat") == 0) *day = 6;
    else if (strcmp(weekDayStr, "Sun") == 0) *day = 7;
}

void USART_TxStr(UART_TypeDef* uart, const char *str)
{
    // 根据传入的串口类型选择发送的串口
    if (uart == UART1)
    {
        // 计算字符串长度
        uint16_t length = strlen(str);

        // 使用DMA发送
        UART_DMA_Send((uint8_t*)str, length);
    }
    else if (uart == UART2)
    {
        while (*str)
        {
            UART_SendData(UART2, (uint8_t)*str); // 发送字符到UART2
            while (UART_GetFlagStatus(UART2, UART_Flag_TX) == RESET)
                ;                                // 等待发送完成
            UART_ClearFlag(UART2, UART_Flag_TX); // 清除发送完成标志
            str++;
        }
    }
}
//需要时开启轮询即可
uint8_t Get_Weather(void)
{
    if (Flag_weather_received == SET)
    {
        // 处理接收到的天气数据(注意天气格式为x#,其中x为char格式，读出来的weatherData是ASCII对应的十六进制数)
        uint8_t weatherData = UART_RxBuffer[RxIndex - 2]; // 获取天气数据（尾帧前一个字节）
        printf("Received Weather Data: 0x%02X\n", weatherData);

        // 重置标志位和接收索引
        Flag_weather_received = RESET;
        RxIndex = 0;
    }
    return 0;
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

        // 检查是否接收到 "OK"
        if (RxIndex >= 2 && UART_RxBuffer[RxIndex - 2] == 'O' && UART_RxBuffer[RxIndex - 1] == 'K')
        {
            Flag_usart2_receive_OK = SET; // 设置接收OK标志位
        }
        // 检查是否接收到天气数据的尾帧#
        if (receivedChar == WEATHER_FRAME_END)
        {
            Flag_weather_received = SET; // 设置天气数据接收标志位
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

