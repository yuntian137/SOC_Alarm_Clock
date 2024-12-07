#ifndef __MQTT_H
#define __MQTT_H 			   
#include "sc32_conf.h"
#include <string.h>
#include <stdio.h>
#include "Delay.h"
#define MAX_BUFFER_SIZE 100 // 定义最大接收和发送的数据大小

static volatile uint8_t UART_RxBuffer[MAX_BUFFER_SIZE]; // 接收数据缓冲区
static volatile uint8_t UART_TxBuffer[MAX_BUFFER_SIZE]; // 发送数据缓冲区
static volatile uint8_t RxIndex = 0; // 接收数据索引
static volatile FlagStatus UART_RxComplete = RESET; // 接收完成标志位
static volatile FlagStatus Flag_usart2_receive_OK = RESET; // 接收OK标志位

#define User_ESP8266_SSID     "scy1"    		//wifi名字
#define User_ESP8266_PWD      "123456789"      	//wifi密码


#define User_ESP8266_client_id    	"12345|securemode=3\\,signmethod=hmacsha1|"   	//MQTTclientID 用于标志client身份  最长256字节
#define User_ESP8266_username     	"SC32F10TS8&k23m6LAJ0V9"										//用于登录 MQTT 服务器 的 username, 最长 64 字节	
#define User_ESP8266_password		"C2B81F691F8AFC60347A715F0155D0D97FAFE2FC"          			//用于登录 MQTT 服务器 的 password, 最长 64 字节
#define User_ESP8266_MQTTServer_IP     "k23m6LAJ0V9.iot-as-mqtt.cn-shanghai.aliyuncs.com"     		//MQTT本地服务器IP或者域名
#define User_ESP8266_MQTTServer_PORT   1883     													//服务器端口号


//自定义Topic
#define User_ESP8266_MQTTServer_Topic  						"/k23m6LAJ0V9/SC32F10TS8/user/Test"  	

void USART_TxStr(UART_TypeDef* uart, const char *str);
void CLR_Buf2(void);
void WIFI_Init(void);
void WIFI_Rst(void);
void Setting_Connect_Work(char *type);
void Login_URL(void);
void WIFI_Connect(char *name,char *password);
void USER_Connect(void);
void Client_Connect(void);
void Connect_Aliyun_Server(void);
void Client_Subscribe(void);
void wait_OK(void);

#endif

