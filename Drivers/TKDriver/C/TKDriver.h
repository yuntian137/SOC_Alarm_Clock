//*************************************************************************************************
//  Copyright (c) 	深圳市赛元微电子股份有限公司
//	文件名称	:  SensorMethod.h
//	作者		: 
//	模块功能	:  触控算法头文件
// 	版本	    :
// 	更改记录	:
//	注意事项	:    
//************************************************************************************************
#ifndef	_SENSORMETHOD_C_H
#define	_SENSORMETHOD_C_H

#include "sc32f1xxx.h"
#include "SC32F1xxx_rcc.h"

typedef	enum
{
	Circle = 0,		//滑轮
	Bar = 1	,		//滑条
	wheel = 0,
    slider =1
}TKSlideModuleType;

typedef struct
{
	TKSlideModuleType	TypeFlag;		//(需修改)		滑动块类型,Bar:为滑条，Circle:为滑轮
	uint8_t  	TKChannel[16];	 		//(需修改)		按照滑动顺序，依次将使用到的滑条按键通道存入数组6->5->4->3滑动值逐渐变大
	uint8_t 	UsingTKChannelNumber;	//(需修改)		当前滑动模块使用的TK通道数量
	uint16_t 	SideLevel;	            //(需修改)		滑动输出的档位最大级数（从1算起）
	
	int16_t		SUBData;		        //(可修改)		DDiffer值<SUB_DATA该Differ值无效，会设置为0，屏蔽干扰（修改灵敏度）  该值建议默认 
	int16_t 	LIBData;		        //Differ值>LIB_DATA表示有键（修改灵敏度）该值建议设置为阈值的四分之一到三分之一
	
	uint8_t 	TKOrderChannel[16];		//(无需修改)		存放内部TK通道排序都的index
	uint8_t 	MAXUpdateCount;			//(无需修改)		扫描多少轮无滑条按键即更新基线，默认10   可修改
	uint16_t    LastOutValue;			//(无需修改)		记录上一次输出的值
	uint16_t 	OutValue;				//(无需修改)		滑动输出值，通过该值获取输出结果
	uint16_t  	UpdateBaseLineNumber;	//(无需修改)		基线更新计数
	uint16_t	CouplingValue;			//(无需修改)		耦合参数范围(100~200)，参数过小会出现出值缺失,例如滑动出值OutValue为1->2->3->5->6->7,参数过大会出现出值重叠,例如滑动出值OutValue为1->2->3->4->3->4->5->6->7
	uint16_t	DebugCouplingValue;		//(无需修改)		耦合参数，DeBug值,用于调试，调试时输出该值，滑动滑动模块取输出的最小值，根据最小值调节CouplingValue耦合参数
	uint16_t	TriggerFlagCount;		//(无需修改)		滑动模块触发计数标志
	
}TKSlideModulePCB;

//<<<Use SPTML>>>
#define USING_TKSlideModule_Number_Set 1 //USING_TKSlideModule_Number_Set 
#define TypeFlag0 wheel //TypeFlag0 
#define UsingTKChannelNumber0 0 //UsingTKChannelNumber0 
#define SideLevel0 0 //SideLevel0 
#define TKChannel0 0 //TKChannel0 
#define LIBData0 0 //LIBData0 
#define TypeFlag1 wheel //TypeFlag1 
#define UsingTKChannelNumber1 0 //UsingTKChannelNumber1 
#define SideLevel1 0 //SideLevel1 
#define TKChannel1 0 //TKChannel1 
#define LIBData1 0 //LIBData1 
#define TypeFlag2 wheel //TypeFlag2 
#define UsingTKChannelNumber2 0 //UsingTKChannelNumber2 
#define SideLevel2 0 //SideLevel2 
#define TKChannel2 0 //TKChannel2 
#define LIBData2 0 //LIBData2 
#define TypeFlag3 wheel //TypeFlag3 
#define UsingTKChannelNumber3 0 //UsingTKChannelNumber3 
#define SideLevel3 0 //SideLevel3 
#define TKChannel3 0 //TKChannel3 
#define LIBData3 0 //LIBData3 
#define SCD_TK_Type 1 //SCD_TK_Type 
#define SCD_Wheel 1 //SCD_Wheel 
#define SCD_Slider 1 //SCD_Slider 
#define SCD_Key 1 //SCD_Key 

extern uint8_t    TK_TouchKeyStatus;  //按键扫描完成标志位
extern  TKSlideModulePCB  TKSlideModulePCBArray[];

extern void   TK_Init(void);
extern void   TK_Restart(void);
extern uint32_t  TK_TouchKeyScan(void);
//<<<end of SPTML>>>



extern uint8_t	 	ConfirmTouchCnt;

extern uint8_t   	RestAreaCnt[]; 
extern uint8_t   	TouchCnt[];				
extern uint8_t   	NoTouchCnt[];
extern uint8_t		CurrentChannel[];
extern uint8_t   	LowFingerDataCnt[];
extern int16_t   	DifferAccum[];
extern uint8_t	 	FloatAreaCnt[]; 
extern uint8_t	 	MultipleDealTpye;
extern int8_t		SetNoiseThreshold;

#endif 
