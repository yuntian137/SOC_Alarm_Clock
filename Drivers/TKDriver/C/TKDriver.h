//*************************************************************************************************
//  Copyright (c) 	��������Ԫ΢���ӹɷ����޹�˾
//	�ļ�����	:  SensorMethod.h
//	����		: 
//	ģ�鹦��	:  �����㷨ͷ�ļ�
// 	�汾	    :
// 	���ļ�¼	:
//	ע������	:    
//************************************************************************************************
#ifndef	_SENSORMETHOD_C_H
#define	_SENSORMETHOD_C_H

#include "sc32f1xxx.h"
#include "SC32F1xxx_rcc.h"

typedef	enum
{
	Circle = 0,		//����
	Bar = 1	,		//����
	wheel = 0,
    slider =1
}TKSlideModuleType;

typedef struct
{
	TKSlideModuleType	TypeFlag;		//(���޸�)		����������,Bar:Ϊ������Circle:Ϊ����
	uint8_t  	TKChannel[16];	 		//(���޸�)		���ջ���˳�����ν�ʹ�õ��Ļ�������ͨ����������6->5->4->3����ֵ�𽥱��
	uint8_t 	UsingTKChannelNumber;	//(���޸�)		��ǰ����ģ��ʹ�õ�TKͨ������
	uint16_t 	SideLevel;	            //(���޸�)		��������ĵ�λ���������1����
	
	int16_t		SUBData;		        //(���޸�)		DDifferֵ<SUB_DATA��Differֵ��Ч��������Ϊ0�����θ��ţ��޸������ȣ�  ��ֵ����Ĭ�� 
	int16_t 	LIBData;		        //Differֵ>LIB_DATA��ʾ�м����޸������ȣ���ֵ��������Ϊ��ֵ���ķ�֮һ������֮һ
	
	uint8_t 	TKOrderChannel[16];		//(�����޸�)		����ڲ�TKͨ�����򶼵�index
	uint8_t 	MAXUpdateCount;			//(�����޸�)		ɨ��������޻������������»��ߣ�Ĭ��10   ���޸�
	uint16_t    LastOutValue;			//(�����޸�)		��¼��һ�������ֵ
	uint16_t 	OutValue;				//(�����޸�)		�������ֵ��ͨ����ֵ��ȡ������
	uint16_t  	UpdateBaseLineNumber;	//(�����޸�)		���߸��¼���
	uint16_t	CouplingValue;			//(�����޸�)		��ϲ�����Χ(100~200)��������С����ֳ�ֵȱʧ,���绬����ֵOutValueΪ1->2->3->5->6->7,�����������ֳ�ֵ�ص�,���绬����ֵOutValueΪ1->2->3->4->3->4->5->6->7
	uint16_t	DebugCouplingValue;		//(�����޸�)		��ϲ�����DeBugֵ,���ڵ��ԣ�����ʱ�����ֵ����������ģ��ȡ�������Сֵ��������Сֵ����CouplingValue��ϲ���
	uint16_t	TriggerFlagCount;		//(�����޸�)		����ģ�鴥��������־
	
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
#define SCD_Wheel 0 //SCD_Wheel 
#define SCD_Slider 0 //SCD_Slider 
#define SCD_Key 1 //SCD_Key 

extern uint8_t    TK_TouchKeyStatus;  //����ɨ����ɱ�־λ
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
