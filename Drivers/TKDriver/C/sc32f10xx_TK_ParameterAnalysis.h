//*************************************************************************************************
//  Copyright (c) 	��������Ԫ΢���ӹɷ����޹�˾
//	�ļ�����	:  SensorMethod.h
//	����		: 
//	ģ�鹦��	:  �����㷨ͷ�ļ�
// 	�汾	    :
// 	���ļ�¼	:
//	ע������	:    
//************************************************************************************************
#ifndef	_sc32f10xx_TK_FileAnalysis_H
#define	_sc32f10xx_TK_FileAnalysis_H

typedef struct
{
	unsigned short int	Rawdata;   //ԭʼ����
	unsigned short int	Baseline;  //����
	unsigned short int	FilterData;  //ԭʼ�����˲�������
	
}TK_BaseLineUpdatePar_StructDef;

typedef struct
{
	unsigned char	BaseLineAdjusttmp;   //IA��������
	
}TK_ParameterBuffer_StructDef;


extern TK_BaseLineUpdatePar_StructDef SingleChannelsBaseLineUpdatePar[];
extern TK_BaseLineUpdatePar_StructDef CombineChannelsBaseLineUpdatePar[];

extern TK_ParameterBuffer_StructDef SingleParameterBufferSet[] ;



#endif 
