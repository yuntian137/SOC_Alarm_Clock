//*************************************************************************************************
//Copyright (c) ��������Ԫ΢�������޹�˾
//�ļ�����:  S_TouchKeyCFG.h
//����: 
//ģ�鹦��:  ���ؼ������ļ�
//�汾:  V0.2
//���ļ�¼:
//************************************************************************************************
#ifndef __S_TOUCHKEYCFG_H__
#define __S_TOUCHKEYCFG_H__
#define		    SOCAPI_SET_TOUCHKEY_TOTAL					  3
#define			SOCAPI_SET_TOUCHKEY_CHANNEL					  0x00000000
#define			SOCAPI_SET_TOUCHKEY_CHANNEL2			      0x00000000
const unsigned int   TKCFG[17] = {0,0,0,5,10,3000,200,100,2,1,0,4,0,1,65535,65535,20}; 
const unsigned char  TKChannelCfg[3][10]={
0x06,0x02,0x2e,0x04,0x08,0x03,0x0e,0x05,0x0f,0xff,
0x06,0x02,0x2e,0x04,0x08,0x03,0x11,0x05,0x0f,0xff,
0x06,0x02,0x2e,0x04,0x08,0x03,0x0f,0x05,0x0f,0xff,
};
#endif
