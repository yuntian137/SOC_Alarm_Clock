//*************************************************************************************************
//Copyright (c) 深圳市赛元微电子有限公司
//文件名称:  S_TouchKeyCFG.h
//作者: 
//模块功能:  触控键配置文件
//版本:  V0.2
//更改记录:
//************************************************************************************************
#ifndef __S_TOUCHKEYCFG_H__
#define __S_TOUCHKEYCFG_H__
#define		    SOCAPI_SET_TOUCHKEY_TOTAL					  8
#define			SOCAPI_SET_TOUCHKEY_CHANNEL					  0x0007F800
#define			SOCAPI_SET_TOUCHKEY_CHANNEL2			      0x00000000
const unsigned int TKCFG[17] = {0,0,0,5,10,3000,200,100,2,1,0,4,0,1,65535,65535,62}; 
const unsigned char TKChannelCfg[8][10]={
0x04,0x02,0x28,0x04,0x08,0x03,0x30,0x05,0x05,0xdc,
0x04,0x02,0x28,0x04,0x08,0x03,0x38,0x05,0x06,0x2b,
0x04,0x02,0x28,0x04,0x08,0x03,0x32,0x05,0x06,0x0c,
0x04,0x02,0x28,0x04,0x08,0x03,0x36,0x05,0x06,0x4c,
0x04,0x02,0x28,0x04,0x08,0x03,0x3e,0x05,0x05,0xc0,
0x04,0x02,0x28,0x04,0x08,0x03,0x36,0x05,0x05,0x74,
0x04,0x02,0x28,0x04,0x08,0x03,0x28,0x05,0x06,0x4b,
0x04,0x02,0x28,0x04,0x08,0x03,0x2e,0x05,0x06,0x03,
};
#endif
