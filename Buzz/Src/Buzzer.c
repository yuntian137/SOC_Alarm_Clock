#include "sc32f1xxx_tim.h"
#include "Buzzer.h"
#include "Delay.h"
#include "stdint.h"
//播放速度，值为四分音符的时长(ms)
#define Speed 500
 
//音符对应定时器分频系数，P：休止符，L：低音，M：中音，H：高音，下划线：升半音符号#
#define P	   50000   
#define L1     131
#define L2     147
#define L3     165
#define L4     175
#define L5     196
#define L6     221
#define L7     248
#define M1     262
#define M2     294
#define M3     330
#define M4     350
#define M5     393
#define M6     441
#define M7     495
#define H1     525
#define H2     589
#define H3     661
#define H4     700
#define H5     786
#define H6     882
#define H7     990

 
//天空之城乐谱
const uint16_t Music[]=
{
	//音符,时值,
	
	//1
	P,	4,
	P,	4,
	P,	4,
	M6,	2,
	M7,	2,
	
	H1,	4+2,
	M7,	2,
	H1,	4,
	H3,	4,
	
	M7,	4+4+4,
	M3,	2,
	M3,	2,
	
	//2
	M6,	4+2,
	M5,	2,
	M6, 4,
	H1,	4,
	
	M5,	4+4+4,
	M3,	4,
	
	M4,	4+2,
	M3,	2,
	M4,	4,
	H1,	4,
	
	//3
	M3,	4+4,
	P,	2,
	H1,	2,
	H1,	2,
	H1,	2,
	
	M7,	4+2,
	M4,2,
	M4,4,
	M7,	4,
	
	M7,	8,
	P,	4,
	M6,	2,
	M7,	2,
	
	//4
	H1,	4+2,
	M7,	2,
	H1,	4,
	H3,	4,
	
	M7,	4+4+4,
	M3,	2,
	M3,	2,
	
	M6,	4+2,
	M5,	2,
	M6, 4,
	H1,	4,
	
	//5
	M5,	4+4+4,
	M2,	2,
	M3,	2,
	
	M4,	4,
	H1,	2,
	M7,	2+2,
	H1,	2+4,
	
	H2,	2,
	H2,	2,
	H3,	2,
	H1,	2+4+4,
	
	//6
	H1,	2,
	M7,	2,
	M6,	2,
	M6,	2,
	M7,	4,
	M5,4,
	
	
	M6,	4+4+4,
	H1,	2,
	H2,	2,
	
	H3,	4+2,
	H2,	2,
	H3,	4,
	H5,	4,
	
	//7
	H2,	4+4+4,
	M5,	2,
	M5,	2,
	
	H1,	4+2,
	M7,	2,
	H1,	4,
	H3,	4,
	
	H3,	4+4+4+4,
	
	//8
	M6,	2,
	M7,	2,
	H1,	4,
	M7,	4,
	H2,	2,
	H2,	2,
	
	H1,	4+2,
	M5,	2+4+4,
	
	H4,	4,
	H3,	4,
	H3,	4,
	H1,	4,
	
	//9
	H3,	4+4+4,
	H3,	4,
	
	H6,	4+4,
	H5,	4,
	H5,	4,
	
	H3,	2,
	H2,	2,
	H1,	4+4,
	P,	2,
	H1,	2,
	
	//10
	H2,	4,
	H1,	2,
	H2,	2,
	H2,	4,
	H5,	4,
	
	H3,	4+4+4,
	H3,	4,
	
	H6,	4+4,
	H5,	4+4,
	
	//11
	H3,	2,
	H2,	2,
	H1,	4+4,
	P,	2,
	H1,	2,
	
	H2,	4,
	H1,	2,
	H2,	2+4,
	M7,	4,
	
	M6,	4+4+4,
	P,	4,
	
	0xFF	//终止标志，防止数组越界之后乱音，用一个最大值来做一个终止标志，也可以设别的值
};
 
void PWM_SetHz(uint16_t Hz)
{
	//TIM_SetCounter(TIM2, 32768 - Hz);
    TIM_SetAutoreload(TIM2, 100000/(1.8*Hz));
	//TIM_SetAutoreload(TIM2, 32768 - Hz);
}
 
void test_Buzz()
{
	TIM_Cmd(TIM2, ENABLE);
	PWM_SetHz(M1);
	Delay_ms(500);
	PWM_SetHz(P);
	Delay_ms(500);
	PWM_SetHz(M2);
	Delay_ms(500);
	PWM_SetHz(M3);
	Delay_ms(500);
	PWM_SetHz(M4);
	Delay_ms(700);
	TIM_Cmd(TIM2, DISABLE);
}
 
void Buzzer_Play()
{
    static uint16_t MusicSelect=0,TimeSelect=0;
 
    while(Music[MusicSelect]!=0xFF)
    {
        TIM_Cmd(TIM2, ENABLE);
        PWM_SetHz(Music[MusicSelect]);
 
        TimeSelect=Music[++MusicSelect];
        Delay_ms(Speed/4*TimeSelect);
        MusicSelect++;
 
        Delay_ms(10);
    }
    if(Music[MusicSelect]==0xFF) 
    {
        MusicSelect=0;
        TIM_Cmd(TIM2, DISABLE);
    }
}
 

