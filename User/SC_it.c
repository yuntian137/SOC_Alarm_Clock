/**
 *****************************************************************************************************
  * @copyright	(c)  Shenzhen Saiyuan Microelectronics Co., Ltd
  * @file	         SC_it.c
  * @author
  * @version
  * @date
  * @brief
  * @details         Interrupt Service Routine
 *****************************************************************************************************
 * @attention
 *
 *****************************************************************************************************
 */
/********************Includes************************************************************************/
#include "SC_it.h"
#include "sc32_conf.h"
#include "stdio.h"
#include "HeadFiles\SC_itExtern.h"
#include "SCDriver_List.h"

/**************************************Generated by EasyCodeCube*************************************/
//Forbid editing areas between the labels !!!

/*************************************.Generated by EasyCodeCube.************************************/

void INT0_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void INT1_7_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void INT8_11_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void INT12_15_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void RCC_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}


void UART1_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

#if !defined (TK_USE_UART1_3)
void UART1_3_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
	UART_ClearFlag(UART1,UART_Flag_RX | UART_Flag_TX);//Generated by EasyCodeCube, forbid editing!!!
}
#endif

#if !defined (TK_USE_UART1_3_5)
void UART1_3_5_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}
#endif

#if !defined (TK_USE_UART0_2)
void UART0_2_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<UserCodeStart>*//*<SinOne-Tag><471>*/
    //UART_Communication_UART2Handler();
    UART_Handler();
    /*<UserCodeEnd>*//*<SinOne-Tag><471>*/
    /*<Generated by EasyCodeCube end>*/
	UART_ClearFlag(UART2,UART_Flag_RX | UART_Flag_TX);//Generated by EasyCodeCube, forbid editing!!!
}
#endif

#if !defined (TK_USE_UART0_2_4)
void UART0_2_4_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}
#endif


#if !defined (TK_USE_BTM)
void BTM_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}
#endif

void DMA0_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<UserCodeStart>*//*<SinOne-Tag><79>*/
    /*<UserCodeEnd>*//*<SinOne-Tag><79>*/
    /*<UserCodeStart>*//*<SinOne-Tag><455>*/
    TIM_TI_DMA_DMA0Handler();
    /*<UserCodeEnd>*//*<SinOne-Tag><455>*/
    /*<Generated by EasyCodeCube end>*/
}

void DMA1_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}


void DMA2_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void DMA3_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}


void TIMER0_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void TIMER1_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
	TIM_ClearFlag(TIM1, TIM_Flag_TI|TIM_Flag_EXR|TIM_Flag_EXF);//Generated by EasyCodeCube, forbid editing!!!
}

void TIMER2_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
	TIM_ClearFlag(TIM2, TIM_Flag_TI|TIM_Flag_EXR|TIM_Flag_EXF);//Generated by EasyCodeCube, forbid editing!!!
}

void TIMER3_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
	TIM_ClearFlag(TIM3, TIM_Flag_TI|TIM_Flag_EXR|TIM_Flag_EXF);//Generated by EasyCodeCube, forbid editing!!!
}


void TIMER4_5_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void TIMER6_7_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}



void QEP0_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void QEP1_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void OP1_OP2_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void SPI1_TWI1_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void CAN_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void EPWM_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void PCAP_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}


void TWI0_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void TWI1_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void PWM0_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
	PWM_ClearFlag(PWM0, PWM_Flag_PWMIF|PWM_Flag_FLTSTA);//Generated by EasyCodeCube, forbid editing!!!
}

void LEDPWM_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void ADC_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
	ADC_ClearFlag(ADC, ADC_Flag_ADCIF);//Generated by EasyCodeCube, forbid editing!!!
}

void CMP0_1_2_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void CMP3_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void CMP_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}


void SysTick_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void SPI0_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void SPI1_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}

void SPI1_2_IRQHandler(void)
{
    /*<Generated by EasyCodeCube begin>*/
    /*<Generated by EasyCodeCube end>*/
}
