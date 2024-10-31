/**
  ******************************************************************************
  * @file    sc32f1xxx_can.c
  * @author  SOC AE Team
 * @version V1.6
 * @date     04-09-2024
  * @brief   CAN function module
  ******************************************************************************
 * @attention
 *
 *1.This software is supplied by SinOne Microelectronics Co.,Ltd. and is only
 *intended for use with SinOne products. No other uses are authorized. This
 *software is owned by SinOne Microelectronics Co.,Ltd. and is protected under
 *all applicable laws, including copyright laws.
 *2.The software which is for guidance only aims at providing customers with
 *coding information regarding their products in order for them to save time.
 *As a result, SinOne shall not be held liable for any direct, indirect or
 *consequential damages with respect to any claims arising from the content of
 *such software and/or the use made by customers of the coding information
 *contained herein in connection with their products.
 *
 *  COPYRIGHT 2024 SinOne Microelectronics
 */

/* Includes ------------------------------------------------------------------*/
#if defined(SC32f11xx) || defined(SC32f15xx)
#include "sc32f1xxx_can.h"

/** @defgroup CAN_Group1 Initialization and Configuration
 *  @brief   Initialization and Configuration
 *
@verbatim
 ===============================================================================
                     ##### CAN configuration functions #####
 ===============================================================================

@endverbatim
  * @{
  */
/**
  * @brief  Deinitializes the CAN peripheral registers to their default reset values.
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @retval None.
  */
void CAN_DeInit(CAN_TypeDef* CANx)
{
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));

    /* Enable CAN1 reset state */
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_CAN, ENABLE);
    /* Release CAN1 from reset state */
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_CAN, DISABLE);
 
}

/**
  * @brief  Deinitializes the CAN peripheral registers to their default reset values.
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @retval None.
  */
void CAN_Init(CAN_TypeDef *CANx, CAN_InitTypeDef *CAN_InitStruct)
{
		 /* Check the parameters */
		assert_param(IS_CAN_SJW(CAN_InitStruct->S_SJW)); 
		assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CANTimeStamp)); 
		assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_PTB_AutoRetrans)); 
		assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_STB_AutoRetrans)); 
		assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CANTXEN));
		assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CANRXEN));
		assert_param(IS_CAN_TransMod(CAN_InitStruct->CANTSMODESelect)); 
		assert_param(IS_CANTimeStampPosition(CAN_InitStruct->CANTimeStampPositon));
		assert_param(IS_CANFD_ISO_MODE(CAN_InitStruct->CANFDISOSelect));
		assert_param(IS_CANTSMODE(CAN_InitStruct->TransMod)); 
		assert_param(IS_CANTBUF(CAN_InitStruct->CANTBUFSelect)); 
		assert_param(IS_CANROM(CAN_InitStruct->CANROMSelect)); 
	  /*Related register clean */
		CAN_RUF->CAN_RX_ID = 0;
		CAN_RUF->CAN_RX_CTRL = 0;
		CAN_TUF->CAN_TX_ID = 0;
		CAN->CAN_RTIE = 0;
		CAN->CAN_IDE = 0;
   /*The automatic retransmission mode of PTB is set*/
		if(CAN_InitStruct->CAN_PTB_AutoRetrans == ENABLE)
		{CANx->CAN_CFG_STAT |= TPSS_Enable;	}
		else
		{CANx->CAN_CFG_STAT |= TPSS_Disable;}
   /*STB automatic retransmission mode setting*/
		if(CAN_InitStruct->CAN_STB_AutoRetrans == ENABLE)
		{CANx->CAN_CFG_STAT |= TSSS_Enable;}
		else
		{CANx->CAN_CFG_STAT |= TSSS_Disable;}
   /*CAN send enable Settings*/	
		if(CAN_InitStruct->CANTXEN == ENABLE)
		{CANx->CAN_IDE |= TX_Enable;}
		else
		{CANx->CAN_IDE |=TX_Disable;}
    /*CAN receive enable Settings*/
		if(CAN_InitStruct->CANRXEN == ENABLE)
		{CANx->CAN_IDE |=RX_Enable;}
		else
		{CANx->CAN_IDE |=RX_Disable;}
    /*Set the slow baud rate*/
		if(CAN_InitStruct->S_PRESCALER != 0)
		{
		CAN->CAN_S_SEG = 0;
		CANx->CAN_CFG_STAT |= (CAN_CFG_STAT_RESET);
		CAN->CAN_S_SEG |=  ((CAN_InitStruct->S_PRESCALER)<<CAN_S_PRESC_Pos)| \
												 (CAN_InitStruct->S_SJW << CAN_S_SJW_Pos)    | \
												 (CAN_InitStruct->S_SEG2 << CAN_S_SEG_2_Pos) | \
												 CAN_InitStruct->S_SEG1;
		}
    /*Set the high baud rate*/
		if(CAN_InitStruct->F_PRESCALER != 0)
		{
		CAN->CAN_F_SEG = 0;
		CAN->CAN_CFG_STAT |= (CAN_CFG_STAT_RESET);
		CAN->CAN_F_SEG |=  ((CAN_InitStruct->F_PRESCALER)<<CAN_F_PRESC_Pos)  | \
												 (CAN_InitStruct->F_SJW << CAN_F_SJW_Pos)  | \
												 (CAN_InitStruct->F_SEG2 << CAN_F_SEG_2_Pos)  | \
												 CAN_InitStruct->F_SEG1;
		}
		 /*Set the CAN working mode*/
{
		CANx->CAN_CFG_STAT &= ~(CAN_CFG_STAT_RESET);
		CANx->CAN_CFG_STAT |= CAN_CFG_STAT_BUSOFF;
		if(CAN_InitStruct->CAN_MOD ==CAN_Listenonly )
		{
			CANx->CAN_CFG_STAT |= CAN_CFG_STAT_LOM;
		}
		else if (CAN_InitStruct->CAN_MOD ==CAN_StandBy )
		{
			CANx->CAN_CFG_STAT |=CAN_CFG_STAT_STBY;
		}
		else if(CAN_InitStruct->CAN_MOD ==CAN_LoopBack )
		{
			CANx->CAN_CFG_STAT |= CAN_CFG_STAT_LBME;
		}
		else if(CAN_InitStruct->CAN_MOD ==CAN_LoopBackIn )
		{
			CANx->CAN_CFG_STAT |= CAN_CFG_STAT_LBMI;
		}
		else if(CAN_InitStruct->CAN_MOD ==CAN_LoopBackSack )
		{
			CANx->CAN_CFG_STAT |= CAN_CFG_STAT_LBME;
			CANx->CAN_CFG_STAT |= CAN_CFG_STAT_ACK;
		}
			CANx->CAN_CFG_STAT &= ~(CAN_CFG_STAT_BUSOFF);
			CANx->CAN_CFG_STAT &= ~(CAN_CFG_STAT_RESET);
		
}
	

	  /*CANFD Frame format Settings*/
		CANx->CAN_IDE |= CAN_InitStruct->CANFDFrame;
		/*Timestamp count enable setting*/
		CANx->CAN_IDE |= CAN_InitStruct->CANTimeStampCnt<<CAN_IDE_TIMEN_Pos;

		CANx->CAN_ACFCTRL |= CAN_InitStruct->CANTimeStamp<<CAN_ACFCTRL_TIMEEN_Pos;

		CANx->CAN_ACFCTRL |= CAN_InitStruct->CANTimeStampPositon<<CAN_ACFCTRL_TIMEPOS_Pos;
     
		CANx->CAN_CFG_STAT &= ~(0x80);	
    /*Send buffer setting,ISO setting,STB send mode Settings*/
		CANx->CAN_CFG_STAT |= CAN_InitStruct->CANTBUFSelect|CAN_InitStruct->CANFDISOSelect|CAN_InitStruct->CANTSMODESelect;

}



/**
  * @brief  Fills each CAN_InitStruct member with its default value.
  * @param  CAN_InitStruct[out]: Pointer to structure CAN_InitStruct, to be initialized. 
  * @retval None.
  */
void CAN_StructInit(CAN_InitTypeDef* CAN_InitStruct)
{
	/* Set the default configuration */
  	CAN_InitStruct->S_PRESCALER = 0;
	  CAN_InitStruct->S_SEG1 = 0;
    CAN_InitStruct->S_SEG2 = 0;
    CAN_InitStruct->S_SJW = 0;
    CAN_InitStruct->F_PRESCALER = 0;
    CAN_InitStruct->F_SEG1 = 0;
    CAN_InitStruct->F_SEG2 = 0;
    CAN_InitStruct->F_SJW = 0;
    CAN_InitStruct->CANFDFrame = FD_Mode_on;
    CAN_InitStruct->CANROMSelect = CANROM_Old;
    CAN_InitStruct->CAN_MOD = CAN_Normal;
    CAN_InitStruct->CANTXEN = ENABLE;
    CAN_InitStruct->CANRXEN = ENABLE;
    CAN_InitStruct->CANFDISOSelect = CAN_ISO;
    CAN_InitStruct->CANTSMODESelect = TX_FIFO;
    CAN_InitStruct->CANTBUFSelect = CANTBUF_Tx_STB;
    CAN_InitStruct->CANTimeStampPositon = CANTimeStampPosition_SOF;
    CAN_InitStruct->CAN_MOD = CAN_TransMod_TSONE;
    CAN_InitStruct->CAN_PTB_AutoRetrans = DISABLE;
    CAN_InitStruct->CAN_STB_AutoRetrans = DISABLE;
    CAN_InitStruct->CANTimeStamp = ENABLE;
    CAN_InitStruct->CANTimeStampCnt = ENABLE;

}
/**
  * @brief  Fills each CAN_InitStruct member with its default value.
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @param  CANTBUF[out]: Pointer to structure CANTBUF_TypeDef
  *                  - CANTBUF_Tx_PTB :Send buffer Select PTB 
  *                  - CANTBUF_Tx_STB: Send buffer Select STB 
  * @retval None.
  */
void CAN_TBUFSelect(CAN_TypeDef *CANx, CANTBUF_TypeDef CANTBUF)
{
	if(CANTBUF  ==  CANTBUF_Tx_STB)
	CAN->CAN_CFG_STAT |= CANTBUF_Tx_STB;	//Send buffer Select STB 
	if(CANTBUF == CANTBUF_Tx_PTB)
	CAN->CAN_CFG_STAT &= ~CANTBUF_Tx_STB;	//Send buffer Select PTB 
	
}
/**
  * @brief  Settings Select the option to set the high-speed baud rate of the CAN peripheral
  * @param  BaudRate[in]: Select to set the high-speed baud rate of CAN.
  *                    - Baudrate_125kHz:Select set the high speed baud rate of CAN to 125khz
	*                    - Baudrate_250kHz:Select set the high speed baud rate of CAN to 250khz
	*                    - Baudrate_500kHz:Select set the high speed baud rate of CAN to 500khz	
	*                    - Baudrate_800kHz:Select set the high speed baud rate of CAN to 800khz
	*                    - Baudrate_1000kHz:Select set the high speed baud rate of CAN to 1000khz
  * @retval None.
  */
void CAN_FBaudRate_Select(unsigned int CAN_Clk,unsigned int CAN_FPrescaler,CAN_BaudRate BaudRate)
{ 
   int F_PRESCALER,F_SEG1,F_SEG2,F_SJW,F_SEG;
	F_SJW = 1;
	F_PRESCALER = CAN_FPrescaler;
	 double   Fast_Sampling_pos;
		/*���ݲ�����ȷ��ȡ��λ��*/
	if(BaudRate >= BaudRate_800k)
	{
		Fast_Sampling_pos = Sampling_pos_75;
	}
	else if(BaudRate >= BaudRate_500k)
	{
		Fast_Sampling_pos = Sampling_pos_80;
	}
	else
	{
		Fast_Sampling_pos = Sampling_pos_875;
	}
  CAN->CAN_F_SEG = 0;
	/*����SEG1,SEG2*/
	F_SEG = (CAN_Clk/(BaudRate * (F_PRESCALER+1)));
	F_SEG1 = F_SEG*Fast_Sampling_pos - 2;
	F_SEG2 = F_SEG*(1 - Fast_Sampling_pos) - 1;	

     /*Set the high baud rate*/

		CAN->CAN_CFG_STAT |= (CAN_CFG_STAT_RESET);
		CAN->CAN_F_SEG |=   (F_PRESCALER<<CAN_F_PRESC_Pos) | \
												(F_SJW << CAN_F_SJW_Pos)       | \
												(F_SEG2<< CAN_F_SEG_2_Pos)     | \
												 F_SEG1;	
}

/**
  * @brief  Settings Select the option to set the slow-speed baud rate of the CAN peripheral
  * @param  BaudRate[in]: Select to set the slow-speed baud rate of CAN.
  *                    - Baudrate_125kHz:Select set the high speed baud rate of CAN to 125khz
	*                    - Baudrate_250kHz:Select set the high speed baud rate of CAN to 250khz
	*                    - Baudrate_500kHz:Select set the high speed baud rate of CAN to 500khz	
	*                    - Baudrate_800kHz:Select set the high speed baud rate of CAN to 800khz
	*                    - Baudrate_1000kHz:Select set the high speed baud rate of CAN to 1000khz
  * @retval None.
  */
void CAN_SBaudRate_Select(unsigned int CAN_Clk,unsigned int CAN_SPrescaler,CAN_BaudRate BaudRate)
{
  double   Slow_Sampling_pos;
  int S_PRESCALER,S_SEG1,S_SEG2,S_SJW,S_SEG;
	S_SJW= 1;
	S_PRESCALER = CAN_SPrescaler;

		/*���ݲ�����ȷ��ȡ��λ��*/
	if(BaudRate >= BaudRate_800k)
	{
		Slow_Sampling_pos = Sampling_pos_75;
	}
	else if(BaudRate >= BaudRate_500k)
	{
		Slow_Sampling_pos = Sampling_pos_80;
	}
	else
	{
		Slow_Sampling_pos = Sampling_pos_875;
	}
	/*����SEG1,SEG2*/
	S_SEG = (CAN_Clk/(BaudRate * (S_PRESCALER+1)));
	S_SEG1 = S_SEG*Slow_Sampling_pos - 2;
	S_SEG2 = S_SEG*(1 - Slow_Sampling_pos) - 1;
  CAN->CAN_S_SEG = 0;

		CAN->CAN_CFG_STAT |= (CAN_CFG_STAT_RESET);
		CAN->CAN_S_SEG |=   (S_PRESCALER<<CAN_S_PRESC_Pos) | \
												(S_SJW << CAN_S_SJW_Pos)       | \
												(S_SEG2<< CAN_S_SEG_2_Pos)     | \
												 S_SEG1;	
}
/**
  * @brief  Settings Select the sending mode of the CAN peripheral.
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @param TransModSelect[in]:Settings Select the CAN sending mode.
  *                  - CAN_TransMod_TPE :PTB Send a single message
  *                  - CAN_TransMod_TSONE:STB single send
  *                  - CAN_TransMod_TSALL:STB send all
  * @retval None.
  */
void CAN_Trans_Select(CAN_TypeDef *CANx,CAN_TransMod_TypeDef TransModSelect)
{
		if(TransModSelect==CAN_TransMod_TSONE)
		CANx->CAN_CFG_STAT |= CAN_TransMod_TSONE;
		else if(TransModSelect==CAN_TransMod_TSALL)
		CANx->CAN_CFG_STAT |= CAN_TransMod_TSALL;
		else if(TransModSelect==CAN_TransMod_TPE)
		CANx->CAN_CFG_STAT |= CAN_TransMod_TPE;

}


/**
  * @brief Set the threshold of the CAN peripheral
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @param  Rth[in]:Threshold of CAN peripherals. 
  * @retval None.
  */
void CAN_RxThresholdConfig(CAN_TypeDef *CANx, int Rth)
{
	  CANx->CAN_RTIE |= Rth << CAN_RTIE_AFWL_Pos; 
}


/**
  * @brief  Set filters for CAN peripherals.
  * @param  CAN_FliterStruct[out]: Pointer to structure CAN_FliterTypeDef, to be initialized.   
  * @retval None.
  */
void CAN_FliterInit( CAN_FliterTypeDef *CAN_FliterStruct)
{
		uint32_t state = 0;
		state |= CAN->CAN_CFG_STAT;
		CAN->CAN_CFG_STAT |= (0x80);	
		/*Clear filter pointer*/
		CAN->CAN_ACFCTRL &= 0xFFFFFF00;
		/*The filter pointer points to the filter to be configured*/
		CAN->CAN_ACFCTRL |= CAN_FliterStruct->CAN_Fliter<<CAN_ACFCTRL_ACFADR_Pos;
		
		/*The pointer points to the Acode area and writes to the Acode*/
		CAN->CAN_ACFCTRL &= ~(CAN_ACFCTRL_SELMASK);
		/*Acode Clear zero*/
		CAN->CAN_ACF &= 0x0;
		/*Write to Acode*/
		CAN->CAN_ACF |= CAN_FliterStruct->Acode<<CAN_ACF_AMASK_ACODE_Pos;
		
		/*Pointer to Amask area, write Amask*/
		CAN->CAN_ACFCTRL |= CAN_ACFCTRL_SELMASK;
		/*Amask clear zero*/
		CAN->CAN_ACF &= 0X0;
		/*Write to Amask*/
		CAN->CAN_ACF |= CAN_FliterStruct->Amask<<CAN_ACF_AMASK_ACODE_Pos;
		
		/*Determine the frame filtering type*/
		switch(CAN_FliterStruct->CANFliterFrame)
		{
			case Fliter_Standard:
			{
				CAN->CAN_ACF |= CAN_ACF_AIDEE;
				CAN->CAN_ACF &= ~(CAN_ACF_AIDE);
			}
			break;

			case Fliter_Extension:
			{
				CAN->CAN_ACF |= CAN_ACF_AIDEE;
				CAN->CAN_ACF |= CAN_ACF_AIDE;
			}
			break;

			case Fliter_Default:
				CAN->CAN_ACF &= ~(CAN_ACF_AIDEE);
			break;

			default:
			break;
		}

		/*Write Amask*/
		CAN->CAN_ACF |= CAN_FliterStruct->Amask<<CAN_ACF_AMASK_ACODE_Pos;;
		/*Turn on the filter*/
		CAN->CAN_ACFCTRL |= 1<<((CAN_FliterStruct->CAN_Fliter)+16);
		CAN->CAN_CFG_STAT |=state;
		CAN->CAN_CFG_STAT &= ~(0x80);	
}

/**
  * @brief  Set the CAN working mode.
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @param  CAN_MOD[in]:Select the working mode of CAN.  
  *                  -  CAN_Normal:Select the CAN_Normal.
  *                  - 	CAN_Listenonly:Select the CAN_Listenonly.
  *                  - 	CAN_StandBy:Select the CAN_StandBy.
  *                  - 	CAN_LoopBack:Select the CAN_LoopBack.
  *                  - 	CAN_LoopBackIn:Select the CAN_LoopBackIn.
  *                  - 	CAN_LoopBackSack:Select the CAN_LoopBackSack.
  * @retval None 
  */
void CAN_ModeSelect(CAN_TypeDef *CANx, ModSelect_TypeDef CAN_MOD)
{
	CANx->CAN_CFG_STAT &= ~(CAN_CFG_STAT_RESET);
	CANx->CAN_CFG_STAT |= CAN_CFG_STAT_BUSOFF;
	switch(CAN_MOD)
	{
		/*General transceiver mode*/
		case CAN_Normal:
		break;

		/*Listen only mode*/
		case CAN_Listenonly:
		CANx->CAN_CFG_STAT |= CAN_CFG_STAT_LOM;
		break;

		/*Standby mode*/
		case CAN_StandBy:
		CANx->CAN_CFG_STAT |=CAN_CFG_STAT_STBY;
		break;

		/*External loop mode*/
		case CAN_LoopBack:
		CANx->CAN_CFG_STAT |= CAN_CFG_STAT_LBME;
		break;

		/*Internal loop mode*/
		case CAN_LoopBackIn:
		CANx->CAN_CFG_STAT |= CAN_CFG_STAT_LBMI;
		break;

		/*External loop mode Auto-answer mode*/
		case CAN_LoopBackSack:
			CANx->CAN_CFG_STAT |= CAN_CFG_STAT_LBME;
			CANx->CAN_CFG_STAT |= CAN_CFG_STAT_ACK;
		break;

		default:
		break;
	}
	CANx->CAN_CFG_STAT &= ~(CAN_CFG_STAT_BUSOFF);
	CANx->CAN_CFG_STAT &= ~(CAN_CFG_STAT_RESET);
}


/**
 * @}
 */
/* End of CAN_Group1.	*/

/** @defgroup CAN_Group2 Data transfers functions
 *  @brief   Data transfers functions
 *
@verbatim
 ===============================================================================
                      ##### Data transfers functions #####
 ===============================================================================
@endverbatim
  * @{
  */

/**
  * @brief  CAN peripherals send initialization.
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @param  CAN_Tx_msg[out]:[out]: Pointer to structure CAN_Tx_msg_Typedef, to be initialized.   
  * @retval None.
  */
void CAN_Trans_Init(CAN_TypeDef *CANx, CanTxMsg *CAN_Tx_msg)
{	
	uint8_t index;
	/*Gets the data length value*/
	static int Tx_msg_cnt = 0;
  /*Since the values in BUF are garbled after each burn, they need to be cleared each time slot is replaced*/
	CAN_TUF->CAN_TX_ID = 0x00000000; 
	CAN_TUF->CAN_TX_CTRL = 0x00000000;

	if(CAN_Tx_msg->CAN_IDE != CAN_Standard_format)
	{
		/*Set the extension ID*/
		CAN_TUF->CAN_TX_CTRL |= CAN_TX_CTRL_IDE;
		/*Write ID*/
		CAN_TUF->CAN_TX_ID = CAN_Tx_msg->CAN_TXID;
	}
	else
	{
		/*Set standard ID*/
		CAN_TUF->CAN_TX_CTRL &= ~(CAN_TX_CTRL_IDE);
		/*Write ID*/
		CAN_TUF->CAN_TX_ID = CAN_Tx_msg->CAN_TXID;
	}

	if(CAN_Tx_msg->CAN_RTR != CAN_Data_frame)
	{
		/*Setting remote frame*/
		CAN_TUF->CAN_TX_CTRL |= CAN_TX_CTRL_RTR;
	}
	else
	{
		/*Set data frame*/
		CAN_TUF->CAN_TX_CTRL &= ~(CAN_TX_CTRL_RTR);
	}

	if(CAN_Tx_msg->CAN_FDF != CAN_Standard_frame)
	{
		/*Set FD frame*/
		CAN_TUF->CAN_TX_CTRL |= CAN_TX_CTRL_FDF;
	}
	else
	{
		/*Set 2.0 frames*/
		CAN_TUF->CAN_TX_CTRL &= ~(CAN_TX_CTRL_FDF);
	}

	if(CAN_Tx_msg->CAN_BRS != CAN_Disable_BRS)
	{
		/*The baud rate change mode was enabled*/
		CAN_TUF->CAN_TX_CTRL |= CAN_TX_CTRL_BRS;
	}
	else
	{
		/*Disable variable baud rate mode*/
		CAN_TUF->CAN_TX_CTRL &= ~(CAN_TX_CTRL_BRS);
	}

	/*Write data length*/

		 
	
	CAN_TUF->CAN_TX_CTRL |= CAN_Tx_msg->Data_len<<CAN_TX_CTRL_DLC_Pos;


	/*Each frame is written according to the actual data length*/
	for(index = 0; index<CAN_Tx_msg->Tx_msg_len; index++)
	{	
		/*Write message*/
		CAN_TUF->CAN_TBUF[index]=CAN_Tx_msg->Tx_msg[index];
		Tx_msg_cnt++;

	}

	if((CANx->CAN_CFG_STAT) & (CAN_CFG_STAT_TBSEL))
	{
		/*After a frame is written, it points to the next frame storage area*/
		CANx->CAN_CFG_STAT |= CAN_CFG_STAT_TSNEXT;
		
	}
}
/**
  * @brief  CAN peripherals send initialization.
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @retval None.
  */
void CAN_TransStop(CAN_TypeDef *CANx)
{
	CANx->CAN_CFG_STAT |= CAN_CFG_STAT_TSA;
}
/**
  * @brief  CAN peripherals send initialization.
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @param  RxMessage[out]:[out]: Pointer to structure CAN_RxMsg_Typedef, to be initialized.   
  * @retval None.
  */
void CAN_RecieveConfig(CAN_TypeDef *CANx, CanRxMsg* RxMessage)
{

	uint8_t index;

	/*DLC converts array with actual data length*/
	int Reallen[16] = {0,4>>2,4>>2,4>>2,4>>2,8>>2,8>>2,8>>2,8>>2,12>>2,16>>2,20>>2,24>>2,32>>2,48>>2,64>>2};

	if((CAN_RUF->CAN_RX_CTRL & CAN_RX_CTRL_IDE) != 0)
	{
		RxMessage->CAN_IDE = CAN_Standard_format;

		/*Write ID*/
		RxMessage->CAN_RXID=CAN_RUF->CAN_RX_ID  ;
	}
	else
	{
		/*Set standard ID*/
    RxMessage->CAN_IDE = CAN_Extended_format;
	/*Write ID*/
		RxMessage->CAN_RXID=CAN_RUF->CAN_RX_ID  ;
	}

	if((CAN_RUF->CAN_RX_CTRL&CAN_RX_CTRL_RTR) != 0)
	{
		/*Setting remote frame*/
		RxMessage->CAN_RTR |= CAN_Remote_frame;
	}
	else
	{
		/*Set data frame*/
		RxMessage->CAN_RTR &= CAN_Data_frame;
	}

	if((CAN_RUF->CAN_RX_CTRL&CAN_RX_CTRL_FDF)  != 0)
	{
		/*Set FD frame*/
		RxMessage->CAN_FDF |= CAN_FD_frame;
	}
	else
	{
		/*Set 2.0 frames*/
		RxMessage->CAN_FDF  &= CAN_Standard_frame;
	}

	if((CAN_RUF->CAN_RX_CTRL&CAN_RX_CTRL_BRS) != 0)
	{
		/*The baud rate change mode was enabled*/
		RxMessage->CAN_BRS |= CAN_Enable_BRS;
	}
	else
	{
		/*Disable variable baud rate mode*/
		RxMessage->CAN_BRS &=CAN_Disable_BRS;
	}

 	/*Write data length*/
	RxMessage->Data_len |= (CAN_RUF->CAN_RX_CTRL&CAN_RX_CTRL_DLC);

 int Rlen = RxMessage->Data_len;
	/*Each frame is written according to the actual data length*/
	for(index = 0; index<Reallen[Rlen]; index++)
	{	
		
		/*Write message*/
		RxMessage->Rx_msg[index]=CAN_RUF->CAN_RBUF[index];

	}
 
   CANx->CAN_CFG_STAT |= CAN_CFG_STAT_RREL;


}

/**
 * @}
 */
/* End of CAN_Group2.	*/
/** @defgroup CAN_Group3 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions
 *
@verbatim
 ===============================================================================
           ##### Interrupts and flags management functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  The CAN peripheral enables interruption
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @param  CAN_IT[in]:Select the CAN peripheral  interruption. 
  *                 - RIEEN:Select the receiving interrupt.
  *                 - ROIEEN:Select the Overflow interrupt. 
  *                 - RFIEEN:Select the RB full interrupt.  
  *                 - RAFIEEN:Select the RB slots Threshold warning Indicates the interrupt.
  *                 - TPIEEN:Select the interruption of PTB transmission completion.
  *                 - TSIEEN:Select the STB transmission completion interrupt.
  *                 - EIEEN:Select the error interrupt.
  *                 - ALIEEN:Select the arbitration loss interruption.
  *                 - BEIEEN:Select the Bus error interrupt.
  *                 - EPIEEN:Select the Error passive interrupt.
  * @param  NewState[in]: new state of the SPI interrupts.
  *               - DISABLE:Function disable
  *               - ENABLE:Function enable 
  * @retval None.
  */
void CAN_ITConfig(CAN_TypeDef* CANx, uint32_t CAN_IT, FunctionalState NewState)
{
		assert_param(IS_CAN_INT(CAN_IT) );
		assert_param(IS_FUNCTIONAL_STATE(NewState));
	  /*Open total interrupt*/
		CANx->CAN_IDE |=0x01;   
		if (NewState != DISABLE)
		{
			/* Enable the selected CANx interrupt */
			CANx->CAN_RTIE |= CAN_IT;
		}
		else
		{
			/* Disable the selected CANx interrupt */
			CANx->CAN_RTIE &= ~CAN_IT;
		}
}



/**
  * @brief  Gets the CAN flag bit status.
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @param  CAN_Flag[out]:Select the flag bit of CAN. 
  *                  -  RIF_FlAG:Select the RIF_FlAG
  *                  -  TSFF_FLAG:Select the TSFF_FLAG.
  *                  - 	AIF_FLAG:Select the AIF_FLAG.
  *                  - 	EIF_FLAG:Select the EIF_FLAG.
  *                  - 	TSIF_FLAG:Select the TSIF_FLAG.
  *                  - 	TPIF_FLAG:Select the TPIF_FLAG.
  *                  - 	RAFIF_FLAG:Select the RAFIF_FLAG.
  *                  - 	RFIF_FLAG:Select the RFIF_FLAG.
  *                  - 	ROIF_FLAG:Select the ROIF_FLAG.
  *                  - 	ALIF_FLAG:Select the ALIF_FLAG.
  *                  - 	BEIF_FLAG:Select the BEIF_FLAG.
  *                  - 	EPIF_FLAG:Select the EPIF_FLAG.
  * @retval Flag bit status
	*        - RESET,
  *        - SET 
  */

ITStatus CAN_GetFlagStatus(CAN_TypeDef *CANx, uint32_t CAN_Flag)
{
	ITStatus bitstatus = RESET;  
	uint16_t itstatus = 0x0;
  assert_param( IS_CAN_FLAG(CAN_Flag) );
	/*Read flag bit information*/
	itstatus = CANx->CAN_RTIE & CAN_Flag;
	if (itstatus != (uint16_t)RESET )
	{
		bitstatus = SET;
	}
	else
	{
		bitstatus = RESET;
	}
	return bitstatus;
}



/**
  * @brief  Clear the CAN flag bit .
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @param  CAN_Flag[out]:Select the flag bit of CAN. 
  *                  -  RIF_FlAG:Select the RIF_FlAG.
  *                  -  TSFF_FLAG:Select the TSFF_FLAG.
  *                  - 	AIF_FLAG:Select the AIF_FLAG.
  *                  - 	EIF_FLAG:Select the EIF_FLAG.
  *                  - 	TSIF_FLAG:Select the TSIF_FLAG.
  *                  - 	TPIF_FLAG:Select the TPIF_FLAG.
  *                  - 	RAFIF_FLAG:Select the RAFIF_FLAG.
  *                  - 	RFIF_FLAG:Select the RFIF_FLAG.
  *                  - 	ROIF_FLAG:Select the ROIF_FLAG.
  *                  - 	ALIF_FLAG:Select the ALIF_FLAG.
  *                  - 	BEIF_FLAG:Select the BEIF_FLAG.
  *                  - 	EPIF_FLAG:Select the EPIF_FLAG.
  * @retval None 
  */
void CAN_ClearFlag(CAN_TypeDef *CANx, uint32_t CAN_Flag)
{
  	/* Check the parameters */
		assert_param( IS_CAN_FLAG(CAN_Flag) );
	  /*Clear the CAN flag bit */
		if((CAN_Flag&( CANx->CAN_RTIE&RIF_FlAG))!=0)
		{
		  CANx->CAN_RTIE &=0xFFEA80FE;
		}
		if((CAN_Flag&( CANx->CAN_RTIE&TSFF_FLAG))!=0)
		{
		  CANx->CAN_RTIE &=0xFFEA00FF;
		}
		if((CAN_Flag&( CANx->CAN_RTIE&AIF_FLAG))!=0)
		{
		  CANx->CAN_RTIE &=0xFFEA01FE;
		}
		if((CAN_Flag&( CANx->CAN_RTIE&EIF_FLAG))!=0)
		{
		  CANx->CAN_RTIE &=0xFFEA02FE;
		}
	  if((CAN_Flag&(CANx->CAN_RTIE&TSIF_FLAG))!=0)
		{
		  CANx->CAN_RTIE &=0xFFEA04FE;
		}
	  if((CAN_Flag&( CANx->CAN_RTIE&TPIF_FLAG))!=0)
		{
		  CANx->CAN_RTIE &=0xFFEA08FE;
		}
	  if((CAN_Flag&( CANx->CAN_RTIE&RAFIF_FLAG))!=0)
		{
		  CANx->CAN_RTIE &=0xFFEA10FE;
		}
	  if((CAN_Flag&( CANx->CAN_RTIE&RFIF_FLAG))!=0)
		{
		 CANx->CAN_RTIE &=0xFFEA20FE;
		}
	  if((CAN_Flag&( CANx->CAN_RTIE&ROIF_FLAG))!=0)
		{
		 CANx->CAN_RTIE &=0xFFEA40FF;
		}
	  if((CAN_Flag&( CANx->CAN_RTIE&ALIF_FLAG))!=0)
		{
		 CANx->CAN_RTIE &=0xFFEE00FF;
		}
	  if((CAN_Flag&( CANx->CAN_RTIE&BEIF_FLAG))!=0)
		{
		 CANx->CAN_RTIE &=0xFFEB00FF;
		}
	  if((CAN_Flag&( CANx->CAN_RTIE&EPIF_FLAG))!=0)
		{
		 CANx->CAN_RTIE &=0xFFFA00FF;
		}

}
/**
 * @}
 */
/* End of CAN_Group3.	*/
/** @defgroup CAN_Group4  Number of error data and Reset 
 *  @brief    Number of error data and Reset management functions
 *
@verbatim
 ===============================================================================
           ##### Number of error data and Reset #####
 ===============================================================================

@endverbatim
  * @{
  */



/**
  * @brief  Gets the number of CAN sent errors.
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @retval CAN send the number of errors
  */
uint8_t CAN_GetTECNT(CAN_TypeDef* CANx)
{
  uint8_t errorcode;
  
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  
  /* Get the error code*/
  errorcode = ((((uint8_t)CANx->CAN_EALCAP) & 0xF0000000)>>24);
  
  /* Return the error code*/
  return errorcode;
}

/**
  * @brief  Gets the number of CAN receive errors.
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @retval CAN receive the number of errors
  */
uint8_t CAN_GetRECNT(CAN_TypeDef* CANx)
{
  uint8_t errorcode;
  
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  
  /* Get the error code*/
  errorcode = ((((uint8_t)CANx->CAN_EALCAP) & 0x0F000000)>>16);
  
  /* Return the error code*/
  return errorcode;
}

/**
  * @brief  resetting the CAN peripheral.
  * @param  CANx[out]:where x can be select the CANx peripheral. 
	*                  - CAN:select the CAN peripheral.
  * @retval None.
  */
void CAN_ResetCmd(CAN_TypeDef *CANx, FunctionalState NewState)
{
	if(NewState != DISABLE)
	{
		CANx->CAN_CFG_STAT |= CAN_CFG_STAT_RESET;
	}
	else
	{
		CANx->CAN_CFG_STAT &= ~(CAN_CFG_STAT_RESET);
	}
}


/**
  * @}
  */


#endif
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
