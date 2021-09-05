#include "UART.h"

static void UART_CloseISR(UART_Handle_Typedef_t*	Uart_Handle)
{
	Uart_Handle->TxBuffSize = 0;
	Uart_Handle->pTxBuff = NULL;
	Uart_Handle->TxStatus = UART_BUS_FREE;
	Uart_Handle->Instance->UART_CR1 &= ~(0x1 << UART_TxEIE);


}
static void CloseISR_Rx(UART_Handle_Typedef_t*	Uart_Handle)
{
		Uart_Handle->RxBuffSize = 0;
		Uart_Handle->pRxBuff = NULL;
		Uart_Handle->RxStatus = UART_BUS_FREE;
		Uart_Handle->Instance->UART_CR1 &= ~(0x1 << UART_RxNEIE);

}



static void UART_SendWith_IT(UART_Handle_Typedef_t*	Uart_Handle)
{
    if ((Uart_Handle->Init.WordLength == UART_WORDLEN_9B) && (Uart_Handle->Init.parity == UART_PARITY_NONE))
    {
    	uint16_t* pData16B = (uint16_t*)(Uart_Handle->pTxBuff);
    	Uart_Handle->Instance->UART_TDR = (uint16_t)(*pData16B & 0x1FFU);
    	Uart_Handle->pTxBuff += sizeof(uint16_t);
    	Uart_Handle->TxBuffSize --;

    }

    else{

    	Uart_Handle->Instance->UART_TDR = (uint16_t)(*(Uart_Handle->pTxBuff) & 0x0FFU);
    	Uart_Handle->pTxBuff += sizeof(uint8_t);
    	Uart_Handle->TxBuffSize --;

    }


	if(Uart_Handle->TxBuffSize == 0) UART_CloseISR(Uart_Handle);


}

static void UART_ReceiveWith_IT(UART_Handle_Typedef_t*	huart)
{
		uint16_t* pData16B;
		uint8_t* pData8B;
		if((huart->Init.WordLength == UART_WORDLEN_9B) && (huart->Init.parity == UART_PARITY_NONE)){
			pData16B=(uint16_t*)(huart->pRxBuff);
			pData8B=NULL;

		}
		else{
			pData8B=(uint8_t*)(huart->pRxBuff);
			pData16B=NULL;
		}

		if(pData8B==NULL){

			*pData16B = (uint16_t)huart->Instance->UART_RDR;
			huart->RxBuffSize--;
			huart->pRxBuff++;
		}

		else{
			if((huart->Init.WordLength == UART_WORDLEN_9B) && (huart->Init.parity != UART_PARITY_NONE)){

				*pData8B = (uint8_t)(huart->Instance->UART_RDR & 0x00FFU);
				huart->RxBuffSize--;
				huart->pRxBuff++;
				}
			else if((huart->Init.WordLength == UART_WORDLEN_8B) && (huart->Init.parity == UART_PARITY_NONE))

			{
				*pData8B = (uint8_t)(huart->Instance->UART_RDR & 0x00FFU);
				huart->RxBuffSize--;
				huart->pRxBuff++;
			}
			else{

				*pData8B = (uint8_t)(huart->Instance->UART_RDR & 0x007FU);
				huart->RxBuffSize--;
				huart->pRxBuff++;
			}


		}
	if(huart->RxBuffSize==0) CloseISR_Rx(huart);

}



/**
  * @brief  SPUART_Init configure the UART periphs.
  * @param  UART_Handle_Typedef_t = User config struct.
  * @retval none.
  */

void UART_Init(UART_Handle_Typedef_t*	Uart_Handle)
{
	  uint32_t tempReg=0;
	  uint16_t brrtemp;
	  UART_ClockSourceTypeDef clocksource;
	  uint32_t usartdiv;
	  uint32_t pclk;
	/***********Mode, Word length, Oversampling Mode,Parity Controls*******************/


	tempReg=Uart_Handle->Instance->UART_CR1;
	tempReg |= (Uart_Handle->Init.mode) | (Uart_Handle->Init.oversampling) |(Uart_Handle->Init.WordLength) \
			| (Uart_Handle->Init.parity);

	Uart_Handle->Instance->UART_CR1 = tempReg;
	/***********Stop Bits Control*******************/

	tempReg = Uart_Handle->Instance->UART_CR2;
	tempReg &= ~(0x3 << 12U);
	tempReg |= Uart_Handle->Init.StopBits;
	Uart_Handle->Instance->UART_CR2 = tempReg;

	/**********Hardware Flow Control****************/

	tempReg = Uart_Handle->Instance->UART_CR3;
	tempReg |= Uart_Handle->Init.HardwareFlowControl;
	Uart_Handle->Instance->UART_CR3 = tempReg;

	/**********Baud Rate Config****************/

	 UART_GETCLOCKSOURCE(Uart_Handle, clocksource);

	  /* Check LPUART instance */
	  if (Uart_Handle->Instance == LPUART)
	  {
	    /* Retrieve frequency clock */
	    switch (clocksource)
	    {
	      case UART_CLOCKSOURCE_PCLK1:
	        pclk = RCC_GetPCLK1Freq();
	        break;
	      case UART_CLOCKSOURCE_HSI:
	        if (IS_HSI_DIV())
	        {
	          pclk = (uint32_t)(HSI_VALUE >> 2U);
	        }
	        else
	        {
	          pclk = (uint32_t) HSI_VALUE;
	        }
	        break;
	      case UART_CLOCKSOURCE_SYSCLK:
	        pclk = RCC_GetSysClockFreq();
	        break;
	      case UART_CLOCKSOURCE_LSE:
	        pclk = (uint32_t) LSE_VALUE;
	        break;
	      default:
	        pclk = 0U;
	        break;
	    }

	    /* If proper clock source reported */
	    if (pclk != 0U)
	    {
	      /* Ensure that Frequency clock is in the range [3 * baudrate, 4096 * baudrate] */
	      if ((pclk > (3U * Uart_Handle->Init.baudRate)) ||
	          (pclk < (4096U * Uart_Handle->Init.baudRate)))
	      {
	        usartdiv = (uint32_t)(UART_DIV_LPUART(pclk, Uart_Handle->Init.baudRate));
	        if ((usartdiv >= 0x300U) && (usartdiv <= 0xFFFFFU))
	        {
	        	Uart_Handle->Instance->UART_BRR = usartdiv;
	        }

	      }
	    }
	  }

	  else if (Uart_Handle->Init.oversampling == UART_OVERSAMP_8)
	  {
	    switch (clocksource)
	    {
	      case UART_CLOCKSOURCE_PCLK1:
	        pclk = RCC_GetPCLK1Freq();
	        break;
	      case UART_CLOCKSOURCE_PCLK2:
	        pclk = RCC_GetPCLK2Freq();
	        break;
	      case UART_CLOCKSOURCE_HSI:
	        if (IS_HSI_DIV())
	        {
	          pclk = (uint32_t)(HSI_VALUE >> 2U);
	        }
	        else
	        {
	          pclk = (uint32_t) HSI_VALUE;
	        }
	        break;
	      case UART_CLOCKSOURCE_SYSCLK:
	        pclk = RCC_GetSysClockFreq();
	        break;
	      case UART_CLOCKSOURCE_LSE:
	        pclk = (uint32_t) LSE_VALUE;
	        break;
	      default:
	        pclk = 0U;
	        break;
	    }

	    /* USARTDIV must be greater than or equal to 0d16 */
	    if (pclk != 0U)
	    {
	      usartdiv = (uint16_t)(UART_DIV_SAMPLING8(pclk, Uart_Handle->Init.baudRate));
	      if ((usartdiv >= 0x300) && (usartdiv <= 0xFFFFFU))
	      {
	        brrtemp = (uint16_t)(usartdiv & 0xFFF0U);
	        brrtemp |= (uint16_t)((usartdiv & (uint16_t)0x000FU) >> 1U);
	        Uart_Handle->Instance->UART_BRR = brrtemp;
	      }

	    }
	  }
	  else
	  {
	    switch (clocksource)
	    {
	      case UART_CLOCKSOURCE_PCLK1:
	        pclk = RCC_GetPCLK1Freq();
	        break;
	      case UART_CLOCKSOURCE_PCLK2:
	        pclk = RCC_GetPCLK2Freq();
	        break;
	      case UART_CLOCKSOURCE_HSI:
	        if (IS_HSI_DIV())
	        {
	          pclk = (uint32_t)(HSI_VALUE >> 2U);
	        }
	        else
	        {
	          pclk = (uint32_t) HSI_VALUE;
	        }
	        break;
	      case UART_CLOCKSOURCE_SYSCLK:
	        pclk = RCC_GetSysClockFreq();
	        break;
	      case UART_CLOCKSOURCE_LSE:
	        pclk = (uint32_t) LSE_VALUE;
	        break;
	      default:
	        pclk = 0U;
	        break;
	    }

	    if (pclk != 0U)
	    {
	      /* USARTDIV must be greater than or equal to 0d16 */
	      usartdiv = (uint16_t)(UART_DIV_SAMPLING16(pclk, Uart_Handle->Init.baudRate));
	      if ((usartdiv >= 0x1) && (usartdiv <= 0xFFFFFU))
	      {
	    	  Uart_Handle->Instance->UART_BRR = usartdiv;
	      }

	    }
	  }



}

/**
  * @brief  UART_GET_FLAG_STATUS = returns the flag of ISR register.
  * @param  UART_Handle_Typedef_t = User config struct.
  * @param  Flag = Flag name of ISR register.
  * @retval UART_FlagStatus_t .
  */
UART_FlagStatus_t UART_GET_FLAG_STATUS(UART_Handle_Typedef_t* UART_Handle,uint16_t Flag)
{

	return ((UART_Handle->Instance->UART_ISR & Flag) ?  UART_FLAG_SET : UART_FLAG_RESET );
}
/**
  * @brief  UART_TransmitData , Transmits data .
  * @param  UART_Handle_Typedef_t = User config struct.
  * @param  pData = Address of data to be sent
  * @param  size = size of data in bytes.
  * @retval none.
  */

void UART_TransmitData(UART_Handle_Typedef_t *huart, uint8_t *pData, uint16_t size)
{
	uint8_t  *pdata8bits;
	uint16_t *pdata16bits;
    if ((huart->Init.WordLength == UART_WORDLEN_9B) && (huart->Init.parity == UART_PARITY_NONE))
    {
      pdata8bits  = NULL;
      pdata16bits = (uint16_t *) pData;
    }
    else
    {
      pdata8bits  = pData;
      pdata16bits = NULL;
    }


    while (size > 0U)
    {
      while(!UART_GET_FLAG_STATUS(huart,UART_TxE_Flag));
      if (pdata8bits == NULL)
      {
        huart->Instance->UART_TDR = (uint16_t)(*pdata16bits & 0x01FFU);
        pdata16bits++;
      }
      else
      {
        huart->Instance->UART_TDR = (uint8_t)(*pdata8bits & 0xFFU);
        pdata8bits++;
      }
     size--;
    }
    while(!UART_GET_FLAG_STATUS(huart,UART_TC_Flag));
  }

void UART_TransmitData_IT(UART_Handle_Typedef_t *huart, uint8_t *pData, uint16_t size)
{
	UART_BusState_t state=huart->TxStatus;
	if(state != UART_BUS_Tx){

		huart->pTxBuff = (uint8_t*)pData;
		huart->TxStatus = UART_BUS_Tx;
		huart->TxBuffSize = size;
		huart->TxISR_Func =UART_SendWith_IT;
		huart->Instance->UART_CR1 |= (0x1U << UART_TxEIE);
	}
}

void UART_ReceiveData_IT(UART_Handle_Typedef_t* huart,uint8_t* pDataBuff,uint16_t size)
{
	UART_BusState_t status = huart->RxStatus;
	if(status != UART_BUS_Rx)
	{
		huart->RxBuffSize = size;
		huart->pRxBuff = (uint8_t*)pDataBuff;
		huart->RxStatus = UART_BUS_Rx;
		huart->RxISR_Func = UART_ReceiveWith_IT;

		huart->Instance->UART_CR1 |= (0x1U << UART_RxNEIE);

	}


}

/**
  * @brief  UART_ReceiveData , Receive data .
  * @param  UART_Handle_Typedef_t = User config struct.
  * @param  pDataBuff = Address of buffer to be stored
  * @param  size = size of data in bytes.
  * @retval none.
  */
void UART_ReceiveData(UART_Handle_Typedef_t *huart, uint8_t *pDataBuff, uint16_t size){

	uint16_t* pData16B;
	uint8_t* pData8B;
	if((huart->Init.WordLength == UART_WORDLEN_9B) && (huart->Init.parity == UART_PARITY_NONE)){
		pData16B=(uint16_t*)pDataBuff;
		pData8B=NULL;

	}
	else{
		pData8B=(uint8_t*)pDataBuff;
		pData16B=NULL;
	}
	while(size>0){

		while(!UART_GET_FLAG_STATUS(huart,UART_RxNE_Flag));
		if(pData8B==NULL) {
			*pData16B = (uint16_t)(huart->Instance->UART_RDR & 0x01FFU);
			pData16B++;
			size--;
		}

		else{

			if((huart->Init.WordLength == UART_WORDLEN_9B) && (huart->Init.parity != UART_PARITY_NONE))
									*pData8B = (uint8_t)(huart->Instance->UART_RDR & 0x0FFU);

			else if((huart->Init.WordLength == UART_WORDLEN_8B) && (huart->Init.parity == UART_PARITY_NONE))
									*pData8B = (uint8_t)(huart->Instance->UART_RDR & 0x0FFU);
			else					*pData8B = (uint8_t)(huart->Instance->UART_RDR & 0x07FU);
			pData8B++;
			size--;

		}

	}
}

/**
  * @brief  Enable Or Disable UART periphs.
  * @param  UART_Handle_Typedef_t = User config struct.
  * @param  status = Enable or Disable.
  * @retval none.
  */
void UART_Periph_Cmd(UART_Handle_Typedef_t* UART_Handle,FunctionalState_t status)
{
	if(status==ENABLE) {
		UART_Handle->Instance->UART_CR1 |= UART_UE;
		//while(!UART_GET_FLAG_STATUS(UART_Handle, UART_TEACK_Flag));
	}
	else  UART_Handle->Instance->UART_CR1 &= ~UART_UE;

}

void UART_Interrupt_Handler(UART_Handle_Typedef_t* huart)
{
	uint8_t flag=0;
	uint8_t interrupt=0;
	flag= (uint8_t)((huart->Instance->UART_ISR >> 7U) & 0x1U);
	interrupt = (uint8_t)((huart->Instance->UART_CR1 >> 7U) & 0x1U);
	if(flag && interrupt){
		huart->TxISR_Func(huart);
	}
	flag = (uint8_t)((huart->Instance->UART_ISR >> 5U) & 0x1U);
	interrupt = (uint8_t)((huart->Instance->UART_CR1 >> 5U) & 0x1U);
	if(flag && interrupt){
			huart->RxISR_Func(huart);
		}

}
