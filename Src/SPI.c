/*
 * SPI.c
 *
 *  Created on: 5 Ağu 2021
 *      Author: win10
 */
#include "SPI.h"



/**
  * @brief  SPI_ISR_Close_Rx disables the interrupt for Reception.
  * @param  SPI_Handle = User config struct.
  * @retval none.
  */

static void SPI_ISR_Close_Rx(SPI_Handle_Typedef_t* SPI_handle){
	SPI_handle->Instance->SPI_CR2 &= ~(0x1U << SPI_CR2_RXNEIE);
	SPI_handle->RxDataSize=0;
	SPI_handle->RxPdataAddr=NULL;
	SPI_handle->BusStateRx=SPI_BUS_FREE;
}

/**
  * @brief  SPI_ISR_Close_Tx disables the interrupt for Transmission.
  * @param  SPI_Handle = User config struct.
  * @retval none.
  */

static void SPI_ISR_Close_Tx(SPI_Handle_Typedef_t* SPI_handle){
	SPI_handle->Instance->SPI_CR2 &= ~(0x1U << SPI_CR2_TXEIE);
	SPI_handle->DataSize=0;
	SPI_handle->pDataAddr=NULL;
	SPI_handle->BusStateTx=SPI_BUS_FREE;
}

/**
  * @brief  SPI_TransmitHelper_16B stores the users data into the DR register.
  * @param  SPI_Handle = User config struct.
  * @retval none.
  */
static void SPI_TransmitHelper_16B(SPI_Handle_Typedef_t* SPI_handle){
	SPI_handle->Instance->SPI_DR = *((uint16_t*)(SPI_handle->pDataAddr));
	SPI_handle->pDataAddr += sizeof(uint16_t);
	SPI_handle->DataSize -=2;
	if(SPI_handle->DataSize==0) SPI_ISR_Close_Tx(SPI_handle);
}


/**
  * @brief  SPI_TransmitHelper_8B stores the users data into the DR register.
  * @param  SPI_Handle = User config struct.
  * @retval none.
  */

static void SPI_TransmitHelper_8B(SPI_Handle_Typedef_t* SPI_handle){
		SPI_handle->Instance->SPI_DR = *((uint8_t*)(SPI_handle->pDataAddr));
		SPI_handle->pDataAddr += sizeof(uint8_t);
		SPI_handle->DataSize --;
		if(SPI_handle->DataSize==0) SPI_ISR_Close_Tx(SPI_handle);
}

/**
  * @brief  SPI_ReceiveHelper_8B reads the DR register and stores into the users variable.
  * @param  SPI_Handle = User config struct.
  * @retval none.
  */

static void SPI_ReceiveHelper_8B(SPI_Handle_Typedef_t* SPI_handle){

	*((uint8_t *)SPI_handle->RxPdataAddr) = *((__IO uint8_t*)&SPI_handle->Instance->SPI_DR);
	SPI_handle->RxPdataAddr += (sizeof(uint8_t));
	SPI_handle->RxDataSize --;
	if(SPI_handle->RxDataSize==0) SPI_ISR_Close_Rx(SPI_handle);

}

/**
  * @brief  SPI_ReceiveHelper_16B reads the DR register and stores into the users variable.
  * @param  SPI_Handle = User config struct.
  * @retval none.
  */

static void SPI_ReceiveHelper_16B(SPI_Handle_Typedef_t* SPI_handle){

	*((uint16_t *)SPI_handle->RxPdataAddr) = *((__IO uint16_t*)&SPI_handle->Instance->SPI_DR);
	SPI_handle->RxPdataAddr += (sizeof(uint16_t));
	SPI_handle->RxDataSize -=2;
	if(SPI_handle->RxDataSize==0) SPI_ISR_Close_Rx(SPI_handle);
}
/**
  * @brief  SPI Init configure the SPI periphs.
  * @param  SPI_Handle = User config struct.
  * @retval none.
  */

void SPI_Init(SPI_Handle_Typedef_t* SPI_Handle)
{

	uint32_t temp=0;
	temp=SPI_Handle->Instance->SPI_CR1;
	temp |= (SPI_Handle->Init.BaudRate) | (SPI_Handle->Init.CPHA) | (SPI_Handle->Init.CPOL) | (SPI_Handle->Init.DFF) \
			| (SPI_Handle->Init.Mode) | (SPI_Handle->Init.FrameFormat) | (SPI_Handle->Init.BusConfig) | (SPI_Handle->Init.SSM_Cmd);

	SPI_Handle->Instance->SPI_CR1 = temp;


}

/**
  * @brief  Enable Or Disable SPI periphs.
  * @param  SPI_Handle = User config struct.
  * @param  status = Enable or Disable.

  * @retval none.
  */
void SPI_PeriphCmd(SPI_Handle_Typedef_t* SPI_Handle,FunctionalState_t status)
{

	if(status==ENABLE)		SPI_Handle->Instance->SPI_CR1 |= (0x1U << SPI_CR1_SPE);
	else 					SPI_Handle->Instance->SPI_CR1 &= ~(0x1U << SPI_CR1_SPE);
}


/**
  * @brief  SPI_TransmitData , Transmits data to the slave.
  * @param  SPI_Handle = User config struct.
  * @param  pData = Address of data to be sent
  * @retval none.
  */

void SPI_TransmitData(SPI_Handle_Typedef_t* SPI_Handle,uint8_t *pData,uint16_t size)
{

	if(SPI_Handle->Init.DFF==SPI_DFF_16BIT){

		while(size>0){

			if(SPI_GetFlagStatus(SPI_Handle,SPI_TxE_FLAG)){
				SPI_Handle->Instance->SPI_DR= *((uint16_t*)pData);
				pData+=sizeof(uint16_t);
				size-=2;

			}
		}
	}
	else{

		while(size>0){

				if((SPI_Handle->Instance->SPI_SR >> 0x1U) && 0x1U){
					SPI_Handle->Instance->SPI_DR= *pData;
					pData++;
					size--;
				}


			}
	}
	while(SPI_GetFlagStatus(SPI_Handle,SPI_Busy_FLAG));//wait for busy flag

}

/**
  * @brief  SPI_ReceiveData , Receive data From the slave.
  * @param  SPI_Handle = User config struct.
  * @param  pBuffer = Address of data to be receive.
  * @retval none.
  */

void SPI_ReceiveData(SPI_Handle_Typedef_t* SPI_Handle,uint8_t *pBuffer,uint16_t size)
{

	if(SPI_Handle->Init.DFF==SPI_DFF_16BIT){

		while(size>0){

					if(SPI_GetFlagStatus(SPI_Handle,SPI_RXNE_FLAG)){
						*((uint16_t*)pBuffer)=(uint16_t)SPI_Handle->Instance->SPI_DR;
						pBuffer += sizeof(uint16_t);
						size -= 2;
					}
				  }
				}

	else{

			while(size>0){

						if(SPI_GetFlagStatus(SPI_Handle,SPI_RXNE_FLAG)){
							*(pBuffer)=*((__IO uint8_t*)&SPI_Handle->Instance->SPI_DR);
							pBuffer += sizeof(uint8_t);
							size -= 1;
						}
					  }
					}

}

/**
  * @brief  SPI_GetFlagStatus , returns the flag of SR register.
  * @param  SPI_Handle = User config struct.
  * @param  SPI_Flag = Flag name of SR register.
  * @retval SPI_FlagStatus_t .
  */


SPI_FlagStatus_t SPI_GetFlagStatus(SPI_Handle_Typedef_t* SPI_Handle,uint16_t SPI_Flag)
{
	return (SPI_Handle->Instance->SPI_SR & SPI_Flag) ? SPI_FLAG_SET : SPI_FLAG_RESET;
}

/**
  * @brief  SPI_TransmitData_IT send the data to external world with interrupt method.
  * @param  SPI_Handle = User config struct.
  * @param	pData= carries the users data.
  * @param  size = size of data to be sent.
  * @retval none.
  */

void SPI_TransmitData_IT(SPI_Handle_Typedef_t* SPI_Handle,uint8_t *pData,uint16_t size)
{
	SPI_Bus_Status_t SPI_bus_state = SPI_Handle->BusStateTx;

	if(SPI_bus_state != SPI_BUS_BUSY_Tx)
	{
	SPI_Handle->pDataAddr=pData;
	SPI_Handle->DataSize=size;
	SPI_Handle->Instance->SPI_CR2 |= (0x1U << SPI_CR2_TXEIE);
	SPI_Handle->BusStateTx = SPI_BUS_BUSY_Tx;

	if(SPI_Handle->Instance->SPI_CR1 & (0x1U << SPI_CR1_DFF)) SPI_Handle->TxISRFunc=SPI_TransmitHelper_16B;

	else SPI_Handle->TxISRFunc=SPI_TransmitHelper_8B;


}}
void SPI_Interrupt_Handler(SPI_Handle_Typedef_t* SPI_Handle)
{
	uint8_t InterruptSource=0;
	uint8_t InterruptFlag=0;
	InterruptSource = SPI_Handle->Instance->SPI_CR2 & (0x1U << SPI_CR2_TXEIE);
	InterruptFlag =  SPI_Handle->Instance->SPI_SR & (0x1U << SPI_SR_Tx);
	if((InterruptFlag !=0) && (InterruptSource!=0))		SPI_Handle->TxISRFunc(SPI_Handle);
	InterruptSource = SPI_Handle->Instance->SPI_CR2 & (0x1U << SPI_CR2_RXNEIE);
	InterruptFlag =  SPI_Handle->Instance->SPI_SR & (0x1U << SPI_SR_RXNE);
	if((InterruptFlag !=0) && (InterruptSource!=0))		SPI_Handle->RxISRFunc(SPI_Handle);

}

/**
  * @brief  SPI_ReceiveData_IT reads the data from external world with interrupt method.
  * @param  SPI_Handle = User config struct.
  * @param	pBuffer= stores the data in this.
  * @param  size = size of data to be read.
  * @retval none.
  */

void SPI_ReceiveData_IT(SPI_Handle_Typedef_t* SPI_Handle,uint8_t *pBuffer,uint16_t size)
{
	SPI_Bus_Status_t busState = SPI_Handle->BusStateRx;
	if(busState != SPI_BUS_BUSY_Rx)
	{
		SPI_Handle->RxPdataAddr = (uint8_t*)pBuffer;
		SPI_Handle->RxDataSize = size;
		if((SPI_Handle->Instance->SPI_CR1) & (0x1U << SPI_CR1_DFF))
			{
			SPI_Handle->RxISRFunc = SPI_ReceiveHelper_16B;
			}
		else 	{
			SPI_Handle->RxISRFunc = SPI_ReceiveHelper_8B;
		}
		SPI_Handle->Instance->SPI_CR2 |= (0x1U << SPI_CR2_RXNEIE);
	}
}
