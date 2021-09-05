/*
 * SPI.h
 *
 *  Created on: 5 Ağu 2021
 *      Author: win10
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "stm32l053.h"

typedef enum{
	SPI_BUS_FREE=0x0U,
	SPI_BUS_BUSY_Tx=0x1U,
	SPI_BUS_BUSY_Rx=0x2U
}SPI_Bus_Status_t;

typedef enum{
	SPI_FLAG_RESET=0x0U,
	SPI_FLAG_SET = !SPI_FLAG_RESET

}SPI_FlagStatus_t;




/*
 * @def_group SPI_BAUD_RATES
 */
#define SPI_BAUDRATE_DIV2						(uint32_t)(0x0000)
#define SPI_BAUDRATE_DIV4						(uint32_t)(0x0008)
#define SPI_BAUDRATE_DIV8						(uint32_t)(0x0010)
#define SPI_BAUDRATE_DIV16						(uint32_t)(0x0018)
#define SPI_BAUDRATE_DIV32						(uint32_t)(0x0020)
#define SPI_BAUDRATE_DIV64						(uint32_t)(0x0028)
#define SPI_BAUDRATE_DIV128						(uint32_t)(0x0030)
#define SPI_BAUDRATE_DIV256					    (uint32_t)(0x0038)

/*
 * @def_group CPHA_Val
 */

#define SPI_CPHA_FIRST							(uint32_t)(0x0000)
#define SPI_CPHA_SECOND							(uint32_t)(0x0001)

/*
 *  @def_group CPOL_Val
 */
#define SPI_CPOL_LOW							(uint32_t)(0x0000)
#define SPI_CPOL_HIGH							(uint32_t)(0x0002)

/*
 * @def_group SPI_DATA_Frame
 */
#define SPI_DFF_8BIT						    (uint32_t)(0x0000)
#define SPI_DFF_16BIT						    (uint32_t)(0x0800)

/*
 *  @def_group MODE
 */
#define SPI_MODE_MASTER						    (uint32_t)(0x0004)
#define SPI_MODE_SLAVE					   	    (uint32_t)(0x0000)


/*
 * @def_group SPI_FRAME_FORM
 */
#define SPI_FF_MSB								(uint32_t)(0x0000)
#define SPI_FF_LSB								(uint32_t)(0x0080)

/*
 * @def_group SPI_BUS_CFG
 */

#define SPI_FullDuplex							(uint32_t)(0x0000)
#define SPI_ReceiveOnly							(uint32_t)(0x0400)
#define SPI_HalfDuplex_T						(uint32_t)(0xC000)
#define SPI_HalfDuplex_R						(uint32_t)(0x8000)

/*
 * @def_group SPI_SSM_CMD
 */
#define SPI_SSM_ENABLE							(uint32_t)(0x0300)
#define SPI_SSM_DISABLE							(uint32_t)(0x0000)


typedef struct {
	uint32_t Mode;					/* MODE Values For SPI @def_group MODE 					*/
	uint32_t CPHA;					/* CPHA Values For SPI @def_group CPHA_Val 				*/
	uint32_t CPOL;					/* CPOL Values For SPI @def_group CPOL_Val 				*/
	uint32_t BaudRate;				/* Baud Rate Values For SPI @def_group SPI_BAUD_RATES   */
	uint32_t SSM_Cmd;				/* SSM Values For SPI @def_group SPI_SSM_CMD  */
	uint32_t DFF;					/* Data Frame Format For SPI @def_group SPI_DATA_Frame  */
	uint32_t BusConfig;				/* Bus Config Values For SPI @def_group SPI_BUS_CFG	    */
	uint32_t FrameFormat;			/* Frame Format For SPI @def_group SPI_FRAME_FORM  		*/

}SPI_Init_Typedef_t;


typedef struct __SPI_Handle_Typedef_t{

	SPI_Typedef_t *Instance;
	SPI_Init_Typedef_t Init;
	uint8_t* pDataAddr;
	uint16_t DataSize;
	uint8_t BusStateTx;
	uint8_t BusStateRx;
	void(*TxISRFunc)(struct __SPI_Handle_Typedef_t*);
	uint8_t* RxPdataAddr;
	uint16_t RxDataSize;
	void(*RxISRFunc)(struct __SPI_Handle_Typedef_t*);
}SPI_Handle_Typedef_t;

void SPI_Init(SPI_Handle_Typedef_t* SPI_Handle);
void SPI_PeriphCmd(SPI_Handle_Typedef_t* SPI_Handle,FunctionalState_t status);
void SPI_TransmitData(SPI_Handle_Typedef_t* SPI_Handle,uint8_t *pData,uint16_t size);
void SPI_TransmitData_IT(SPI_Handle_Typedef_t* SPI_Handle,uint8_t *pData,uint16_t size);
void SPI_ReceiveData(SPI_Handle_Typedef_t* SPI_Handle,uint8_t *pBuffer,uint16_t size);
SPI_FlagStatus_t SPI_GetFlagStatus(SPI_Handle_Typedef_t* SPI_Handle,uint16_t SPI_Flag);
void SPI_Interrupt_Handler(SPI_Handle_Typedef_t* SPI_Handle);
void SPI_ReceiveData_IT(SPI_Handle_Typedef_t* SPI_Handle,uint8_t *pBuffer,uint16_t size);



#endif /* INC_SPI_H_ */
