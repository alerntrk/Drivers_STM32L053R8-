
#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32l053.h"


typedef enum{
	UART_FLAG_RESET=0x0U,
	UART_FLAG_SET = !UART_FLAG_RESET

}UART_FlagStatus_t;

/**
  * @brief UART clock sources definition
  */
typedef enum
{
  UART_CLOCKSOURCE_PCLK1      = 0x00U,    /*!< PCLK1 clock source  */
  UART_CLOCKSOURCE_PCLK2      = 0x01U,    /*!< PCLK2 clock source  */
  UART_CLOCKSOURCE_HSI        = 0x02U,    /*!< HSI clock source    */
  UART_CLOCKSOURCE_SYSCLK     = 0x04U,    /*!< SYSCLK clock source */
  UART_CLOCKSOURCE_LSE        = 0x08U,    /*!< LSE clock source       */
  UART_CLOCKSOURCE_UNDEFINED  = 0x10U     /*!< Undefined clock source */
} UART_ClockSourceTypeDef;

/*
 *
 * @def_group UART_Modes
 *
 */
#define UART_MODE_Tx						((uint32_t)(0x0008))
#define UART_MODE_Rx						((uint32_t)(0x0004))
#define UART_MODE_TxRx						((uint32_t)(0x000C))
/*
 *
 * @def_group WordLengths
 *
 */
#define UART_WORDLEN_8B						((uint32_t)(0x0000))
#define UART_WORDLEN_9B						((uint32_t)(0x1000))

/*
 *
 * @def_group Parity
 *
 */
#define UART_PARITY_NONE					((uint32_t)(0x0000))
#define UART_PARITY_EVEN					((uint32_t)(0x0400))
#define UART_PARITY_ODD						((uint32_t)(0x0600))
/*
 *
 * @def_group StopBits
 */
#define UART_STOP_BITS_1					((uint32_t)(0x0000))
#define UART_STOP_BITS_HALF					((uint32_t)(0x1000))
#define UART_STOP_BITS_2					((uint32_t)(0x2000))
#define UART_STOP_BITS_1_HALF				((uint32_t)(0x3000))

/*
 *
 * @def_group OverSampling
 *
 */
#define UART_OVERSAMP_16					((uint32_t)(0x0000))
#define UART_OVERSAMP_8						((uint32_t)(0x8000))

/*
 *
 * @def_group HardwareFlowCr
 *
 */
#define UART_HW_NONE						((uint32_t)(0x0000))
#define UART_HW_CTS							((uint32_t)(0x0200))
#define UART_HW_RTS							((uint32_t)(0x0100))
#define UART_HW_CRTS						((uint32_t)(0x0300))

typedef struct{

	uint32_t oversampling;					/* ! OverSampling modes 	@def_group OverSampling			 .*/
	uint32_t  baudRate;
	uint32_t WordLength;					/* ! 8 & 9 bits  modes @def_group WordLengths				 .*/
	uint32_t parity;						/* ! Even and Odd parity modes 	@def_group Parity			 .*/
	uint32_t StopBits;						/* ! Stop Bits modes 	@def_group StopBits			 .*/
	uint32_t HardwareFlowControl;			/* ! Hardware Flow  modes	@def_group HardwareFlowCr			 .*/
	uint32_t mode;							/* ! Transmission and reception modes @def_group UART_Modes .*/

}UART_Init_Typedef_t;

typedef struct __UART_Handle_Typedef_t{
	UART_Init_Typedef_t Init;
	UART_Typedef_t* Instance;
	uint8_t* pTxBuff;
	uint16_t TxBuffSize;
	uint8_t TxStatus;
	void(*TxISR_Func)(struct __UART_Handle_Typedef_t* Uart_handle);
	uint8_t* pRxBuff;
	uint16_t RxBuffSize;
	uint8_t RxStatus;
	void(*RxISR_Func)(struct __UART_Handle_Typedef_t* Uart_handle);

}UART_Handle_Typedef_t;


#define UART_GETCLOCKSOURCE(__HANDLE__,__CLOCKSOURCE__)       \
  do {                                                        \
    if((__HANDLE__)->Instance == USART1)                      \
    {                                                         \
      switch(RCC_GET_USART1_SOURCE())               	      \
      {                                                       \
        case RCC_USART1CLKSOURCE_PCLK2:                       \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_PCLK2;         \
          break;                                              \
        case RCC_USART1CLKSOURCE_HSI:                         \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_HSI;           \
          break;                                              \
        case RCC_USART1CLKSOURCE_SYSCLK:                      \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_SYSCLK;        \
          break;                                              \
        case RCC_USART1CLKSOURCE_LSE:                         \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_LSE;           \
          break;                                              \
        default:                                              \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_UNDEFINED;     \
          break;                                              \
      }                                                       \
    }                                                         \
    else if((__HANDLE__)->Instance == USART2)                 \
    {                                                         \
      switch(RCC_GET_USART2_SOURCE())                        \
      {                                                       \
        case RCC_USART2CLKSOURCE_PCLK1:                       \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_PCLK1;         \
          break;                                              \
        case RCC_USART2CLKSOURCE_HSI:                         \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_HSI;           \
          break;                                              \
        case RCC_USART2CLKSOURCE_SYSCLK:                      \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_SYSCLK;        \
          break;                                              \
        case RCC_USART2CLKSOURCE_LSE:                         \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_LSE;           \
          break;                                              \
        default:                                              \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_UNDEFINED;     \
          break;                                              \
      }                                                       \
    }                                                         \
    else if((__HANDLE__)->Instance == LPUART)                 \
    {                                                         \
      switch(RCC_GET_LPUART1_SOURCE())                        \
      {                                                       \
        case RCC_LPUART1CLKSOURCE_PCLK1:                      \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_PCLK1;         \
          break;                                              \
        case RCC_LPUART1CLKSOURCE_HSI:                        \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_HSI;           \
          break;                                              \
        case RCC_LPUART1CLKSOURCE_SYSCLK:                     \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_SYSCLK;        \
          break;                                              \
        case RCC_LPUART1CLKSOURCE_LSE:                        \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_LSE;           \
          break;                                              \
        default:                                              \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_UNDEFINED;     \
          break;                                              \
      }                                                       \
    }                                                         \
    else                                                      \
    {                                                         \
      (__CLOCKSOURCE__) = UART_CLOCKSOURCE_UNDEFINED;         \
    }                                                         \
  } while(0)

/** @brief  BRR division operation to set BRR registers with LPUART and USART.
  * @param  __PCLK__ LPUART clock.
  * @param  __BAUD__ Baud rate set by the user.
  * @retval Division result
  */
#define UART_DIV_LPUART(__PCLK__, __BAUD__)      (((((uint64_t)(__PCLK__)*256U)) + ((__BAUD__)/2U)) / (__BAUD__))
#define UART_DIV_SAMPLING8(__PCLK__, __BAUD__)   ((((__PCLK__)*2U) + ((__BAUD__)/2U)) / (__BAUD__))
#define UART_DIV_SAMPLING16(__PCLK__, __BAUD__)  (((__PCLK__) + ((__BAUD__)/2U)) / (__BAUD__))

typedef enum{
	UART_BUS_FREE=0x0U,
	UART_BUS_Tx = 0x1U,
	UART_BUS_Rx = 0x2U
}UART_BusState_t;



void UART_Init(UART_Handle_Typedef_t*);
void UART_TransmitData(UART_Handle_Typedef_t*,uint8_t*,uint16_t);
void UART_TransmitData_IT(UART_Handle_Typedef_t*,uint8_t*,uint16_t);
void UART_ReceiveData_IT(UART_Handle_Typedef_t*,uint8_t*,uint16_t);
UART_FlagStatus_t UART_GET_FLAG_STATUS(UART_Handle_Typedef_t*,uint16_t);
void UART_Periph_Cmd(UART_Handle_Typedef_t*,FunctionalState_t);
void UART_Interrupt_Handler(UART_Handle_Typedef_t*);
#endif /* INC_LPUART_H_ */
