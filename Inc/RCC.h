#ifndef INC_RCC_H_
#define INC_RCC_H_

#include "stm32l053.h"



/* RCC IOPENR Peripherals clock control macro definitions*/
#define RCC_GPIOA_CLCK_ENABLE()			do{	uint32_t  temp=0;										\
											SET_BIT(RCC->RCC_IOPENR,RCC_IOPENR_IOPAEN);			    \
											temp=READ_PIN(RCC->RCC_IOPENR,RCC_IOPENR_IOPAEN);	    \
											UNUSED(temp);											\
								}while(0)

#define RCC_GPIOB_CLCK_ENABLE()			do{	uint32_t  temp=0;										\
											SET_BIT(RCC->RCC_IOPENR,RCC_IOPENR_IOPBEN);			    \
											temp=READ_PIN(RCC->RCC_IOPENR,RCC_IOPENR_IOPBEN);	    \
											UNUSED(temp);											\
								}while(0)


#define RCC_GPIOC_CLCK_ENABLE()			do{	uint32_t  temp=0;										\
											SET_BIT(RCC->RCC_IOPENR,RCC_IOPENR_IOPCEN);			    \
											temp=READ_PIN(RCC->RCC_IOPENR,RCC_IOPENR_IOPCEN);	    \
											UNUSED(temp);											\
								}while(0)



#define RCC_GPIOD_CLCK_ENABLE()			do{	uint32_t  temp=0;										\
											SET_BIT(RCC->RCC_IOPENR,RCC_IOPENR_IOPDEN);			    \
											temp=READ_PIN(RCC->RCC_IOPENR,RCC_IOPENR_IOPDEN);	    \
											UNUSED(temp);											\
								}while(0)

#define RCC_GPIOA_CLCK_DISABLE()			CLEAR_BIT(RCC->RCC_IOPENR,RCC_IOPENR_IOPAEN)
#define RCC_GPIOB_CLCK_DISABLE()			CLEAR_BIT(RCC->RCC_IOPENR,RCC_IOPENR_IOPBEN)
#define RCC_GPIOC_CLCK_DISABLE()			CLEAR_BIT(RCC->RCC_IOPENR,RCC_IOPENR_IOPCEN)
#define RCC_GPIOD_CLCK_DISABLE()			CLEAR_BIT(RCC->RCC_IOPENR,RCC_IOPENR_IOPDEN)


/* RCC SYSCFG Peripherals clock control macro definitions*/


#define RCC_SYSCFG_CLK_ENABLE()			do{	uint32_t temp=0;										\
											SET_BIT(RCC->RCC_APB2ENR,RCC_APB2ENR_SYSCFG);			\
											temp=READ_PIN(RCC->RCC_APB2ENR,RCC_APB2ENR_SYSCFG);		\
											UNUSED(temp);											\
 	 	 	 	 	 	 	 	 }while(0)

/* RCC SPI Peripherals clock control macro definitions*/

#define RCC_SPI1_CLK_ENABLE()			do{	uint32_t temp=0;										\
											SET_BIT(RCC->RCC_APB2ENR,RCC_APB2ENR_SPI1);				\
											temp=READ_PIN(RCC->RCC_APB2ENR,RCC_APB2ENR_SPI1);		\
											UNUSED(temp);											\
 	 	 	 	 	 	 	 	 }while(0)

#define RCC_SPI2_CLK_ENABLE()			do{	uint32_t temp=0;										\
											SET_BIT(RCC->RCC_APB1ENR,RCC_APB1ENR_SPI2);				\
											temp=READ_PIN(RCC->RCC_APB1ENR,RCC_APB1ENR_SPI2);		\
											UNUSED(temp);											\
 	 	 	 	 	 	 	 	 }while(0)
#define RCC_SPI1_CLCK_DISABLE()			CLEAR_BIT(RCC->RCC_IOPENR,RCC_APB2ENR_SPI1)

/* RCC USART Peripherals clock control macro definitions*/

#define RCC_USART2_CLK_ENABLE()			do{	uint32_t temp=0;										\
											SET_BIT(RCC->RCC_APB1ENR,RCC_APB1ENR_USART2);			\
											temp=READ_PIN(RCC->RCC_APB1ENR,RCC_APB1ENR_USART2);		\
											UNUSED(temp);											\
 	 	 	 	 	 	 	 	 }while(0)

#define RCC_USART4_CLK_ENABLE()			do{	uint32_t temp=0;										\
											SET_BIT(RCC->RCC_APB1ENR,RCC_APB1ENR_USART4);			\
											temp=READ_PIN(RCC->RCC_APB1ENR,RCC_APB1ENR_USART4);		\
											UNUSED(temp);											\
 	 	 	 	 	 	 	 	 }


#define RCC_USART2_CLCK_DISABLE()			CLEAR_BIT(RCC->RCC_IOPENR,RCC_APB1ENR_USART2)

/* RCC I2C1 Peripherals clock control macro definitions*/

#define RCC_I2C1_CLK_ENABLE()			do{	uint32_t temp=0;										\
											SET_BIT(RCC->RCC_APB1ENR,RCC_APB1ENR_I2C1);			\
											temp=READ_PIN(RCC->RCC_APB1ENR,RCC_APB1ENR_I2C1);		\
											UNUSED(temp);											\
 	 	 	 	 	 	 	 	 }


/*   Macro to get the Related instance clock source.
  * retval The clock source can be one of the following values:
  */
#define RCC_GET_USART1_SOURCE() 			((uint32_t)(READ_PIN(RCC->RCC_CCIPR,0x3U)))
#define RCC_GET_USART2_SOURCE() 			((uint32_t)(READ_PIN(RCC->RCC_CCIPR, RCC_CCIPR_USART2SEL)))
#define RCC_GET_LPUART1_SOURCE() 			((uint32_t)(READ_PIN(RCC->RCC_CCIPR, RCC_CCIPR_LPUART1SEL)))


/** @defgroup RCCEx_LPUART1_Clock_Source RCCEx LPUART1 Clock Source

  */
#define RCC_LPUART1CLKSOURCE_PCLK1        (0x00000000U)
#define RCC_LPUART1CLKSOURCE_SYSCLK       RCC_CCIPR_LPUART1SEL_0
#define RCC_LPUART1CLKSOURCE_HSI          RCC_CCIPR_LPUART1SEL_1
#define RCC_LPUART1CLKSOURCE_LSE          (RCC_CCIPR_LPUART1SEL_0 | RCC_CCIPR_LPUART1SEL_1)

/** @defgroup RCCEx_USART1_Clock_Source RCCEx USART1 Clock Source
  *
  */
#define RCC_USART1CLKSOURCE_PCLK2        (0x00000000U)
#define RCC_USART1CLKSOURCE_SYSCLK       RCC_CCIPR_USART1SEL_0
#define RCC_USART1CLKSOURCE_HSI          RCC_CCIPR_USART1SEL_1
#define RCC_USART1CLKSOURCE_LSE          (RCC_CCIPR_USART1SEL_0 | RCC_CCIPR_USART1SEL_1)

/** @defgroup RCCEx_USART2_Clock_Source RCCEx USART2 Clock Source

  */
#define RCC_USART2CLKSOURCE_PCLK1        (0x00000000U)
#define RCC_USART2CLKSOURCE_SYSCLK       RCC_CCIPR_USART2SEL_0
#define RCC_USART2CLKSOURCE_HSI          RCC_CCIPR_USART2SEL_1
#define RCC_USART2CLKSOURCE_LSE          (RCC_CCIPR_USART2SEL_0 | RCC_CCIPR_USART2SEL_1)

/** @defgroup RCC_System_Clock_Source_Status System Clock Source Status

  */
#define RCC_SYSCLKSOURCE_STATUS_MSI      RCC_CFGR_SWS_MSI            /*!< MSI used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSI      RCC_CFGR_SWS_HSI            /*!< HSI used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSE      RCC_CFGR_SWS_HSE            /*!< HSE used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK   RCC_CFGR_SWS_PLL            /*!< PLL used as system clock */

#define RCC_PLLSOURCE_HSI           RCC_CFGR_PLLSRC_HSI        /*!< HSI clock selected as PLL entry clock source */
#define RCC_PLLSOURCE_HSE           RCC_CFGR_PLLSRC_HSE        /*!< HSE clock selected as PLL entry clock source */

#define IS_HSI_DIV()					((RCC->RCC_CR & 48) ? 1:0)

#define RCC_GET_PLL_OSCSOURCE() ((uint32_t)(READ_PIN(RCC->RCC_CFGR, RCC_CFGR_PLLSRC)))
uint32_t RCC_GetSysClockFreq(void);


#endif /* INC_RCC_H_ */
