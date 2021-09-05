/*
 * stm32l0xx.h
 *
 *  Created on: 13 Tem 2021
 *      Author: ALIEREN TURK
 */

#ifndef STM32L0XX_H_
#define STM32L0XX_H_
#include <stdint.h>
#define UNUSED(X)					(void)X
#define __IO 	volatile
#define SET_BIT(REG,BIT)			((REG)|=(BIT))
#define CLEAR_BIT(REG,BIT)			((REG)&=~(BIT))
#define READ_PIN(REG,BIT)			((REG)&(BIT))

#define HSI_VALUE    ((uint32_t)16000000U) 	/*!< Value of the Internal oscillator in Hz*/
#define HSE_VALUE    ((uint32_t)8000000U) 	/*!< Value of the External oscillator in Hz */
#define LSE_VALUE    ((uint32_t)32768U) 	/*!< Value of the External oscillator in Hz*/



typedef enum{
	DISABLE = 0x0,
	ENABLE= !DISABLE
}FunctionalState_t;
/*
 * IRQ numbers of MCU = vector table
 *
 */
typedef enum{
	EXTI_0_1_IRQ=5,
	EXTI_2_3_IRQ=6,
	EXTI_4_15_IRQ=7,
	SPI1_IRQ=25,
	USART2_IRQn=28
}IRQ_Number_Typedef_t;
/*
 * Memory base address
 *
 */
#define FLASH_BASE_ADDR				(0x08000000UL) 	//flash base address
#define SRAM1_BASE_ADDR				(0x20000000UL)	//sram address 8kb
/*
 * peripheral base addresses
 */

#define PERIPH_BASE_ADDR			(0x40000000UL)  				//all peripherals base address
#define APB1_BASE_ADDR				PERIPH_BASE_ADDR  				// APB1 bus domain base address
#define APB2_BASE_ADDR				(PERIPH_BASE_ADDR+0x10000UL) 	//APB2 bus domain base address
#define AHB_BASE_ADDR				(PERIPH_BASE_ADDR+0x20000UL)  	//AHB bus domain base address
#define IOPORT_BASE_ADDR			(PERIPH_BASE_ADDR+0x10000000UL)	//IOPORT bus domain base address

/*
 * APB1 peripherals base addresses
 */
#define TIM2_BASE_ADDR				APB1_BASE_ADDR     				//Timer2 base address
#define TIM3_BASE_ADDR				(APB1_BASE_ADDR+0x400UL)		//Timer3 base address
#define TIM6_BASE_ADDR				(APB1_BASE_ADDR+0x1000UL)		//Timer6 base address
#define SPI2_BASE_ADDR				(APB1_BASE_ADDR+0x3800UL)		//SPI2 base address
#define USART2_BASE_ADDR			(APB1_BASE_ADDR+0x4400UL)		//USART2 base address
#define USART4_BASE_ADDR			(APB1_BASE_ADDR+0x4C00UL)		//USART4 base address
#define LPUART_BASE_ADDR			(APB1_BASE_ADDR+0x4800UL)		//LPUART base address

#define I2C1_BASE_ADDR				(APB1_BASE_ADDR+0x5400UL)		//I2C1 base address
#define I2C2_BASE_ADDR				(APB1_BASE_ADDR+0x5800UL)		//I2C2 base address

/*
 * APB2 peripherals base addresses
 */
#define SYSCFG_BASE_ADDR					APB2_BASE_ADDR					// System config base adddres
#define COMP1_BASE_ADDR						(APB2_BASE_ADDR	+ 0x18)			//COMPARATOR 1 base address
#define EXTI_BASE_ADDR						(APB2_BASE_ADDR	+ 0x400)		//EXTI base address
#define TIM21_BASE_ADDR						(APB2_BASE_ADDR	+ 0x800)		//Timer21 base address
#define FIRWLL_BASE_ADDR					(APB2_BASE_ADDR	+ 0x1C00)		//FIREWALL base address
#define ADC1_BASE_ADDR						(APB2_BASE_ADDR	+ 0x2400)		//ADC1 base address
#define SPI1_BASE_ADDR						(APB2_BASE_ADDR	+ 0x3000)		//SPI1 base address
#define USART1_BASE_ADDR					(APB2_BASE_ADDR	+ 0x3800)		//USART1 base address
/*
 * AHB peripherals base addresses
 */
#define RCC_BASE_ADDR						(AHB_BASE_ADDR + 0x1000)			//RCC base address


/*
 *IOPORT peripherals base addresses
 */
#define GPIOA_BASE_ADDR						(IOPORT_BASE_ADDR)				//GPIOA base address
#define GPIOB_BASE_ADDR						(IOPORT_BASE_ADDR+0x400)		//GPIOB base address
#define GPIOC_BASE_ADDR						(IOPORT_BASE_ADDR+0x800)		//GPIOC base address
#define GPIOD_BASE_ADDR						(IOPORT_BASE_ADDR+0xC00)		//GPIOD base address

/*
 * NVIC base address
 */
#define NVIC_ISER							((uint32_t*)(0xE000E100))		//  NVIC base address

/*
 * Peripheral structure definitions
 */
typedef struct{
	__IO uint32_t MODER;					/*GPIO port mode register  				Address offset 0x0 */
	__IO uint32_t OTYPER;					/*GPIO port output type register  		Address offset 0x04*/
	__IO uint32_t OSPEEDR;					/*GPIO port output speed register  		Address offset 0x08*/
	__IO uint32_t PUPDR;					/*GPIO port pull-up/pull-down register 	Address offset 0x0C*/
	__IO uint32_t IDR;						/*GPIO port input data register		  	Address offset 0x10*/
	__IO uint32_t ODR;						/*GPIO port output data register  		Address offset 0x14*/
	__IO uint32_t BSRR;						/*GPIO port bit set/reset register  	Address offset 0x18*/
	__IO uint32_t LCKR;						/*GPIO port configuration lock register Address offset 0x1C*/
	__IO uint32_t AFR[2];					/*GPIO alternate functio register	    Address offset 0x20*/
	__IO uint32_t BRR;						/* GPIO port bit reset register 	 	Address offset 0x28*/
}GPIO_TypeDef_t;

#define GPIOA								((GPIO_TypeDef_t*)(GPIOA_BASE_ADDR))
#define GPIOB								((GPIO_TypeDef_t*)(GPIOB_BASE_ADDR))
#define GPIOC								((GPIO_TypeDef_t*)(GPIOC_BASE_ADDR))
#define GPIOD								((GPIO_TypeDef_t*)(GPIOD_BASE_ADDR))




typedef struct{
	__IO uint32_t RCC_CR;					/*Clock control register (RCC_CR)   	        		 	 Address offset 0x0*/
	__IO uint32_t RCC_ICSCR;				/*Internal clock sources calibration register 	 			 Address offset 0x4*/
	__IO uint32_t RCC_CRRCR;				/*Clock recovery RC register					 			 Address offset 0x8*/
	__IO uint32_t RCC_CFGR;					/*Clock configuration register					 			 Address offset 0xC*/
	__IO uint32_t RCC_CIER;					/*Clock interrupt enable register				 			 Address offset 0x10*/
	__IO uint32_t RCC_CIFR;					/*Clock interrupt flag register				    			 Address offset 0x14*/
	__IO uint32_t RCC_CICR;					/*Clock interrupt clear register							 Address offset 0x18*/
	__IO uint32_t RCC_IOPRSTR;				/*GPIO reset register				 			 			 Address offset 0x1C*/
	__IO uint32_t RCC_AHBRSTR;				/*AHB peripheral reset register  				 			 Address offset 0x20*/
	__IO uint32_t RCC_APB2RSTR;				/*APB2 peripheral reset register							 Address offset 0x24*/
	__IO uint32_t RCC_APB1RSTR;				/*APB1 peripheral reset register							 Address offset 0x28*/
	__IO uint32_t RCC_IOPENR;				/*GPIO clock enable register     							 Address offset 0x2C*/
	__IO uint32_t RCC_AHBENR;				/*AHB peripheral clock enable register			 			 Address offset 0x30*/
	__IO uint32_t RCC_APB2ENR;				/*APB2 peripheral clock enable register			 			 Address offset 0x34*/
	__IO uint32_t RCC_APB1ENR;				/*APB1 peripheral clock enable register			 			 Address offset 0x38*/
	__IO uint32_t RCC_IOPSMEN;				/*GPIO clock enable in Sleep mode register		 			 Address offset 0x3C*/
	__IO uint32_t RCC_AHBSMENR;				/*AHB peripheral clock enable in Sleep mode register  		 Address offset 0x40*/
	__IO uint32_t RCC_APB2SMENR;			/*APB2 peripheral clock enable in Sleep mode registerr		 Address offset 0x44*/
	__IO uint32_t RCC_APB1SMENR;			/*APB1 peripheral clock enable in Sleep mode register		 Address offset 0x48*/
	__IO uint32_t RCC_CCIPR;				/*Clock configuration register								 Address offset 0x4C*/
	__IO uint32_t RCC_CSR;					/*Control/status register				 					 Address offset 0x50*/
}RCC_Typedef_t;
#define RCC									((RCC_Typedef_t*)(RCC_BASE_ADDR))

typedef struct {
	__IO uint32_t SYSCFG_CFGR1;				/*SYSCFG memory remap register				 					 Address offset 0x0*/
	__IO uint32_t SYSCFG_CFGR2;				/*SYSCFG peripheral mode configuration register				 	 Address offset 0x4*/
	__IO uint32_t SYSCFG_EXTICR[4];			/*SYSCFG external interrupt configuration register			 	 Address offset 0x8*/
	__IO uint32_t COMP_CTRL[2];				/*Comparator 1 control and status register			 	 		 Address offset 0x18*/
	__IO uint32_t SYSCFG_CFGR3;				/*Reference control and status register			 		 		 Address offset 0x20*/

}SYSCFG_Typedef_t;

typedef struct{

	__IO uint32_t IMR;    					/*EXTI interrupt mask register				 					 Address offset 0x0*/
	__IO uint32_t EMR;						/*EXTI event mask register					 					 Address offset 0x4*/
	__IO uint32_t RTSR;						/*EXTI rising edge trigger selection register				 	 Address offset 0x8*/
	__IO uint32_t FTSR;						/*EXTI falling edge trigger selection register				 	 Address offset 0xC*/
	__IO uint32_t SWIER;					/*EXTI software interrupt event register			 			 Address offset 0x10*/
	__IO uint32_t PR;						/*EXTI pending register					 					     Address offset 0x14*/

}EXTI_Typedef_t;
#define EXTI								((EXTI_Typedef_t*)(EXTI_BASE_ADDR))
#define SYSCFG								((SYSCFG_Typedef_t*)(SYSCFG_BASE_ADDR))

typedef struct {

	__IO uint32_t SPI_CR1;					/*SPI control register 1 				 					 Address offset 0x00*/
	__IO uint32_t SPI_CR2;					/*SPI control register 2 				 					 Address offset 0x04*/
	__IO uint32_t SPI_SR;					/*SPI status register 				 						 Address offset 0x08*/
	__IO uint32_t SPI_DR;					/*SPI data register 				 						 Address offset 0x0C*/
	__IO uint32_t SPI_CRCPR;				/*SPI CRC polynomial register				 				 Address offset 0x10*/
	__IO uint32_t SPI_RXCRCR;				/*SPI RX CRC register				 				         Address offset 0x14*/
	__IO uint32_t SPI_TXCRCR;				/*SPI TX CRC register				 				         Address offset 0x18*/
	__IO uint32_t SPI_I2SCFGR;				/*SPI_I2S configuration register                             Address offset 0x1C*/
	__IO uint32_t SPI_I2SPR;				/*SPI_I2S prescaler register                                 Address offset 0x20*/

}SPI_Typedef_t;

#define SPI1								((SPI_Typedef_t*)(SPI1_BASE_ADDR))
#define SPI2								((SPI_Typedef_t*)(SPI2_BASE_ADDR))


typedef struct {
	__IO uint32_t UART_CR1;					/*Control register 1 				 					 Address offset 0x00*/
	__IO uint32_t UART_CR2;					/*Control register 2 				 					 Address offset 0x04*/
	__IO uint32_t UART_CR3;					/*Control register 3 				 					 Address offset 0x08*/
	__IO uint32_t UART_BRR;					/*Baud rate register				 					 Address offset 0x0C*/
	__IO uint32_t USART_GTPR;				/*Guard time and prescaler register				 		 Address offset 0x10*/
	__IO uint32_t USART_RTOR;				/*Receiver timeout register				 				 Address offset 0x14*/
	__IO uint32_t UART_RQR;				/*Request register				 				         Address offset 0x18*/
	__IO uint32_t UART_ISR;				/*Interrupt and status register                          Address offset 0x1C*/
	__IO uint32_t UART_ICR;				/*Interrupt flag clear register                          Address offset 0x20*/
	__IO uint32_t UART_RDR;				/*Receive data register                                  Address offset 0x24*/
	__IO uint32_t UART_TDR;				/*Transmit data register                                 Address offset 0x28*/

}UART_Typedef_t;
#define LPUART									((UART_Typedef_t*)(LPUART_BASE_ADDR))
#define USART1									((UART_Typedef_t*)(USART1_BASE_ADDR))
#define USART2									((UART_Typedef_t*)(USART2_BASE_ADDR))
#define USART4									((UART_Typedef_t*)(USART4_BASE_ADDR))


typedef struct {
	__IO uint32_t I2C_CR1;					/*Control register 1 				 					 Address offset 0x00*/
	__IO uint32_t I2C_CR2;					/*Control register 2 				 					 Address offset 0x04*/
	__IO uint32_t I2C_OAR1;					/*Own address 1 register				 				 Address offset 0x08*/
	__IO uint32_t I2C_OAR2;					/*Own address 2 register				 				 Address offset 0x0C*/
	__IO uint32_t I2C_TIMINGR;				/*Timing register							 		 	 Address offset 0x10*/
	__IO uint32_t I2C_TIMOUTR;				/*Timeout register			 				 			 Address offset 0x14*/
	__IO uint32_t I2C_ISR;					/*Interrupt and status register				 		     Address offset 0x18*/
	__IO uint32_t I2C_ICR;					/*Interrupt clear register 	                             Address offset 0x1C*/
	__IO uint32_t I2C_PECR;					/*PEC register			                         		 Address offset 0x20*/
	__IO uint32_t I2C_RXDR;					/*Receive data register                                  Address offset 0x24*/
	__IO uint32_t I2C_TXDR;					/*Transmit data register                                 Address offset 0x28*/

}I2C_Typedef_t;

#define I2C1							((I2C_Typedef_t*)(I2C1_BASE_ADDR))
#define I2C2							((I2C_Typedef_t*)(I2C2_BASE_ADDR))


/* BIT Definitions*/
#define RCC_IOPENR_IOPAEN_POS				(0x0U)													/*RCC IOPENR register IOPAEN bit position	*/
#define RCC_IOPENR_IOPAEN_MSK				(0x1 << RCC_IOPENR_IOPAEN_POS)							/*RCC IOPENR register IOPAEN bit mask		*/
#define RCC_IOPENR_IOPAEN				    (RCC_IOPENR_IOPAEN_MSK)									/*RCC IOPENR register IOPAEN macro			*/

#define RCC_IOPENR_IOPBEN_POS				(0x1U)													/*RCC IOPENR register IOPBEN bit position	*/
#define RCC_IOPENR_IOPBEN_MSK				(0x1 << RCC_IOPENR_IOPBEN_POS)							/*RCC IOPENR register IOPBEN bit mask		*/
#define RCC_IOPENR_IOPBEN				    (RCC_IOPENR_IOPBEN_MSK)									/*RCC IOPENR register IOPBEN macro			*/


#define RCC_IOPENR_IOPCEN_POS				(0x2U)													/*RCC IOPENR register IOPCEN bit position	*/
#define RCC_IOPENR_IOPCEN_MSK				(0x1 << RCC_IOPENR_IOPCEN_POS)							/*RCC IOPENR register IOPCEN bit mask		*/
#define RCC_IOPENR_IOPCEN				    (RCC_IOPENR_IOPCEN_MSK)									/*RCC IOPENR register IOPCEN macro          */

#define RCC_IOPENR_IOPDEN_POS				(0x3U)													/*RCC IOPENR register IOPDEN bit position   */
#define RCC_IOPENR_IOPDEN_MSK				(0x1 << RCC_IOPENR_IOPDEN_POS)							/*RCC IOPENR register IOPDEN bit mask       */
#define RCC_IOPENR_IOPDEN				    (RCC_IOPENR_IOPDEN_MSK)									/*RCC IOPENR register IOPDEN macro          */

#define RCC_APB2ENR_SYSCFG_Pos 				(0x0)													/*RCC APB2ENR register SYSCFG bit position */
#define RCC_APB2ENR_SYSCFG_Msk				(0x1 << RCC_APB2ENR_SYSCFG_Pos)							/*RCC APB2ENR register SYSCFG bit mask	   */
#define RCC_APB2ENR_SYSCFG					RCC_APB2ENR_SYSCFG_Msk									/*RCC APB2ENR register SYSCFG macro		   */

#define RCC_APB2ENR_SPI1_Pos				(0xC)													/*RCC APB2ENR register SPI1 bit position*/
#define RCC_APB2ENR_SPI1_Msk				(0x1 << RCC_APB2ENR_SPI1_Pos)							/*RCC APB2ENR register SPI1 bit Mask    */
#define RCC_APB2ENR_SPI1				    RCC_APB2ENR_SPI1_Msk									/*RCC APB2ENR register SPI1 bit Macro   */


#define RCC_APB1ENR_SPI2_Pos				(0xE)													/*RCC APB1ENR register SPI2 bit position*/
#define RCC_APB1ENR_SPI2_Msk				(0x1 << RCC_APB1ENR_SPI2_Pos)							/*RCC APB1ENR register SPI2 bit Mask    */
#define RCC_APB1ENR_SPI2				    RCC_APB1ENR_SPI2_Msk									/*RCC APB1ENR register SPI2 bit Macro   */

#define RCC_APB1ENR_USART2_Pos				(17U)													/*RCC APB1ENR register USART2 bit position*/
#define RCC_APB1ENR_USART2_Msk				(0x1 << RCC_APB1ENR_USART2_Pos)							/*RCC APB1ENR register USART2 bit Mask    */
#define RCC_APB1ENR_USART2				    RCC_APB1ENR_USART2_Msk									/*RCC APB1ENR register USART2 bit Macro   */

#define RCC_APB1ENR_USART4_Pos				(19U)													/*RCC APB1ENR register USART4 bit position*/
#define RCC_APB1ENR_USART4_Msk				(0x1 << RCC_APB1ENR_USART4_Pos)							/*RCC APB1ENR register USART4 bit Mask    */
#define RCC_APB1ENR_USART4				    RCC_APB1ENR_USART4_Msk									/*RCC APB1ENR register USART4 bit Macro   */

#define RCC_APB1ENR_LPUART_Pos				(18U)													/*RCC APB1ENR register LPUART bit position*/
#define RCC_APB1ENR_LPUART_Msk				(0x1 << RCC_APB1ENR_LPUART_Pos)							/*RCC APB1ENR register LPUART bit Mask    */
#define RCC_APB1ENR_LPUART				    RCC_APB1ENR_LPUART_Msk									/*RCC APB1ENR register LPUART bit Macro   */

#define RCC_APB1ENR_I2C1_Pos				(21U)													/*RCC APB1ENR register I2C1 bit position*/
#define RCC_APB1ENR_I2C1_Msk				(0x1 << RCC_APB1ENR_I2C1_Pos)							/*RCC APB1ENR register I2C1 bit Mask    */
#define RCC_APB1ENR_I2C1				    RCC_APB1ENR_I2C1_Msk									/*RCC APB1ENR register I2C1 bit Macro   */


#define SPI_CR1_SPE							(6U)
#define SPI_SR_BUSY							(7U)
#define SPI_SR_Tx							(1U)
#define SPI_SR_RXNE							(0U)
#define SPI_CR2_TXEIE						(7U)
#define SPI_CR2_RXNEIE						(6U)
#define SPI_CR1_DFF							(11U)
#define UART_TXE							(7U)
#define UART_TxEIE							(7U)
#define UART_RxNEIE							(5U)
#define UART_TC								(6U)
#define UART_TEACK							(021U)
#define UART_UE_Pos							(0U)
#define UART_UE								(0x1 << UART_UE_Pos)
#define UART_ISR_RxNE						(0x5U)
/* Flag Definitions*/
#define SPI_TxE_FLAG						(0x1U << SPI_SR_Tx)
#define SPI_Busy_FLAG						(0x1U << SPI_SR_BUSY)
#define SPI_RXNE_FLAG						(0x1U << SPI_SR_RXNE)
#define UART_TxE_Flag						(0x1 << UART_TXE)
#define UART_TC_Flag						(0x1 << UART_TC)
#define UART_TEACK_Flag						(0x1 << UART_TEACK)
#define UART_RxNE_Flag						(0x1 << UART_ISR_RxNE)

/* LPUART1 Clock source selection */
#define RCC_CCIPR_LPUART1SEL_Pos         (10U)
#define RCC_CCIPR_LPUART1SEL_Msk         (0x3UL << RCC_CCIPR_LPUART1SEL_Pos)    /*!< 0x00000C00 */
#define RCC_CCIPR_LPUART1SEL             RCC_CCIPR_LPUART1SEL_Msk              /*!< LPUART1SEL[1:0] bits */
#define RCC_CCIPR_LPUART1SEL_0           (0x1UL << RCC_CCIPR_LPUART1SEL_Pos)    /*!< 0x0000400 */
#define RCC_CCIPR_LPUART1SEL_1           (0x2UL << RCC_CCIPR_LPUART1SEL_Pos)    /*!< 0x0000800 */

/* USART1 Clock source selection */
#define RCC_CCIPR_USART1SEL_Pos          (0U)
#define RCC_CCIPR_USART1SEL_Msk          (0x3UL << RCC_CCIPR_USART1SEL_Pos)     /*!< 0x00000003 */
#define RCC_CCIPR_USART1SEL              RCC_CCIPR_USART1SEL_Msk               /*!< USART1SEL[1:0] bits */
#define RCC_CCIPR_USART1SEL_0            (0x1UL << RCC_CCIPR_USART1SEL_Pos)     /*!< 0x00000001 */
#define RCC_CCIPR_USART1SEL_1            (0x2UL << RCC_CCIPR_USART1SEL_Pos)     /*!< 0x00000002 */

/* USART2 Clock source selection */
#define RCC_CCIPR_USART2SEL_Pos          (2U)
#define RCC_CCIPR_USART2SEL_Msk          (0x3UL << RCC_CCIPR_USART2SEL_Pos)     /*!< 0x0000000C */
#define RCC_CCIPR_USART2SEL              RCC_CCIPR_USART2SEL_Msk               /*!< USART2SEL[1:0] bits */
#define RCC_CCIPR_USART2SEL_0            (0x1UL << RCC_CCIPR_USART2SEL_Pos)     /*!< 0x00000004 */
#define RCC_CCIPR_USART2SEL_1            (0x2UL << RCC_CCIPR_USART2SEL_Pos)     /*!< 0x00000008 */

/* HPRE configuration */
#define RCC_CFGR_HPRE_Pos                    (4U)
#define RCC_CFGR_HPRE_Msk                    (0xFUL << RCC_CFGR_HPRE_Pos)       /*!< 0x000000F0 */
#define RCC_CFGR_HPRE                        RCC_CFGR_HPRE_Msk                 /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_0                      (0x1UL << RCC_CFGR_HPRE_Pos)       /*!< 0x00000010 */
#define RCC_CFGR_HPRE_1                      (0x2UL << RCC_CFGR_HPRE_Pos)       /*!< 0x00000020 */
#define RCC_CFGR_HPRE_2                      (0x4UL << RCC_CFGR_HPRE_Pos)       /*!< 0x00000040 */
#define RCC_CFGR_HPRE_3                      (0x8UL << RCC_CFGR_HPRE_Pos)       /*!< 0x00000080 */

#define RCC_CFGR_HPRE_DIV1                   (0x00000000U)                     /*!< SYSCLK not divided */
#define RCC_CFGR_HPRE_DIV2                   (0x00000080U)                     /*!< SYSCLK divided by 2 */
#define RCC_CFGR_HPRE_DIV4                   (0x00000090U)                     /*!< SYSCLK divided by 4 */
#define RCC_CFGR_HPRE_DIV8                   (0x000000A0U)                     /*!< SYSCLK divided by 8 */
#define RCC_CFGR_HPRE_DIV16                  (0x000000B0U)                     /*!< SYSCLK divided by 16 */
#define RCC_CFGR_HPRE_DIV64                  (0x000000C0U)                     /*!< SYSCLK divided by 64 */
#define RCC_CFGR_HPRE_DIV128                 (0x000000D0U)                     /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256                 (0x000000E0U)                     /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512                 (0x000000F0U)                     /*!< SYSCLK divided by 512 */

/* PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos                   (8U)
#define RCC_CFGR_PPRE1_Msk                   (0x7UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000700 */
#define RCC_CFGR_PPRE1                       RCC_CFGR_PPRE1_Msk                /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_0                     (0x1UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000100 */
#define RCC_CFGR_PPRE1_1                     (0x2UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000200 */
#define RCC_CFGR_PPRE1_2                     (0x4UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000400 */

#define RCC_CFGR_PPRE1_DIV1                  (0x00000000U)                     /*!< HCLK not divided */
#define RCC_CFGR_PPRE1_DIV2                  (0x00000400U)                     /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE1_DIV4                  (0x00000500U)                     /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE1_DIV8                  (0x00000600U)                     /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE1_DIV16                 (0x00000700U)                     /*!< HCLK divided by 16 */

/* PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos                   (11U)
#define RCC_CFGR_PPRE2_Msk                   (0x7UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00003800 */
#define RCC_CFGR_PPRE2                       RCC_CFGR_PPRE2_Msk                /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2_0                     (0x1UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00000800 */
#define RCC_CFGR_PPRE2_1                     (0x2UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00001000 */
#define RCC_CFGR_PPRE2_2                     (0x4UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00002000 */

#define RCC_CFGR_PPRE2_DIV1                  (0x00000000U)                     /*!< HCLK not divided */
#define RCC_CFGR_PPRE2_DIV2                  (0x00002000U)                     /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE2_DIV4                  (0x00002800U)                     /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE2_DIV8                  (0x00003000U)                     /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE2_DIV16                 (0x00003800U)                     /*!< HCLK divided by 16 */

/* SWS configuration */
#define RCC_CFGR_SWS_Pos                     (2U)
#define RCC_CFGR_SWS_Msk                     (0x3UL << RCC_CFGR_SWS_Pos)        /*!< 0x0000000C */
#define RCC_CFGR_SWS                         RCC_CFGR_SWS_Msk                  /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_0                       (0x1UL << RCC_CFGR_SWS_Pos)        /*!< 0x00000004 */
#define RCC_CFGR_SWS_1                       (0x2UL << RCC_CFGR_SWS_Pos)        /*!< 0x00000008 */

#define RCC_CFGR_SWS_MSI                     (0x00000000U)                     /*!< MSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSI                     (0x00000004U)                     /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSE                     (0x00000008U)                     /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL                     (0x0000000CU)                     /*!< PLL used as system clock */

/* PLL entry clock source*/
#define RCC_CFGR_PLLSRC_Pos                  (16U)
#define RCC_CFGR_PLLSRC_Msk                  (0x1UL << RCC_CFGR_PLLSRC_Pos)     /*!< 0x00010000 */
#define RCC_CFGR_PLLSRC                      RCC_CFGR_PLLSRC_Msk               /*!< PLL entry clock source */

#define RCC_CFGR_PLLSRC_HSI                  (0x00000000U)                     /*!< HSI as PLL entry clock source */
#define RCC_CFGR_PLLSRC_HSE                  (0x00010000U)                     /*!< HSE as PLL entry clock source */


/* PLLMUL configuration */
#define RCC_CFGR_PLLMUL_Pos                  (18U)
#define RCC_CFGR_PLLMUL_Msk                  (0xFUL << RCC_CFGR_PLLMUL_Pos)     /*!< 0x003C0000 */
#define RCC_CFGR_PLLMUL                      RCC_CFGR_PLLMUL_Msk               /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define RCC_CFGR_PLLMUL_0                    (0x1UL << RCC_CFGR_PLLMUL_Pos)     /*!< 0x00040000 */
#define RCC_CFGR_PLLMUL_1                    (0x2UL << RCC_CFGR_PLLMUL_Pos)     /*!< 0x00080000 */
#define RCC_CFGR_PLLMUL_2                    (0x4UL << RCC_CFGR_PLLMUL_Pos)     /*!< 0x00100000 */
#define RCC_CFGR_PLLMUL_3                    (0x8UL << RCC_CFGR_PLLMUL_Pos)     /*!< 0x00200000 */

#define RCC_CFGR_PLLMUL3                     (0x00000000U)                     /*!< PLL input clock * 3 */
#define RCC_CFGR_PLLMUL4                     (0x00040000U)                     /*!< PLL input clock * 4 */
#define RCC_CFGR_PLLMUL6                     (0x00080000U)                     /*!< PLL input clock * 6 */
#define RCC_CFGR_PLLMUL8                     (0x000C0000U)                     /*!< PLL input clock * 8 */
#define RCC_CFGR_PLLMUL12                    (0x00100000U)                     /*!< PLL input clock * 12 */
#define RCC_CFGR_PLLMUL16                    (0x00140000U)                     /*!< PLL input clock * 16 */
#define RCC_CFGR_PLLMUL24                    (0x00180000U)                     /*!< PLL input clock * 24 */
#define RCC_CFGR_PLLMUL32                    (0x001C0000U)                     /*!< PLL input clock * 32 */
#define RCC_CFGR_PLLMUL48                    (0x00200000U)                     /*!< PLL input clock * 48 */

/* PLLDIV configuration */
#define RCC_CFGR_PLLDIV_Pos                  (22U)
#define RCC_CFGR_PLLDIV_Msk                  (0x3UL << RCC_CFGR_PLLDIV_Pos)     /*!< 0x00C00000 */
#define RCC_CFGR_PLLDIV                      RCC_CFGR_PLLDIV_Msk               /*!< PLLDIV[1:0] bits (PLL Output Division) */
#define RCC_CFGR_PLLDIV_0                    (0x1UL << RCC_CFGR_PLLDIV_Pos)     /*!< 0x00400000 */
#define RCC_CFGR_PLLDIV_1                    (0x2UL << RCC_CFGR_PLLDIV_Pos)     /*!< 0x00800000 */

#define RCC_CFGR_PLLDIV2_Pos                 (22U)
#define RCC_CFGR_PLLDIV2_Msk                 (0x1UL << RCC_CFGR_PLLDIV2_Pos)    /*!< 0x00400000 */
#define RCC_CFGR_PLLDIV2                     RCC_CFGR_PLLDIV2_Msk              /*!< PLL clock output = CKVCO / 2 */
#define RCC_CFGR_PLLDIV3_Pos                 (23U)
#define RCC_CFGR_PLLDIV3_Msk                 (0x1UL << RCC_CFGR_PLLDIV3_Pos)    /*!< 0x00800000 */
#define RCC_CFGR_PLLDIV3                     RCC_CFGR_PLLDIV3_Msk              /*!< PLL clock output = CKVCO / 3 */
#define RCC_CFGR_PLLDIV4_Pos                 (22U)
#define RCC_CFGR_PLLDIV4_Msk                 (0x3UL << RCC_CFGR_PLLDIV4_Pos)    /*!< 0x00C00000 */
#define RCC_CFGR_PLLDIV4                     RCC_CFGR_PLLDIV4_Msk              /*!< PLL clock output = CKVCO / 4 */

/********************  Bit definition for RCC registers  ********************/

#define RCC_CR_HSIDIVF_Pos               (4U)
#define RCC_CR_HSIDIVF_Msk               (0x1UL << RCC_CR_HSIDIVF_Pos)          /*!< 0x00000010 */
#define RCC_CR_HSIDIVF                   RCC_CR_HSIDIVF_Msk                    /*!< Internal High Speed clock divider flag */
#define RCC_ICSCR_MSIRANGE_Pos           (13U)
#define RCC_ICSCR_MSIRANGE_Msk           (0x7UL << RCC_ICSCR_MSIRANGE_Pos)      /*!< 0x0000E000 */
#define RCC_ICSCR_MSIRANGE               RCC_ICSCR_MSIRANGE_Msk                /*!< Internal Multi Speed clock Range */
#include "GPIO.h"
#include <string.h>
#include "EXTI.h"
#include "SPI.h"
#include "UART.h"
#include "RCC.h"
#include "I2C.h"

#endif /* STM32L0XX_H_ */
