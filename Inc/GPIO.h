/*
 * GPIO.h
 *
 *  Created on: 18 Tem 2021
 *      Author: win10
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_
#include "stm32l053.h"
/*
 * @def_group GPIO_PINS
 */
#define GPIO_PIN_0				(uint16_t)(0x0001)   /* Gpio pin 0 selected*/
#define GPIO_PIN_1				(uint16_t)(0x0002)	 /* Gpio pin 1 selected*/
#define GPIO_PIN_2				(uint16_t)(0x0004)	 /* Gpio pin 2 selected*/
#define GPIO_PIN_3				(uint16_t)(0x0008)   /* Gpio pin 3 selected*/
#define GPIO_PIN_4				(uint16_t)(0x0010)   /* Gpio pin 4 selected*/
#define GPIO_PIN_5				(uint16_t)(0x0020)   /* Gpio pin 5 selected*/
#define GPIO_PIN_6				(uint16_t)(0x0040)   /* Gpio pin 6 selected*/
#define GPIO_PIN_7				(uint16_t)(0x0080)   /* Gpio pin 7 selected*/
#define GPIO_PIN_8				(uint16_t)(0x0100)   /* Gpio pin 8 selected*/
#define GPIO_PIN_9				(uint16_t)(0x0200)   /* Gpio pin 9 selected*/
#define GPIO_PIN_10				(uint16_t)(0x0400)   /* Gpio pin 10 selected*/
#define GPIO_PIN_11				(uint16_t)(0x0800)   /* Gpio pin 11 selected*/
#define GPIO_PIN_12				(uint16_t)(0x1000)   /* Gpio pin 12 selected*/
#define GPIO_PIN_13				(uint16_t)(0x2000)   /* Gpio pin 13 selected*/
#define GPIO_PIN_14				(uint16_t)(0x4000)   /* Gpio pin 14 selected*/
#define GPIO_PIN_15				(uint16_t)(0x8000)   /* Gpio pin 15 selected*/
#define GPIO_PIN_ALL			(uint16_t)(0xFFFF)   /* All Gpio pin selected*/
/*
 * @def_group GPIO_PIN_MODES
 */
#define GPIO_MODE_INPUT			(0x0U)
#define GPIO_MODE_OUTPUT		(0x1U)
#define GPIO_MODE_AF			(0x2U)
#define GPIO_MODE_ANALOG		(0x3U)
/*
 * @def_group GPIO_OTYPE_MODES
 */
#define GPIO_OTYPE_PP		(0x0U)
#define GPIO_OTYPE_OD		(0x1U)

/*
 * @def_group GPIO_PUPD_MODES
 */
#define GPIO_PUPD_NOPULL		(0x0U)
#define GPIO_PUPD_PULLUP		(0x1U)
#define GPIO_PUPD_PULLDOWN		(0x2U)
/*
 * @def_group GPIO_SPEED_MODES
 */

#define GPIO_SPEED_LOW			(0x0U)
#define GPIO_SPEED_MEDIUM		(0x1U)
#define GPIO_SPEED_HIGH			(0x2U)
#define GPIO_VERY_HIGH			(0x3U)

/*
 * @def_group GPIO_AF_MODES
 */
#define GPIO_AF0				(0x00U)
#define GPIO_AF1				(0x01U)
#define GPIO_AF2				(0x02U)
#define GPIO_AF3				(0x03U)
#define GPIO_AF4				(0x04U)
#define GPIO_AF5				(0x05U)
#define GPIO_AF6				(0x06U)
#define GPIO_AF7				(0x07U)



typedef enum{
	GPIO_PIN_RESET=0x0,
	GPIO_PIN_SET=!GPIO_PIN_RESET
}GPIO_PIN_STATE;



typedef struct{
	uint32_t Mode;
	uint32_t PinNum; 		 /* GPIO pin numbers @def_group GPIO_PINS */
	uint32_t OTYPE; 		 /* GPIO pin numbers @def_group GPIO_PIN_MODES */
	uint32_t PuPd;   		 /* GPIO pin numbers @def_group GPIO_OTYPE_MODES */
	uint32_t Speed;	 		 /* GPIO pin numbers @def_group GPIO_SPEED_MODES */
	uint32_t Alternate;	 	 /* GPIO pin numbers @def_group GPIO_AF_MODES */

}GPIO_Init_Typedef_t;

void GPIO_Write_Pin(GPIO_TypeDef_t *GPIOx,uint16_t pinNumber,GPIO_PIN_STATE pinState);
GPIO_PIN_STATE GPIO_Read_Pin(GPIO_TypeDef_t *GPIOx,uint16_t pinNumber);
void GPIO_Lock_Pin(GPIO_TypeDef_t *GPIOx,uint16_t pinNumber);
void GPIO_Init(GPIO_TypeDef_t *,GPIO_Init_Typedef_t*);
void GPIO_Toggle_Pin(GPIO_TypeDef_t *GPIOx,uint16_t pinNumber);
#endif /* INC_GPIO_H_ */
