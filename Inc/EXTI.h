/*
 * EXTI.h
 *
 *  Created on: 27 Tem 2021
 *      Author: win10
 */

#ifndef INC_EXTI_H_
#define INC_EXTI_H_
#include "stm32l053.h"
/*
 * @def_group PORT_values
 */
#define EXTI_PortSource_GPIOA						((uint8_t)(0x00))
#define EXTI_PortSource_GPIOB						((uint8_t)(0x01))
#define EXTI_PortSource_GPIOC						((uint8_t)(0x02))
#define EXTI_PortSource_GPIOD						((uint8_t)(0x03))
#define EXTI_PortSource_GPIOE						((uint8_t)(0x04))
#define EXTI_PortSource_GPIOH						((uint8_t)(0x05))
/*
 * @def_group EXTI_LineValues
 */
#define EXTI_LineSource_0							((uint8_t)(0x00))
#define EXTI_LineSource_1							((uint8_t)(0x01))
#define EXTI_LineSource_2							((uint8_t)(0x02))
#define EXTI_LineSource_3							((uint8_t)(0x03))
#define EXTI_LineSource_4							((uint8_t)(0x04))
#define EXTI_LineSource_5							((uint8_t)(0x05))
#define EXTI_LineSource_6							((uint8_t)(0x06))
#define EXTI_LineSource_7							((uint8_t)(0x07))
#define EXTI_LineSource_8							((uint8_t)(0x08))
#define EXTI_LineSource_9							((uint8_t)(0x09))
#define EXTI_LineSource_10							((uint8_t)(0x0A))
#define EXTI_LineSource_11							((uint8_t)(0x0B))
#define EXTI_LineSource_12							((uint8_t)(0x0C))
#define EXTI_LineSource_13							((uint8_t)(0x0D))
#define EXTI_LineSource_14							((uint8_t)(0x0E))
#define EXTI_LineSource_15							((uint8_t)(0x0F))


/*
 * @def_group EXTI_Modes
 */
#define EXTI_MODE_Interrupt							((uint8_t)(0x00))
#define EXTI_MODE_Event								((uint8_t)(0x04))

/*
 * @def_group EXTI_Trigger_Values
 */
#define EXTI_Rising_Trigger							((uint8_t)(0x08))
#define	EXTI_Falling_Trigger						((uint8_t)(0x0C))
#define EXTI_RF_Trigger								((uint8_t)(0x10))

void EXTI_LineConfig(uint8_t portSource,uint8_t LineSource);


typedef struct{

		FunctionalState_t	EXTI_LineCmd;			/* Mask or unmask the line number								  */
		uint8_t EXTI_LineNum;						/* EXTI line number for valid GPIO pin @def_group EXTI_LineValues */
		uint8_t Trigger_Selection;					/* EXTI Trigger selection values  @def_group EXTI_Trigger_Values */
		uint8_t	EXTI_Mode;							/* EXTI modes @def_group EXTI_Modes 							  */

}EXTI_InitTypedef_t;

void EXTI_Init(EXTI_InitTypedef_t *EXTI_InitStruct);
void NVIC_Enable(IRQ_Number_Typedef_t);

#endif /* INC_EXTI_H_ */
