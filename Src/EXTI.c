#include "EXTI.h"

/**
  * @brief  EXTI init for valid GPIO port and line number.
  * @param  EXTI_InitStruct =user config structure.

  * @retval none.
  */

void EXTI_Init(EXTI_InitTypedef_t *EXTI_InitStruct){
	uint32_t temp;
	temp=(uint32_t)EXTI_BASE_ADDR;
	EXTI->IMR &= ~(0x1 << EXTI_InitStruct->EXTI_LineNum);
	EXTI->EMR &= ~(0x1 << EXTI_InitStruct->EXTI_LineNum);
	if(EXTI_InitStruct->EXTI_LineCmd ==ENABLE){
		temp += EXTI_InitStruct->EXTI_Mode;
		*((__IO uint32_t *)temp) |= (0x1 << EXTI_InitStruct->EXTI_LineNum);
		temp=(uint32_t)EXTI_BASE_ADDR;
		EXTI->RTSR &= ~(0x1 << EXTI_InitStruct->EXTI_LineNum);
		EXTI->FTSR &= ~(0x1 << EXTI_InitStruct->EXTI_LineNum);
		if(EXTI_InitStruct->Trigger_Selection == EXTI_RF_Trigger){
			EXTI->RTSR |= (0x1 << EXTI_InitStruct->EXTI_LineNum);
			EXTI->FTSR |= (0x1 << EXTI_InitStruct->EXTI_LineNum);
		}
		else{
			temp += EXTI_InitStruct->Trigger_Selection;
			*((__IO uint32_t*)temp) |= (0x1 << EXTI_InitStruct->EXTI_LineNum);

		}
	}
	else{

		temp=(uint32_t)EXTI_BASE_ADDR;
		temp += EXTI_InitStruct->EXTI_Mode;
		*((__IO uint32_t*)temp) &= ~(0x1 << EXTI_InitStruct->EXTI_LineNum);
	}

}



/**
  * @brief  Set configuration of a dedicated Exti line.
  * @param  portSource =port value A to H @def_group PORT_values
  * @param  EXTI_LineSource =Pin numbers & Line numbers 0 to 15 @def_group EXTI_LineValues
  * @retval none.
  */
void EXTI_LineConfig(uint8_t portSource,uint8_t EXTI_LineSource)
{
	uint32_t tempValue=SYSCFG->SYSCFG_EXTICR[EXTI_LineSource >> 2U];
	tempValue &= ~(0xFU << (EXTI_LineSource & 0x3)*4);
	tempValue=(portSource << (EXTI_LineSource & 0x3)*4);
	SYSCFG->SYSCFG_EXTICR[EXTI_LineSource >> 2U]=tempValue;
}
/**
  * @brief  NVIC enable interrupt
  * @param  IRQnumber = number of line.
  * @retval none.
  */
void NVIC_Enable(IRQ_Number_Typedef_t IRQnumber){

	uint32_t temp=0;
	temp= *NVIC_ISER;
	temp &= ~(0x1 << IRQnumber);
	temp |= (0x1 << IRQnumber);
	*NVIC_ISER=temp;
}

