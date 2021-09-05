/*
 * GPIO.c
 *
 *  Created on: 18 Tem 2021
 *      Author: win10
 */


#include "GPIO.h"
/**
  * @brief  Initializes the GPIOx peripheral according to the specified parameters in the GPIO_Init.
  * @param  GPIOx where x can be (A..E and H) to select the GPIO peripheral for STM32L0XX family devices.
  *                Note that GPIOE is not available on all devices.
  * @param  GPIO_Init pointer to a GPIO_InitTypeDef structure that contains
  *                    the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void GPIO_Init(GPIO_TypeDef_t *GPIOx,GPIO_Init_Typedef_t* GPIO_Config){
	uint32_t fakePosition=0;
	uint32_t position;
	uint32_t lastPosition=0;

	for(position=0;position<16;position++){

		fakePosition=(0x1 << position);
		lastPosition=(uint32_t)(GPIO_Config->PinNum) & fakePosition;
		if(fakePosition==lastPosition){
			/* Mode Config*/

			uint32_t tempValue;
			tempValue=GPIOx->MODER;
			tempValue &= ~(0x3 << (position*2));
			tempValue |= (GPIO_Config->Mode <<  (position*2));
			GPIOx->MODER=tempValue;


		if(GPIO_Config->Mode==GPIO_MODE_OUTPUT || GPIO_Config->Mode==GPIO_MODE_AF){
			/* Output Type Config*/

			tempValue=GPIOx->OTYPER;
			tempValue &= ~(0x1U << position);
			tempValue |= (GPIO_Config->OTYPE << position);
			GPIOx->OTYPER=tempValue;
			/* Output Speed Config*/

			tempValue=GPIOx->OSPEEDR;
			tempValue &= ~(0x3U << (position*2));
			tempValue |= (GPIO_Config->Speed << (position*2));
			GPIOx->OSPEEDR=tempValue;
		}
		    /* Push Pull Config*/

			tempValue=GPIOx->PUPDR;
			tempValue &= ~(0x3U << (position*2));
			tempValue |= (GPIO_Config->PuPd << (position*2));
			GPIOx->PUPDR=tempValue;

			if(GPIO_Config->Mode==GPIO_MODE_AF){
				tempValue=GPIOx->AFR[position >> 3U];
				tempValue &= ~(0xFU << ((position & 0x7U)*4));
				tempValue |= (GPIO_Config->Alternate << ((position & 0x7U)*4));
				GPIOx->AFR[position >> 3U]=tempValue;
			}
		}
	}

}

/**
  * @brief  Sets or clears the selected data port bit.
  *
  * @note   This function uses GPIOx_BSRR register to allow atomic read/modify
  *         accesses. In this way, there is no risk of an IRQ occurring between
  *         the read and the modify access.
  *
  * @param  GPIOx where x can be (A..E and H) to select the GPIO peripheral for STM32L0xx family devices.
  *                Note that GPIOE is not available on all devices.
  * @param  pinNumber specifies the port bit to be written.
  *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
  *                   All port bits are not necessarily available on all GPIOs.
  * @param  pinState specifies the value to be written to the selected bit.
  *                   This parameter can be one of the GPIO_PinState enum values:
  *                        GPIO_PIN_RESET: to clear the port pin
  *                        GPIO_PIN_SET: to set the port pin
  * @retval None
  */
void GPIO_Write_Pin(GPIO_TypeDef_t *GPIOx,uint16_t pinNumber,GPIO_PIN_STATE pinState){

	if(pinState==GPIO_PIN_SET) GPIOx->BSRR=pinNumber;
	else GPIOx->BSRR=(pinNumber<<16U);

}


/**
  * @brief  Reads the specified input port pin.
  * @param  GPIOx where x can be (A..E and H) to select the GPIO peripheral for STM32L0xx family devices.
  *                Note that GPIOE is not available on all devices.
  * @param  pinNumber specifies the port bit to read.
  *                   This parameter can be GPIO_PIN_x where x can be (0..15).
  *                   All port bits are not necessarily available on all GPIOs.
  * @retval The input port pin value.
  */

GPIO_PIN_STATE GPIO_Read_Pin(GPIO_TypeDef_t *GPIOx,uint16_t pinNumber)
{

	GPIO_PIN_STATE Bitstatus=GPIO_PIN_RESET;
	if((GPIOx->IDR & pinNumber) != GPIO_PIN_RESET)	Bitstatus=GPIO_PIN_SET;
	return Bitstatus;
}

/**
* @brief  Locks GPIO Pins configuration registers.
* @note   The locked registers are GPIOx_MODER, GPIOx_OTYPER, GPIOx_OSPEEDR,
*         GPIOx_PUPDR, GPIOx_AFRL and GPIOx_AFRH.
* @note   The configuration of the locked GPIO pins can no longer be modified
*         until the next reset.
* @param  GPIOx where x can be (A..E and H) to select the GPIO peripheral for STM32L0xx family.
*                Note that GPIOE is not available on all devices.
* @param  pinNumber specifies the port bit to be locked.
*         This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
*         All port bits are not necessarily available on all GPIOs.
* @retval None
*/
void GPIO_Lock_Pin(GPIO_TypeDef_t *GPIOx,uint16_t pinNumber)
{
	uint32_t temp=(0x1U << 16U) | (pinNumber);
	GPIOx->LCKR= temp;		//WR LCKR[16] = ‘1’ + LCKR[15:0]
	GPIOx->LCKR=pinNumber;	//WR LCKR[16] = ‘0’ + LCKR[15:0]
	GPIOx->LCKR=temp;		//WR LCKR[16] = ‘1’ + LCKR[15:0]
	temp=GPIOx->LCKR;		//RD LCKR
}
/**
* @brief Toggles GPIO Pins configuration registers.
*
* @param  GPIOx where x can be (A..E and H) to select the GPIO peripheral for STM32L0xx family.
*                Note that GPIOE is not available on all devices.
* @param  pinNumber specifies the port bit to be locked.
*         This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
*         All port bits are not necessarily available on all GPIOs.
* @retval None
*/
void GPIO_Toggle_Pin(GPIO_TypeDef_t *GPIOx,uint16_t pinNumber){
	uint32_t temp=GPIOx->ODR;
	GPIOx->BSRR=((temp & pinNumber) << 16U) | (~temp & pinNumber);
}




