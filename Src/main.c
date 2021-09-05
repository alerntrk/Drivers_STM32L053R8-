#include "stm32l053.h"

UART_Handle_Typedef_t huart;
static void UART_Config(void)
{
	RCC_USART2_CLK_ENABLE();
	huart.Instance=USART2;
	huart.Init.mode=UART_MODE_TxRx;
	huart.Init.HardwareFlowControl=UART_HW_NONE;
	huart.Init.oversampling=UART_OVERSAMP_16;
	huart.Init.WordLength=UART_WORDLEN_8B;
	huart.Init.parity=UART_PARITY_NONE;
	huart.Init.StopBits=UART_STOP_BITS_1;
	huart.Init.baudRate=115200;
	UART_Init(&huart);
	UART_Periph_Cmd(&huart, ENABLE);
	NVIC_Enable(USART2_IRQn);

}
void USART2_IRQHandler(){

	UART_Interrupt_Handler(&huart);


}
/*   **** SPI AND INTERRUPT TEST  ****
SPI_Handle_Typedef_t SPI_Handle;
static void SPI_Config(){

	RCC_SPI1_CLK_ENABLE();

	SPI_Handle.Instance=SPI1;
	SPI_Handle.Init.BaudRate = SPI_BAUDRATE_DIV8;
	SPI_Handle.Init.BusConfig=SPI_FullDuplex;
	SPI_Handle.Init.CPHA = SPI_CPHA_FIRST;
	SPI_Handle.Init.CPOL = SPI_CPOL_LOW;
	SPI_Handle.Init.DFF = SPI_DFF_8BIT;
	SPI_Handle.Init.FrameFormat = SPI_FF_MSB;
	SPI_Handle.Init.Mode = SPI_MODE_MASTER;
	SPI_Handle.Init.SSM_Cmd = SPI_SSM_ENABLE;
	SPI_Init(&SPI_Handle);
	NVIC_Enable(SPI1_IRQ);
	SPI_PeriphCmd(&SPI_Handle, ENABLE);

}
static void SPI_GPIO_Config(){

	GPIO_Init_Typedef_t GPIO_InitStruct={0};
	GPIO_InitStruct.PinNum = GPIO_PIN_5 | GPIO_PIN_7;
	GPIO_InitStruct.Mode=GPIO_MODE_AF;
	GPIO_InitStruct.OTYPE=GPIO_OTYPE_PP;
	GPIO_InitStruct.PuPd=GPIO_PUPD_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate=GPIO_AF0;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void SPI1_IRQHandler(){
	SPI_Interrupt_Handler(&SPI_Handle);
}
static void Interrupt()
{
	RCC_SYSCFG_CLK_ENABLE();
	EXTI_InitTypedef_t EXTI_InitStruct={0};
	EXTI_LineConfig(EXTI_PortSource_GPIOA, EXTI_LineSource_4);
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;
	EXTI_InitStruct.EXTI_LineNum=EXTI_LineSource_4;
	EXTI_InitStruct.EXTI_Mode=EXTI_MODE_Interrupt;
	EXTI_InitStruct.Trigger_Selection=EXTI_Rising_Trigger;
	EXTI_Init(&EXTI_InitStruct);
	NVIC_Enable(EXTI_4_15_IRQ);
}
void EXTI4_15_IRQHandler(){
	char msg[]="heloWorld\n";
	if(EXTI->PR & (0x1<<4)){
		EXTI->PR |= (0x1 << 4U);
		SPI_TransmitData_IT(&SPI_Handle, (uint8_t*)msg, strlen(msg));
	}

}*/
static void GPIO_LedConfig(){
  GPIO_Init_Typedef_t ledStruct={0};
  RCC_GPIOA_CLCK_ENABLE();
 // RCC_GPIOC_CLCK_ENABLE();
  ledStruct.PinNum=GPIO_PIN_2 | GPIO_PIN_3;
  ledStruct.Mode=GPIO_MODE_AF;
  ledStruct.PuPd=GPIO_PUPD_NOPULL;
  ledStruct.OTYPE=GPIO_OTYPE_PP;
  ledStruct.Speed=GPIO_SPEED_HIGH;
  ledStruct.Alternate=GPIO_AF4;
  GPIO_Init(GPIOA,&ledStruct);

}


int main(void)
{
	//char msgToSent[]="HEY Bro How You Doing?\n";
	char buff[30]="\0";
	GPIO_LedConfig();
	UART_Config();
	/*Interrupt();
	SPI_GPIO_Config();
	SPI_Config();*/
	//UART_ReceiveData(&huart, (uint8_t*)msgToSent, 4);
//	UART_TransmitData_IT(&huart,(uint8_t*)msgToSent, strlen(msgToSent));
	UART_ReceiveData_IT(&huart, (uint8_t*)buff,4);

	for(;;){
		UART_TransmitData(&huart,(uint8_t*)buff, strlen(buff));


	}

}








