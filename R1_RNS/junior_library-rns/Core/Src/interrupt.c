
/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "interrupt.h"
#include "include.h"
//#include "common.h"
/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
//#define USED_QEI1
//#define USED_QEI4
//#define USED_QEI6

int count = 0;
int count2 = 0;
int count3 = 0;
int _counter = 0;
/**
 * * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{

}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{

}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{

	while(1){

	}
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{

}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{

}

/**
 * @brief This function handles System service call via SWI instruction.
 */
//void SVC_Handler(void)
//{
//
//}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{

}

/**
 * @brief This function handles Pendable request for system service.
 */
//void PendSV_Handler(void)
//{
//
//}
void TIM8_UP_TIM13_IRQHandler(void){
	//A

	if (TIM8 -> CR1 == 129)
		BIOS_QEI6.signbit += 1;
	else if (TIM8 ->CR1 == 145)
		BIOS_QEI6.signbit -= 1;

	TIM8 -> SR = 0;
//	testCounter = testCounter + 1;
	QEIDelay(100);
}
void TIM4_IRQHandler(void){
	//B
	if (TIM4 -> CR1 == 129)
		BIOS_QEI4.signbit += 1;
	else if (TIM4 ->CR1 == 145)
		BIOS_QEI4.signbit -= 1;

	TIM4 -> SR = 0;
//	testCounter = testCounter + 1;
	QEIDelay(100);
}
void TIM1_UP_TIM10_IRQHandler(void){
	//C
	if (TIM1 -> CR1 == 129)
		BIOS_QEI1.signbit += 1;
	else if (TIM1 ->CR1 == 145)
		BIOS_QEI1.signbit -= 1;

	TIM1 -> SR = 0;
//	testCounter = testCounter + 1;
	QEIDelay(100);
}
void TIM3_IRQHandler(void){
	//D
	if (TIM3 -> CR1 == 129)
		BIOS_QEI3.signbit += 1;
	else if (TIM3 ->CR1 == 145)
		BIOS_QEI3.signbit -= 1;

	TIM3 -> SR = 0;
//	testCounter = testCounter + 1;
	QEIDelay(100);
}
void TIM2_IRQHandler(void){
	//X
	if (TIM2 -> CR1 == 129)
		BIOS_QEI2.signbit += 1;
	else if (TIM2 ->CR1 == 145)
		BIOS_QEI2.signbit -= 1;
	TIM2 -> SR = 0;
//	testCounter = testCounter + 1;
	QEIDelay(100);
}
void TIM5_IRQHandler(void){
	//Y

	if (TIM5 -> CR1 == 129)
		BIOS_QEI5.signbit += 1;
	else if (TIM5 ->CR1 == 145)
		BIOS_QEI5.signbit -= 1;

	TIM5 -> SR = 0;
//	testCounter = testCounter + 1;
	QEIDelay(100);
}

void TIM7_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim7);
}

char budtest[100];
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int budcount = 0;
	if (htim->Instance == TIM7) {
		HAL_IncTick();
		//		MUXUpdate(&MUX);
		SHIFTREGShift(&SR);
		if(budcount++ >= 20){
			sprintf(budtest, "Where am i? I am in your head :)\n");
			UARTPrintString(&huart5, budtest);
			budcount = 0;
		}
	}
}

void USART2_IRQHandler(void)
{
//	rcvdata = (uint16_t)(huart2.Instance->DR & (uint16_t)0x01FF);
//	if(rcvdata == '\n'){
//		for(int i = u2rx_count; i < 25; i++)
//			c[i] = '\0';
//		u2rx_count = 0;
//		sscanf(c, "%u%d%u", &header, &yaw, &lastbyte);
//		checksum = header + yaw + lastbyte;
//		if(!(checksum & 0xFF)){
//			fyaw = yaw/100.0+180.0;
//			LED3=!LED3;
//		}
//	}else if(u2rx_count > 25){
//		u2rx_count = 0;
//	}else{
//		c[u2rx_count++] = rcvdata;
//	}

//	sprintf(uartbuf," %f \r\n",fyaw);
	HAL_UART_IRQHandler(&huart2);
//	UARTPrintString_IT(&huart5,uartbuf);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == IMU.huartx){
		R6091U_Handler(&IMU);
	}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
