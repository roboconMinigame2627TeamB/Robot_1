
/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "uart.h"

/*
 * Function Name		: UART1Init
 * Function Description : This function is called to initialize USART1 only.
 * Function Remarks		:
 * Function Arguments	: huartx                ,Pointer to uart handle type
 * 						  baudrate				,normally set to 115200 according to UTM ROBOCON UART COMMUNICATION PROTOCOL.
 * 						  rxstate				,can be ENABLE (enable USART1 receive interrupt) or DISBALE
 * 						  preemptionpriority    ,interrupt with higher preemption priority executed first
 * 						  subpriority			,when 2 interrupts have similar preemption priorities, interrupt with higher
 * 						  						 priority will be executed first. If 2 interrupts have similar preemption
 * 						  						 subpriority, then the one comes first in the program will be executed first.
 * Function Return		: None
 * Function Example		: UARTxInit(&huart1, 115200, ENABLE, 0, 0);
 */
void UARTInit(UART_HandleTypeDef* huartx, uint32_t baudrate, FunctionalState rxstate, uint16_t preemptionpriority,uint16_t subpriority)
{
	IRQn_Type nvic;
	uint8_t *rcv_data;

	if (huartx == &huart1){
		huartx->Instance = USART1;
		nvic = USART1_IRQn;
		rcv_data = &uart1_data;
	}else if(huartx == &huart2){
		huartx->Instance = USART2;
		nvic = USART2_IRQn;
		rcv_data = &uart2_data;
	}else if(huartx == &huart3){
		huartx->Instance = USART3;
		nvic = USART3_IRQn;
		rcv_data = &uart3_data;
	}else if(huartx == &huart4){
		huartx->Instance = UART4;
		nvic = UART4_IRQn;
		rcv_data = &uart4_data;
	}else if(huartx == &huart5){
		huartx->Instance = UART5;
		nvic = UART5_IRQn;
		rcv_data = &uart5_data;
	}else{
		huartx->Instance = USART6;
		nvic = USART6_IRQn;
		rcv_data = &uart6_data;
	}

	huartx->Init.BaudRate = baudrate;
	huartx->Init.WordLength = UART_WORDLENGTH_8B;
	huartx->Init.StopBits = UART_STOPBITS_1;
	huartx->Init.Parity = UART_PARITY_NONE;
	huartx->Init.Mode = UART_MODE_TX_RX;
	huartx->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huartx->Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(huartx) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_NVIC_SetPriority(nvic, preemptionpriority, subpriority);
	HAL_NVIC_EnableIRQ(nvic);

	if(rxstate == ENABLE){
		__HAL_UART_ENABLE_IT(huartx, UART_IT_RXNE);
	}
}

void UARTx_DMA_Rx_Init(UART_HandleTypeDef* huartx, DMA_HandleTypeDef* hdma_usart_rx, uint16_t preemptionpriority, uint16_t subpriority)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	IRQn_Type nvic;
	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	if(hdma_usart_rx == &hdma_usart2_rx){
		nvic = DMA1_Stream5_IRQn;
		hdma_usart_rx->Instance = DMA1_Stream5;
	}else if(hdma_usart_rx == &hdma_usart3_rx){
		nvic = DMA1_Stream1_IRQn;
		hdma_usart_rx->Instance = DMA1_Stream1;
	}else if(hdma_usart_rx == &hdma_uart4_rx){
		nvic = DMA1_Stream2_IRQn;
		hdma_usart_rx->Instance = DMA1_Stream2;
	}else if(hdma_usart_rx == &hdma_uart5_rx){
		nvic = DMA1_Stream0_IRQn;
		hdma_usart_rx->Instance = DMA1_Stream0;
	}

	hdma_usart_rx->Init.Channel = DMA_CHANNEL_4;
	hdma_usart_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart_rx->Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart_rx->Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart_rx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart_rx->Init.Mode = DMA_NORMAL;
	hdma_usart_rx->Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart_rx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(hdma_usart_rx) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_LINKDMA(huartx,hdmarx, *hdma_usart_rx);
	HAL_NVIC_SetPriority(nvic, preemptionpriority, subpriority);
	HAL_NVIC_ClearPendingIRQ(nvic);
	HAL_NVIC_EnableIRQ(nvic);
}

void UARTx_DMA_Tx_Init(UART_HandleTypeDef* huartx, DMA_HandleTypeDef* hdma_usart_tx, uint16_t preemptionpriority, uint16_t subpriority)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	IRQn_Type nvic;
	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	if(hdma_usart_tx == &hdma_usart2_tx){
		nvic = DMA1_Stream6_IRQn;
		hdma_usart_tx->Instance = DMA1_Stream6;
	}else if(hdma_usart_tx == &hdma_usart3_tx){
		nvic = DMA1_Stream3_IRQn;
		hdma_usart_tx->Instance = DMA1_Stream3;
	}else if(hdma_usart_tx == &hdma_uart4_tx){
		nvic = DMA1_Stream4_IRQn;
		hdma_usart_tx->Instance = DMA1_Stream4;
	}else if(hdma_usart_tx == &hdma_uart5_tx){
		nvic = DMA1_Stream7_IRQn;
		hdma_usart_tx->Instance = DMA1_Stream7;
	}

	hdma_usart_tx->Init.Channel = DMA_CHANNEL_4;
	hdma_usart_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_usart_tx->Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart_tx->Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart_tx->Init.Mode = DMA_NORMAL;
	hdma_usart_tx->Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart_tx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(hdma_usart_tx) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_LINKDMA(huartx,hdmatx, *hdma_usart_tx);
	HAL_NVIC_SetPriority(nvic, preemptionpriority, subpriority);
	HAL_NVIC_ClearPendingIRQ(nvic);
	HAL_NVIC_EnableIRQ(nvic);
}


/*
 * Function Name		: USART_ReceiveData
 * Function Description : This function is called to receive a char from desired huartx, x can be 1 to 6.
 * Function Remarks		: None
 * Function Arguments	: huartx	,x can be 1 to 6.
 * 						  s			,buffer or string
 * Function Return		: None
 * Function Example		: USART_ReceiveData(huart4, buffer);
 */
uint16_t USART_ReceiveData(UART_HandleTypeDef* huartx)
{
	/* Receive Data */
	return (uint16_t)(huartx->Instance->DR & (uint16_t)0x01FF);
}



/*
 * Function Name		: UARTPrintString
 * Function Description : This function is called to print string to desired huartx, x can be 1 to 6.
 * Function Remarks		: None
 * Function Arguments	: huartx	,x can be 1 to 6.
 * 						  s			,buffer or string
 * Function Return		: None
 * Function Example		: UARTPrintString(huart4, buffer);
 */
void UARTPrintString(UART_HandleTypeDef* huartx, char s[])
{
	HAL_UART_Transmit(huartx, (uint8_t *)s, strlen(s), 10);
}

/*
 * Function Name		: UARTPrintString_IT
 * Function Description : This function is called to print string to desired huartx, x can be 1 to 6 in Interrupt mode.
 * Function Remarks		: None
 * Function Arguments	: huartx	,x can be 1 to 6.
 * 						  s			,buffer or string
 * Function Return		: None
 * Function Example		: UARTPrintString_IT(huart4, buffer);
 */
void UARTPrintString_IT(UART_HandleTypeDef* huartx, char s[]){
	if(HAL_UART_GetState(huartx) == HAL_UART_STATE_READY)
		HAL_UART_Transmit_IT(huartx, (uint8_t *)s, strlen(s));
}


//Expiremental

//void UART_DMA_TX_Init(UART_HandleTypeDef* huartx, DMA_HandleTypeDef* huart_dma, uint32_t baudrate, uint16_t preemptionpriority,uint16_t subpriority)
//{
//	IRQn_Type nvic,nvicdma;
//
//	if (huartx == &huart1){
//		huartx->Instance = USART1;
//		nvic = USART1_IRQn;
//		huart_dma->Instance = DMA2_Stream7;
//		huart_dma->Init.Channel = DMA_CHANNEL_4;
//		__HAL_RCC_DMA2_CLK_ENABLE();
//	}else if(huartx == &huart2){
//		huartx->Instance = USART2;
//		nvic = USART2_IRQn;
//		huart_dma->Instance = DMA1_Stream6;
//		huart_dma->Init.Channel = DMA_CHANNEL_4;
//		__HAL_RCC_DMA1_CLK_ENABLE();
//	}else if(huartx == &huart3){
//		huartx->Instance = USART3;
//		nvic = USART3_IRQn;
//		huart_dma->Instance = DMA1_Stream3;
//		huart_dma->Init.Channel = DMA_CHANNEL_4;
//	}else if(huartx == &huart4){
//		huartx->Instance = UART4;
//		nvic = UART4_IRQn;
//		huart_dma->Instance = DMA1_Stream4;
//		huart_dma->Init.Channel = DMA_CHANNEL_4;
//		__HAL_RCC_DMA1_CLK_ENABLE();
//	}else if(huartx == &huart5){
//		huartx->Instance = UART5;
//		nvic = UART5_IRQn;
//		huart_dma->Instance = DMA1_Stream7;
//		huart_dma->Init.Channel = DMA_CHANNEL_4;
//		__HAL_RCC_DMA1_CLK_ENABLE();
//	}else{
//		huartx->Instance = USART6;
//		nvic = USART6_IRQn;
//		huart_dma->Instance = DMA2_Stream6;
//		huart_dma->Init.Channel = DMA_CHANNEL_5;
//		__HAL_RCC_DMA2_CLK_ENABLE();
//	}
//
//	huartx->Init.BaudRate = baudrate;
//	huartx->Init.WordLength = UART_WORDLENGTH_8B;
//	huartx->Init.StopBits = UART_STOPBITS_1;
//	huartx->Init.Parity = UART_PARITY_NONE;
//	huartx->Init.Mode = UART_MODE_TX_RX;
//	huartx->Init.HwFlowCtl = UART_HWCONTROL_NONE;
//	huartx->Init.OverSampling = UART_OVERSAMPLING_16;
//
//
//	if (HAL_UART_Init(huartx) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	HAL_NVIC_SetPriority(nvicdma, 5, 0);
//	HAL_NVIC_EnableIRQ(nvicdma);
//
//
//	huart_dma->Init.Direction = DMA_MEMORY_TO_PERIPH;
//	huart_dma->Init.PeriphInc = DMA_PINC_DISABLE;
//	huart_dma->Init.MemInc = DMA_MINC_ENABLE;
//	huart_dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//	huart_dma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//	huart_dma->Init.Mode = DMA_CIRCULAR;
//	huart_dma->Init.Priority = DMA_PRIORITY_LOW;
//	huart_dma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//	if (HAL_DMA_Init(huart_dma) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	__HAL_LINKDMA(huartx,hdmatx,*huart_dma);
//
//	HAL_NVIC_SetPriority(nvic, preemptionpriority, subpriority);
//	HAL_NVIC_EnableIRQ(nvic);
//
//}

//void UARTPrintString_DMA_Start(UART_HandleTypeDef* huartx, char s[]){
//		HAL_UART_Transmit_DMA(huartx, (uint8_t *)s, strlen(s));
//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huartx){
//
//	if(huartx->Instance == USART1){
//		HAL_UART_Receive_IT(&huart1,&uart1_data,1);
//	}else if(huartx->Instance == USART2){
//		HAL_UART_Receive_IT(&huart2,&uart2_data,1);
//	}else if(huartx->Instance == USART3){
//		HAL_UART_Receive_IT(&huart3,&uart3_data,1);
//	}else if(huartx->Instance == UART4){
//		HAL_UART_Receive_IT(&huart4,&uart4_data,1);
//	}else if(huartx->Instance == UART5){
//		HAL_UART_Receive_IT(&huart5,&uart5_data,1);
//	}else{
//		HAL_UART_Receive_IT(&huart6,&uart6_data,1);
//	}
//
//}


// Handlers

void  USART1_IRQHandler(void){


	HAL_UART_IRQHandler(&huart1);
}
/*
void  USART2_IRQHandler(void){

	HAL_UART_IRQHandler(&huart2);
}
*/
void  USART3_IRQHandler(void){

	HAL_UART_IRQHandler(&huart3);
}

//void  UART4_IRQHandler(void){
//
//	HAL_UART_IRQHandler(&huart4);
//}

void UART5_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart5);
}

void  USART6_IRQHandler(void){

	HAL_UART_IRQHandler(&huart6);
}

/* Used by I2C1 RX*/
//void DMA1_Stream0_IRQHandler(void)
//{
//	HAL_DMA_IRQHandler(&hdma_uart5_rx);
//}

void DMA1_Stream1_IRQHandler(void)
{

	HAL_DMA_IRQHandler(&hdma_usart3_rx);


}

void DMA1_Stream2_IRQHandler(void)
{

	HAL_DMA_IRQHandler(&hdma_uart4_rx);
}

void DMA1_Stream5_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart2_rx);
}
void DMA1_Stream3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart3_tx);
}void DMA1_Stream4_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_uart4_tx);
}void DMA1_Stream6_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart2_tx);
}void DMA1_Stream7_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_uart5_tx);
}

//
///*********************************************/
///*          Include Header                   */
///*********************************************/
//#include "uart.h"
///*
// * Function Name		: UART1Init
// * Function Description : This function is called to initialize USART1 only.
// * Function Remarks		:
// * Function Arguments	: huartx                ,Pointer to uart handle type
// * 						  baudrate				,normally set to 115200 according to UTM ROBOCON UART COMMUNICATION PROTOCOL.
// * 						  rxstate				,can be ENABLE (enable USART1 receive interrupt) or DISBALE
// * 						  preemptionpriority    ,interrupt with higher preemption priority executed first
// * 						  subpriority			,when 2 interrupts have similar preemption priorities, interrupt with higher
// * 						  						 priority will be executed first. If 2 interrupts have similar preemption
// * 						  						 subpriority, then the one comes first in the program will be executed first.
// * Function Return		: None
// * Function Example		: UARTxInit(&huart1, 115200, ENABLE, 0, 0);
// */
//void UARTInit(UART_HandleTypeDef* huartx, uint32_t baudrate, FunctionalState rxstate, uint16_t preemptionpriority,uint16_t subpriority)
//{
//	IRQn_Type nvic;
//	uint8_t *rcv_data;
//
//	if (huartx == &huart1){
//		huartx->Instance = USART1;
//		nvic = USART1_IRQn;
//		rcv_data = &uart1_data;
//	}else if(huartx == &huart2){
//		huartx->Instance = USART2;
//		nvic = USART2_IRQn;
//		rcv_data = &uart2_data;
//	}else if(huartx == &huart3){
//		huartx->Instance = USART3;
//		nvic = USART3_IRQn;
//		rcv_data = &uart3_data;
//	}else if(huartx == &huart4){
//		huartx->Instance = UART4;
//		nvic = UART4_IRQn;
//		rcv_data = &uart4_data;
//	}else if(huartx == &huart5){
//		huartx->Instance = UART5;
//		nvic = UART5_IRQn;
//		rcv_data = &uart5_data;
//	}else{
//		huartx->Instance = USART6;
//		nvic = USART6_IRQn;
//		rcv_data = &uart6_data;
//	}
//
//
//	huartx->Init.BaudRate = baudrate;
//	huartx->Init.WordLength = UART_WORDLENGTH_8B;
//	huartx->Init.StopBits = UART_STOPBITS_1;
//	huartx->Init.Parity = UART_PARITY_NONE;
//	huartx->Init.Mode = UART_MODE_TX_RX;
//	huartx->Init.HwFlowCtl = UART_HWCONTROL_NONE;
//	huartx->Init.OverSampling = UART_OVERSAMPLING_16;
//
//
//	if (HAL_UART_Init(huartx) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	HAL_NVIC_SetPriority(nvic, preemptionpriority, subpriority);
//	HAL_NVIC_EnableIRQ(nvic);
//
//	if(rxstate == ENABLE){
//		__HAL_UART_ENABLE_IT(huartx, UART_IT_RXNE);
//	}
//}
//
//
//
//
///*
// * Function Name		: USART_ReceiveData
// * Function Description : This function is called to receive a char from desired huartx, x can be 1 to 6.
// * Function Remarks		: None
// * Function Arguments	: huartx	,x can be 1 to 6.
// * 						  s			,buffer or string
// * Function Return		: None
// * Function Example		: USART_ReceiveData(huart4, buffer);
// */
//uint16_t USART_ReceiveData(UART_HandleTypeDef* huartx)
//{
//	/* Receive Data */
//	return (uint16_t)(huartx->Instance->DR & (uint16_t)0x01FF);
//}
//
//
//
///*
// * Function Name		: UARTPrintString
// * Function Description : This function is called to print string to desired huartx, x can be 1 to 6.
// * Function Remarks		: None
// * Function Arguments	: huartx	,x can be 1 to 6.
// * 						  s			,buffer or string
// * Function Return		: None
// * Function Example		: UARTPrintString(huart4, buffer);
// */
//void UARTPrintString(UART_HandleTypeDef* huartx, char s[])
//{
//	HAL_UART_Transmit(huartx, (uint8_t *)s, strlen(s), 10);
//}
//
///*
// * Function Name		: UARTPrintString_IT
// * Function Description : This function is called to print string to desired huartx, x can be 1 to 6 in Interrupt mode.
// * Function Remarks		: None
// * Function Arguments	: huartx	,x can be 1 to 6.
// * 						  s			,buffer or string
// * Function Return		: None
// * Function Example		: UARTPrintString_IT(huart4, buffer);
// */
//void UARTPrintString_IT(UART_HandleTypeDef* huartx, char s[]){
//	if(HAL_UART_GetState(huartx) == HAL_UART_STATE_READY)
//		HAL_UART_Transmit_IT(huartx, (uint8_t *)s, strlen(s));
//}
//
//
////Expiremental
//
////void UART_DMA_TX_Init(UART_HandleTypeDef* huartx, DMA_HandleTypeDef* huart_dma, uint32_t baudrate, uint16_t preemptionpriority,uint16_t subpriority)
////{
////	IRQn_Type nvic,nvicdma;
////
////	if (huartx == &huart1){
////		huartx->Instance = USART1;
////		nvic = USART1_IRQn;
////		huart_dma->Instance = DMA2_Stream7;
////		huart_dma->Init.Channel = DMA_CHANNEL_4;
////		__HAL_RCC_DMA2_CLK_ENABLE();
////	}else if(huartx == &huart2){
////		huartx->Instance = USART2;
////		nvic = USART2_IRQn;
////		huart_dma->Instance = DMA1_Stream6;
////		huart_dma->Init.Channel = DMA_CHANNEL_4;
////		__HAL_RCC_DMA1_CLK_ENABLE();
////	}else if(huartx == &huart3){
////		huartx->Instance = USART3;
////		nvic = USART3_IRQn;
////		huart_dma->Instance = DMA1_Stream3;
////		huart_dma->Init.Channel = DMA_CHANNEL_4;
////	}else if(huartx == &huart4){
////		huartx->Instance = UART4;
////		nvic = UART4_IRQn;
////		huart_dma->Instance = DMA1_Stream4;
////		huart_dma->Init.Channel = DMA_CHANNEL_4;
////		__HAL_RCC_DMA1_CLK_ENABLE();
////	}else if(huartx == &huart5){
////		huartx->Instance = UART5;
////		nvic = UART5_IRQn;
////		huart_dma->Instance = DMA1_Stream7;
////		huart_dma->Init.Channel = DMA_CHANNEL_4;
////		__HAL_RCC_DMA1_CLK_ENABLE();
////	}else{
////		huartx->Instance = USART6;
////		nvic = USART6_IRQn;
////		huart_dma->Instance = DMA2_Stream6;
////		huart_dma->Init.Channel = DMA_CHANNEL_5;
////		__HAL_RCC_DMA2_CLK_ENABLE();
////	}
////
////	huartx->Init.BaudRate = baudrate;
////	huartx->Init.WordLength = UART_WORDLENGTH_8B;
////	huartx->Init.StopBits = UART_STOPBITS_1;
////	huartx->Init.Parity = UART_PARITY_NONE;
////	huartx->Init.Mode = UART_MODE_TX_RX;
////	huartx->Init.HwFlowCtl = UART_HWCONTROL_NONE;
////	huartx->Init.OverSampling = UART_OVERSAMPLING_16;
////
////
////	if (HAL_UART_Init(huartx) != HAL_OK)
////	{
////		Error_Handler();
////	}
////
////	HAL_NVIC_SetPriority(nvicdma, 5, 0);
////	HAL_NVIC_EnableIRQ(nvicdma);
////
////
////	huart_dma->Init.Direction = DMA_MEMORY_TO_PERIPH;
////	huart_dma->Init.PeriphInc = DMA_PINC_DISABLE;
////	huart_dma->Init.MemInc = DMA_MINC_ENABLE;
////	huart_dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
////	huart_dma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
////	huart_dma->Init.Mode = DMA_CIRCULAR;
////	huart_dma->Init.Priority = DMA_PRIORITY_LOW;
////	huart_dma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
////	if (HAL_DMA_Init(huart_dma) != HAL_OK)
////	{
////		Error_Handler();
////	}
////
////	__HAL_LINKDMA(huartx,hdmatx,*huart_dma);
////
////	HAL_NVIC_SetPriority(nvic, preemptionpriority, subpriority);
////	HAL_NVIC_EnableIRQ(nvic);
////
////}
//
////void UARTPrintString_DMA_Start(UART_HandleTypeDef* huartx, char s[]){
////		HAL_UART_Transmit_DMA(huartx, (uint8_t *)s, strlen(s));
////}
//
////void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huartx){
////
////	if(huartx->Instance == USART1){
////		HAL_UART_Receive_IT(&huart1,&uart1_data,1);
////	}else if(huartx->Instance == USART2){
////		HAL_UART_Receive_IT(&huart2,&uart2_data,1);
////	}else if(huartx->Instance == USART3){
////		HAL_UART_Receive_IT(&huart3,&uart3_data,1);
////	}else if(huartx->Instance == UART4){
////		HAL_UART_Receive_IT(&huart4,&uart4_data,1);
////	}else if(huartx->Instance == UART5){
////		HAL_UART_Receive_IT(&huart5,&uart5_data,1);
////	}else{
////		HAL_UART_Receive_IT(&huart6,&uart6_data,1);
////	}
////
////}
//
//
//// Handlers
//
//void  USART1_IRQHandler(void){
//
//
//	HAL_UART_IRQHandler(&huart1);
//}
//
//void  USART2_IRQHandler(void){
//
//	HAL_UART_IRQHandler(&huart2);
//}
//
//void  USART3_IRQHandler(void){
//
//	HAL_UART_IRQHandler(&huart3);
//}
//
////void  UART4_IRQHandler(void){
////
////	HAL_UART_IRQHandler(&huart4);
////}
//
//void UART5_IRQHandler(void)
//{
//	HAL_UART_IRQHandler(&huart5);
//}
//
//void  USART6_IRQHandler(void){
//
//	HAL_UART_IRQHandler(&huart6);
//}
//
//
//void DMA2_Stream7_IRQHandler(void){
//	HAL_DMA_IRQHandler(&huart1_dma);
//}
//
//void DMA1_Stream6_IRQHandler(void){
//	HAL_DMA_IRQHandler(&huart2_dma);
//}
//
//void DMA1_Stream4_IRQHandler(void){
//	HAL_DMA_IRQHandler(&huart4_dma);
//}
//
//void DMA1_Stream3_IRQHandler(void){
//	HAL_DMA_IRQHandler(&huart3_dma);
//}
//
//void DMA2_Stream6_IRQHandler(void){
//	HAL_DMA_IRQHandler(&huart6_dma);
//}
//
//void DMA1_Stream7_IRQHandler(void){
//	HAL_DMA_IRQHandler(&huart5_dma);
//}
//
