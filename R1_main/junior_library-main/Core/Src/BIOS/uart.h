/************************************************
 * Title   : UART
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 10/12/2020
 * **********************************************
 * Descriptions: Send and receive data using UART
 *
 *
 * Version History:
 * 1.0 - implemented using hal library
 *
 * Bugs:
 *
 ************************************************/

#ifndef BIOS_UART_H_
#define BIOS_UART_H_

/***************************************
 * 		Include Libraries 			   *
 **************************************/
#include "system.h"

/**************************************************
 * 		STRUCTURE DEFINES					  	  *
 *************************************************/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;


DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
/**************************************************
 * 		Function Prototype					  	  *
 *************************************************/
void UARTInit(UART_HandleTypeDef* huartx, uint32_t baudrate, FunctionalState rxstate, uint16_t preemptionpriority,uint16_t subpriority);
void UARTx_DMA_Rx_Init(UART_HandleTypeDef* huartx, DMA_HandleTypeDef* hdma_usart_rx, uint16_t preemptionpriority, uint16_t subpriority);
void UARTx_DMA_Tx_Init(UART_HandleTypeDef* huartx, DMA_HandleTypeDef* hdma_usart_tx, uint16_t preemptionpriority, uint16_t subpriority);
void UARTPrintString(UART_HandleTypeDef* huartx, char s[]);
void UARTPrintString_IT(UART_HandleTypeDef* huartx, char s[]);
char ReadUART(UART_HandleTypeDef* huartx);
uint16_t USART_ReceiveData(UART_HandleTypeDef* huartx);

uint8_t uart1_data,uart2_data,uart3_data,uart4_data,uart5_data,uart6_data;
#endif /* BIOS_UART_H_ */

///************************************************
// * Title   : UART
// * Author  : Anas Amer
// * Version : 1.0
// * Date    : 10/12/2020
// * **********************************************
// * Descriptions: Send and receive data using UART
// *
// *
// * Version History:
// * 1.0 - implemented using hal library
// *
// * Bugs:
// *
// ************************************************/
//
//#ifndef BIOS_UART_H_
//#define BIOS_UART_H_
//
///***************************************
// * 		Include Libraries 			   *
// **************************************/
//#include "system.h"
//
///**************************************************
// * 		STRUCTURE DEFINES					  	  *
// *************************************************/
//UART_HandleTypeDef huart1;
//UART_HandleTypeDef huart2;
//UART_HandleTypeDef huart3;
//UART_HandleTypeDef huart4;
//UART_HandleTypeDef huart5;
//UART_HandleTypeDef huart6;
//
//DMA_HandleTypeDef huart1_dma;
//DMA_HandleTypeDef huart2_dma;
//DMA_HandleTypeDef huart3_dma;
//DMA_HandleTypeDef huart4_dma;
//DMA_HandleTypeDef huart5_dma;
//DMA_HandleTypeDef huart6_dma;
///**************************************************
// * 		Function Prototype					  	  *
// *************************************************/
//void UARTInit(UART_HandleTypeDef* huartx, uint32_t baudrate, FunctionalState rxstate, uint16_t preemptionpriority,uint16_t subpriority);
//void UART_DMA_TX_Init(UART_HandleTypeDef* huartx,DMA_HandleTypeDef* huart_dmax , uint32_t baudrate, uint16_t preemptionpriority, uint16_t subpriority);
//void UARTPrintString(UART_HandleTypeDef* huartx, char s[]);
//void UARTPrintString_IT(UART_HandleTypeDef* huartx, char s[]);
//char ReadUART(UART_HandleTypeDef* huartx);
//uint16_t USART_ReceiveData(UART_HandleTypeDef* huartx);
//
//uint8_t uart1_data,uart2_data,uart3_data,uart4_data,uart5_data,uart6_data;
//#endif /* BIOS_UART_H_ */
