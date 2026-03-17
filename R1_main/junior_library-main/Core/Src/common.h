/*******************************************************************************
 * Title   : common.h
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 12/10
 *******************************************************************************
 * Description: Sensor and function definitions
 *
 * Version History:
 *  1.0 - converted to hal library
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "adapter.h"
#include "cmsis_os.h"


//#define printPressure

#define lsf1  		HAL_GPIO_ReadPin(IP1_PIN)
#define IP2  		HAL_GPIO_ReadPin(IP2_PIN)
#define lsfl1  		HAL_GPIO_ReadPin(IP3_PIN)
#define lsfl2 		HAL_GPIO_ReadPin(IP4_PIN)
#define lsfb 		HAL_GPIO_ReadPin(IP5_PIN)
#define lsb1 		HAL_GPIO_ReadPin(IP6_PIN)
#define lsb2	    HAL_GPIO_ReadPin(IP7_PIN)
#define lsf2 		HAL_GPIO_ReadPin(IP8_PIN)
#define IP9  		HAL_GPIO_ReadPin(IP9_PIN)
#define IP10    	HAL_GPIO_ReadPin(IP10_PIN)
#define IP11  		HAL_GPIO_ReadPin(IP11_PIN)
#define IP12 		HAL_GPIO_ReadPin(IP12_PIN)
#define IP13  		HAL_GPIO_ReadPin(IP13_PIN)
#define IP14 		HAL_GPIO_ReadPin(IP14_PIN)
#define IP15		HAL_GPIO_ReadPin(IP15_PIN)


//ANALOG PIN//
#define IP16		HAL_GPIO_ReadPin(IP16_Analog1_PIN)
#define IP17		HAL_GPIO_ReadPin(IP17_Analog2_PIN)
#define	IP18   		HAL_GPIO_ReadPin(IP18_Analog3_PIN)
#define IP19		HAL_GPIO_ReadPin(IP19_Analog4_PIN)
#define IP20		HAL_GPIO_ReadPin(IP20_Analog5_PIN)
#define IP21		HAL_GPIO_ReadPin(IP21_Analog6_PIN)


#define Mux1		 MUX.mux_data.bit0
#define Mux2		 MUX.mux_data.bit1
#define Mux3		 MUX.mux_data.bit2
#define Mux4		 MUX.mux_data.bit3
#define Mux5		 MUX.mux_data.bit4
#define Mux6		 MUX.mux_data.bit5
#define Mux7		 MUX.mux_data.bit6
#define Mux8		 MUX.mux_data.bit7

void RNS_config(CAN_HandleTypeDef* hcanx);
void set(void);
void manual_mode(void);
void Navi(void);
void enq(void);

void BlueBee(UART_HandleTypeDef *huart);

#endif /* INC_COMMON_H_ */
