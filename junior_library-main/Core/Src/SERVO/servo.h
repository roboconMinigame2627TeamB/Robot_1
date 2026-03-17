/************************************************
 * Title   : SERVO
 * Author  : Kai Sheng, Qiu Hui, and Anas
 * Version : 1.00
 * Date    : 13 JULY 2017
 * **********************************************
 * Descriptions:
 *	- provide the function for controlling the servo.
 *
 * Version History:
 *
 *  1.0 - Converted to Hal Library
 *
 *
 * Bugs:
 *
 ************************************************/

#ifndef SERVO_H_
#define SERVO_H_

/***************************************
 * 		Include Libraries 			   *
 **************************************/

#include "../BIOS/bios.h"


/**************************************************
 * 		Structure							  	  *
 *************************************************/
typedef struct{
	TIM_HandleTypeDef* htimx ;
	uint32_t htimx_Channel;
	uint32_t SERVO_pulse0degree;
	uint32_t SERVO_pulse1degree;
	uint32_t TIMx_Compare;
}SERVO_t;



/**************************************************
 * 		Function Prototype			  			  *
 *************************************************/
void ServoxInit(SERVO_t* servo, TIM_HandleTypeDef* htimx, GPIO_TypeDef * SERVO_GPIOx,uint16_t SERVO_GPIO_Pin, uint32_t channel);
void ServoSetPulse(SERVO_t* servo, uint32_t pulse);
void ServoInitAngle(SERVO_t* servo, uint32_t pulse0degree , uint32_t pulse180degree);
void ServoSetAngle(SERVO_t* servo,uint8_t angle);


#endif /* SERVO_H_ */
