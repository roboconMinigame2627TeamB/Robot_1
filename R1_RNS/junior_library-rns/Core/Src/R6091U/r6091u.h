/*
 * r6091u.h
 *
 *  Created on: Oct 10, 2021
 *      Author: amera
 */

#ifndef SRC_R6091U_R6091U_H_
#define SRC_R6091U_R6091U_H_

#include "../BIOS/bios.h"


enum {PENDING_SYNC = 0, CONFIRMING_SYNC, IN_SYNC};

typedef struct {

	UART_HandleTypeDef* huartx;
	uint8_t State;
	uint8_t checksum;
	uint8_t Buffer[13];

}R6091U_t;

void R6091U_Init(R6091U_t* IMU,UART_HandleTypeDef* huartx);
void R6091U_Handler(R6091U_t* IMU);
#endif /* SRC_R6091U_R6091U_H_ */
