/*
 * r6091u.c
 *
 *  Created on: Oct 10, 2021
 *      Author: amera
 */

#include "r6091u.h"
#include "../include.h"




void R6091U_Init(R6091U_t* IMU,UART_HandleTypeDef* huartx){

	IMU->huartx = huartx;
	IMU->State = PENDING_SYNC;
	HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);

}


void R6091U_Handler(R6091U_t* IMU){

//	uint8_t checksum;

	switch(IMU->State){


	case PENDING_SYNC:

		if(IMU->Buffer[0] == 0xAA){
			IMU->State = CONFIRMING_SYNC;
		}

		HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);
		break;

	case CONFIRMING_SYNC:

		if(IMU->Buffer[0] == 0x00){
			IMU->State = IN_SYNC;
			HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 13);

		}else{

			IMU->State = PENDING_SYNC;
			HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);
		}

		break;

	case IN_SYNC:
		IMU->checksum = 0;

		IMU->checksum = IMU->Buffer[0] + IMU->Buffer[1] + IMU->Buffer[2] + IMU->Buffer[3] + IMU->Buffer[4] + IMU->Buffer[5]
						+ IMU->Buffer[6] + IMU->Buffer[7] + IMU->Buffer[8] + IMU->Buffer[9] + IMU->Buffer[10] + IMU->Buffer[11];


		if( IMU->checksum == IMU->Buffer[12]){
//		int16_t yaw = ((IMU->Buffer[1] & 0xFF)) | ((IMU->Buffer[2] << 8) & 0xFF00) ;
			int16_t yaw = *((uint16_t*)&IMU->Buffer[1]);
			fyaw = ((float)(yaw) / (float)100.0) + 180.0 ;
			if(testCounter2 >= 20){
				LED3 = !LED3;
				testCounter2 = 0;
			}else
				testCounter2 ++;
		}
		memset(IMU->Buffer, 0, 13);
		IMU->State = PENDING_SYNC;
		HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);

		break;
	}
}
