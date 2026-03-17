/*
 * r6091u.c
 *
 *  Created on: Aug 5, 2022
 *      Author: wai
 */

#include "r6091u.h"

void IMU_Init(R6091U_t* IMU, UART_HandleTypeDef* huartx){
	IMU->huartx = huartx;
	IMU->State = PENDING_SYNC;
	IMU->checksum = 0;
	IMU->offset = 0;
	IMU->yaw_constant = 0;
	IMU->prev_yaw = 0;
	HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);
//	KalmanFilterInit(&IMU->y_acc, &IMU->y_acc_filtered, 0.06, 6, 100, &IMU->kf);
	Moving_Average_Init(&IMU->ma, &IMU->x_acc, &IMU->y_acc_filtered);
}

void IMU_InitI2C(R6091U_t* IMU, I2C_HandleTypeDef *hi2c){
//	I2CxInit(hi2c, main_board_2, CLOCK_SPEED_100KHz, ENABLE);
	IMU->hi2cimu = hi2c;
	IMU->checksum = 0;
	IMU->offset = 0;
	IMU->yaw_constant = 0;
	IMU->prev_yaw = 0;
	HAL_I2C_Master_Receive_IT(IMU->hi2cimu, 0x35 << 1, (uint8_t*)IMU->Buffer, 20);
}

void IMU_InitI2C_DMA(R6091U_t* IMU, I2C_HandleTypeDef *hi2c){
	IMU->hi2cimu = hi2c;
	IMU->checksum = 0;
	IMU->offset = 0;
	IMU->yaw_constant = 0;
	IMU->prev_yaw = 0;
	HAL_I2C_Master_Receive_DMA(IMU->hi2cimu, 0x35 << 1, (uint8_t*)IMU->Buffer, 20);
}

void IMU_I2CHandle(R6091U_t* IMU){
	IMU->checksum = 0;
	for(int i = 0; i < 19; i++)
		IMU->checksum += IMU->Buffer[i];

	if(IMU->checksum == IMU->Buffer[19]){
		IMU->roll = *((int16_t *)&IMU->Buffer[0]) / 100.0;
		IMU->pitch = *((int16_t *)&IMU->Buffer[2]) / 100.0; 	 // -90 to 90
		IMU->yaw = *((int16_t *)&IMU->Buffer[4]) / 100.0;	 //-180 to 180
		IMU->roll_rate = *((int16_t *)&IMU->Buffer[6]) / 100.0;
		IMU->pitch_rate = *((int16_t *)&IMU->Buffer[8]) / 100.0;
		IMU->yaw_rate = *((int16_t *)&IMU->Buffer[10]) / 100.0;
		IMU->x_acc = *((int16_t *)&IMU->Buffer[12]) / 1000 * 9.8067;
		IMU->y_acc = *((int16_t *)&IMU->Buffer[14]) / 1000 * 9.8067;
		IMU->z_acc = *((int16_t *)&IMU->Buffer[16]) / 1000 * 9.8067;
		IMU->index = IMU->Buffer[18];
	}

	if(IMU->yaw < -150.0){
		if(IMU->prev_yaw > 150.0){
			IMU->yaw_constant++;
		}
	}else if(IMU->yaw > 150.0){
		if(IMU->prev_yaw < -150.0){
			IMU->yaw_constant--;
		}
	}

	IMU->prev_yaw = IMU->yaw;
	IMU->real_z = IMU->yaw + IMU->yaw_constant * 360.0 + IMU->offset;
	IMU->real_zrad = (IMU->real_z / 180.0) * 3.141593;
	memset(IMU->Buffer, 0, 20);
	HAL_I2C_Master_Receive_IT(IMU->hi2cimu, 0x35 << 1, (uint8_t*)IMU->Buffer, 20);
}

void IMU_DMAHandle(R6091U_t* IMU){
	IMU->checksum = 0;
	for(int i = 0; i < 19; i++)
		IMU->checksum += IMU->Buffer[i];

	if(IMU->checksum == IMU->Buffer[19]){
		IMU->roll = *((int16_t *)&IMU->Buffer[0]) / 100.0;
		IMU->pitch = *((int16_t *)&IMU->Buffer[2]) / 100.0; 	 // -90 to 90
		IMU->yaw = *((int16_t *)&IMU->Buffer[4]) / 100.0;	 //-180 to 180
		IMU->roll_rate = *((int16_t *)&IMU->Buffer[6]) / 100.0;
		IMU->pitch_rate = *((int16_t *)&IMU->Buffer[8]) / 100.0;
		IMU->yaw_rate = *((int16_t *)&IMU->Buffer[10]) / 100.0;
		IMU->x_acc = *((int16_t *)&IMU->Buffer[12]) / 1000 * 9.8067;
		IMU->y_acc = *((int16_t *)&IMU->Buffer[14]) / 1000 * 9.8067;
		IMU->z_acc = *((int16_t *)&IMU->Buffer[16]) / 1000 * 9.8067;
		IMU->index = IMU->Buffer[18];
	}

	if(IMU->yaw < -150.0){
		if(IMU->prev_yaw > 150.0){
			IMU->yaw_constant++;
		}
	}else if(IMU->yaw > 150.0){
		if(IMU->prev_yaw < -150.0){
			IMU->yaw_constant--;
		}
	}

	IMU->prev_yaw = IMU->yaw;
	IMU->real_z = IMU->yaw + IMU->yaw_constant * 360.0 + IMU->offset;
	IMU->real_zrad = (IMU->real_z / 180.0) * 3.141593;
	memset(IMU->Buffer, 0, 20);
	HAL_I2C_Master_Receive_DMA(IMU->hi2cimu, 0x35<<1, (uint8_t*)&IMU->Buffer, 20);//RECEIVE FROM R6091U
}

void IMU_Handler(R6091U_t* IMU){
//	static uint32_t imuledtick = 0;
	switch(IMU->State){
	case PENDING_SYNC:

		if(IMU->Buffer[0] == 0xAA){
			IMU->State = CONFIRMING_SYNC;
			HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);
			break;
		}
		else{
			IMU->State = PENDING_SYNC;
			HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);
			break;
		}

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
			IMU->index = IMU->Buffer[0];
			IMU->yaw = *((int16_t *)&IMU->Buffer[1]) / 100.0;
			IMU->yaw_rate = *((int16_t *)&IMU->Buffer[3]) / 100.0;
			IMU->x_acc = *((int16_t *)&IMU->Buffer[5]) / 1000.0 * 9.8067;
			IMU->y_acc = *((int16_t *)&IMU->Buffer[7]) / 1000.0 * 9.8067;
			IMU->z_acc = *((int16_t *)&IMU->Buffer[9]) / 1000.0 * 9.8067;
			IMU->turn_no = *((int8_t *)&IMU->Buffer[11]);
		}

		if(IMU->yaw < -150.0){
			if(IMU->prev_yaw > 150.0){
				IMU->yaw_constant++;
			}
		}else if(IMU->yaw > 150.0){
			if(IMU->prev_yaw < -150.0){
				IMU->yaw_constant--;
			}
		}
//		imuledtick ++;
//		if(imuledtick >= 50){
//			GPIOC_OUT->bit14 = !GPIOC_OUT->bit14;	//led2
//			imuledtick = 0;
//		}
//		KalmanFilter(&IMU->kf);
		Moving_Average_Filter(&IMU->ma);
		IMU->prev_yaw = IMU->yaw;
		IMU->real_z = IMU->yaw + IMU->yaw_constant * 360.0 + IMU->offset;
		IMU->real_zrad = (IMU->real_z / 180.0) * 3.141593;
		IMU->State = PENDING_SYNC;
		HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);
		break;

	}
}
