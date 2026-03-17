
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "cmsis_os.h"
/**
 * @brief  The application entry point.
 * @retval int
 */

//#define PLOT

#ifdef PLOT
#define HEADER 0x67
#define NUM_FLOAT 7
osThreadId_t Debug_Task_Handle;
osSemaphoreId_t DebugSemaphore;
void Debug(void *argument);
#endif

int main(void)
{

	Initialize();

	const osThreadAttr_t Calculation_Task_attributes = {
			.name = "Calculation_Task",
			.stack_size = 1024 *  4,
			.priority = (osPriority_t) osPriorityNormal,
	};


	const osThreadAttr_t STTChecker_Task_attributes = {
			.name = "STTChecker_Task",
			.stack_size = 1024 * 4,
			.priority = (osPriority_t) osPriorityNormal,
	};
#ifdef PLOT
	const osThreadAttr_t Debug_Task_attributes = {
			.name = "Debug_Task",
			.stack_size = 1024 * 4,
			.priority = (osPriority_t) osPriorityNormal,
	};
	const osSemaphoreAttr_t DebugSemaphore_attributes = {
			.name = "DebugSemaphore"
	};
#endif
	const osSemaphoreAttr_t CalcSemaphore_attributes = {
			.name = "InitSemaphore"
	};


	osKernelInitialize();

	Calculation_Task_Handle = osThreadNew(Calculation, NULL, &Calculation_Task_attributes);
	STTChecker_Task_Handle = osThreadNew(STTChecker, NULL, &STTChecker_Task_attributes);
#ifdef PLOT
	Debug_Task_Handle = osThreadNew(Debug, NULL, &Debug_Task_attributes);
	DebugSemaphore = osSemaphoreNew(1, 0, &DebugSemaphore_attributes);
#endif
	CalcSemaphore = osSemaphoreNew(1, 0, &CalcSemaphore_attributes);

//	vTraceEnable(TRC_START);

	osKernelStart();


}

void TIM6_DAC_IRQHandler(void)
{
	osSemaphoreRelease(CalcSemaphore);
	testCounter1 = testCounter1 + 1;
#ifdef PLOT
	osSemaphoreRelease(DebugSemaphore);
#endif
//	if(!sys.vel_flag){
//		if(dev_cfg.PID_type == s_fuzzyPID)
//			sprintf(uartbuf, "%2.5f,%2.5f,%2.5f,%2.5f,%2.5f,%2.5f,%2.5f\n",
//					fleft_vel.K[KP],
//					fleft_vel.K[KI],
//					fleft_vel.K[KD],
//					fuz_fleft_pid.d_Error,
//					fuz_fleft_pid.delta_P,
//					fuz_fleft_pid.delta_I,
//					fuz_fleft_pid.delta_D);
//		else
//			sprintf(uartbuf, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",\
//					fuz_fleft_vel.K[KP_B], fuz_fleft_vel.K[KI_B], fuz_fleft_vel.K[KD_B],\
//					fuz_fleft_vel.K[KP_P], fuz_fleft_vel.K[KI_P], fuz_fleft_vel.K[KD_P]);
//		UART_DMA_PrintString(&huart5, uartbuf);
//	}
	HAL_TIM_IRQHandler(&htim6);
}



void Calculation(void *argument){
	int led=0;


	main_board_1_data_receive.common_instruction = RNS_PENDING;
	main_board_1_data_receive.common_buffer[0].data = 0;
	main_board_1_data_receive.common_buffer[1].data = 0;
	main_board_1_data_receive.common_buffer[2].data = 0;
	main_board_1_data_receive.common_buffer[3].data = 0;


	fFPtd=fFKcd=fBPtd=fBKcd=xPtd=yPtd=1.0;
	feedback_ins = 0;
	sys.flag = 0;
	fyaw=180.0;

	SYSSystemInit();
	STTStateInit();
	APPStop();
	APPResetPos();

	TIMxInit(&htim6, 5000, 84, 6, 0);


	while(1){
		osSemaphoreAcquire(CalcSemaphore,osWaitForever);
		SYSSystem5ms();
		SYSSystemAct();
		if (++led==4){
			//			if(APPPrintstatus()){
			//				APPPrinting(&uartbuff);
			//				UARTPrintString_IT(&huart5,uartbuf);
			//			}

			LED1=!LED1;
			led=0;
		}


	}

}


void STTChecker(void *argument){

	while(1){
		STTEventChecker();
		button();
		if(state != RNS_S_PENDING){
			LED2=0;
		} else {
			LED2=1;
		}
	}

}

#ifdef PLOT
void Debug(void *argument){
	char debugBuf[2][512];
	int turn=0;
//	uint8_t debugBuf[2][1 + NUM_FLOAT*4 + 1];
	uint8_t header = HEADER, checksum=0x76, totalByte=NUM_FLOAT*4 + 2;
	while(1){
		osSemaphoreAcquire(DebugSemaphore,osWaitForever);
		//CSV format

		if(sys.vel_flag){
			/*Velocity error*/
//						sprintf(debugBuf, "%2.5f,%2.5f,%2.5f,%2.5f\r\n", fFLeftVelErr,fFRightVelErr,fBLeftVelErr,fBRightVelErr);
			turn = !turn;
			sprintf(debugBuf[turn], "%2.5f,%2.5f,%.1f,%d,%d,%d,%2.5f,%d,%d,%2.5f\n",\
					fFLeftVelR,fFLeftVel,fFLeftVelU,\
					fuz_fleft_vel.set_Kp, fuz_fleft_vel.set_Ki, fuz_fleft_vel.set_Kd,\
					fuz_fleft_vel.errs, fuz_fleft_vel.errCal, fuz_fleft_vel.errcCal,fuz_fleft_vel.outt);
//			sprintf(debugBuf[turn], "%2.5f,%2.5f,%2.5f,%2.5f,%2.5f,%2.5f,%2.5f\n",
//					fleft_vel.K[KP], fleft_vel.K[KI], fleft_vel.K[KD], fuz_fleft_pid.d_Error, fuz_fleft_pid.delta_P, fuz_fleft_pid.delta_I, fuz_fleft_pid.delta_D);
			//[header][float1][][][][float2][][][][checksum]
//			memset(debugBuf[turn], 0, 13);
//			debugBuf[turn][0] = HEADER;
//			memcpy(&debugBuf[turn][1], &fleft_vel.K[KP], 4);
//			memcpy(&debugBuf[turn][5], &fleft_vel.K[KI], 4);
//			memcpy(&debugBuf[turn][9], &fleft_vel.K[KD], 4);
//			memcpy(&debugBuf[turn][13], &fuz_fleft_pid.d_Error, 4);
//			memcpy(&debugBuf[turn][17], &fuz_fleft_pid.delta_P, 4);
//			memcpy(&debugBuf[turn][21], &fuz_fleft_pid.delta_I, 4);
//			memcpy(&debugBuf[turn][25], &fuz_fleft_pid.delta_D, 4);

//			checksum = fleft_vel.K[KP] + fleft_vel.K[KI] + fleft_vel.K[KD];
//			debugBuf[turn][1+NUM_FLOAT*4] = checksum;
//			HAL_UART_Transmit_DMA(&huart5, debugBuf[turn], totalByte);
			HAL_UART_Transmit_DMA(&huart5, (uint8_t *)debugBuf[turn], strlen(debugBuf[turn]));
//			HAL_UART_Transmit_DMA(&huart5, &header, 1));

		}

		/*Velocity Reference*/
		//			sprintf(debugBuf, "%2.5f,%2.5f,%2.5f,%2.5f\r\n", fFLeftVelR,fFRightVelR,fBLeftVelR,fBRightVelR);

		/*Velocity estimated by ABT*/
		//			sprintf(debugBuf, "%2.5f,%2.5f,%2.5f,%2.5f\r\n", fFLeftVel,fFRightVel,fBLeftVel,fBRightVel);

		/*Postion in meter*/
		//			sprintf(debugBuf, "%2.5f,%2.5f,%2.5f,%2.5f\r\n", fFLeftPosData, fFRightPosData, fBLeftPosData, fBRightPosData);


	}
}
#endif
/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler(void)
{
	errorInit = errorInit + 1;

}
#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
