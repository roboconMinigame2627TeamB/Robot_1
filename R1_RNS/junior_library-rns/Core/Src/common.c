

/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "include.h"

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/
void Initialize(){
	/* MCU Configuration--------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */

	GPIOPinsInit (LED1_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit (LED2_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit (LED3_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit (LED4_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit (LED5_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);

	GPIOPinsInit (BUTTON1_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit (BUTTON2_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);

	LED1 = 1;
	LED2 = 1;
	LED3 = 1;
	LED4 = 1;
	LED5 = 1;

	SHIFTREGInit (&SR, NO_CASCADE, SR_SCK_PIN, SR_RCK_PIN, SR_SI_PIN);

	CANxInit(&hcan1,4,CAN_FILTER_FIFO0,0,0,13,5,0);

	QEIInit(&htim1,6,2);//C
	QEIInit(&htim2,7,0);//X
	QEIInit(&htim3,6,3);//D
	QEIInit(&htim4,6,1);//B
	QEIInit(&htim5,7,1);//Y
	QEIInit(&htim8,6,0);//A

//	CANxInit(&hcan1,3,CAN_FILTER_FIFO0,0,0,13,5,0);
	//	CANxInit(&hcan2,4,CAN_FILTER_FIFO1,0,0,27,5,2);

	UARTInit(&huart2, 115200, ENABLE, 5,0);
	UARTInit(&huart5, 115200, ENABLE,5,0);
	UARTx_DMA_Tx_Init(&huart5, &hdma_uart5_tx, 5, 0);

	PWMTimeBaseInit(&htim9, 20000, 4);
	PWMTimeBaseInit(&htim12, 20000, 2);

	PWMChannelConfig(&htim9, TIM_CHANNEL_1, PWM_TIM9_CH1_PIN);
	PWMChannelConfig(&htim9, TIM_CHANNEL_2, PWM_TIM9_CH2_PIN);
	PWMChannelConfig(&htim12, TIM_CHANNEL_1, PWM_TIM12_CH1_PIN);
	PWMChannelConfig(&htim12, TIM_CHANNEL_2, PWM_TIM12_CH2_PIN);

	BDCInit(&BDC1, &htim9, TIM_CHANNEL_1, SHIFTREG, &(SR.cast[0]), Bit0, Bit5);
	BDCInit(&BDC2, &htim9, TIM_CHANNEL_2, SHIFTREG, &(SR.cast[0]), Bit6, Bit7);
	BDCInit(&BDC3, &htim12, TIM_CHANNEL_1, SHIFTREG, &(SR.cast[0]), Bit4, Bit3);
	BDCInit(&BDC4, &htim12, TIM_CHANNEL_2, SHIFTREG, &(SR.cast[0]), Bit1, Bit2);


	//	LSAInit(&LSA_A, S1_A_PIN, S2_A_PIN, S3_A_PIN, S4_A_PIN, S5_A_PIN, S6_A_PIN, S7_A_PIN, S8_A_PIN);
	//	LSAInit(&LSA_B, S1_B_PIN, S2_B_PIN, S3_B_PIN, S4_B_PIN, S5_B_PIN, S6_B_PIN, S7_B_PIN, S8_B_PIN);
	//	LSAInit(&LSA_C, S1_C_PIN, S2_C_PIN, S3_C_PIN, S4_C_PIN, S5_C_PIN, S6_C_PIN, S7_C_PIN, S8_C_PIN);
	//	LSAInit(&LSA_D, S1_D_PIN, S2_D_PIN, S3_D_PIN, S4_D_PIN, S5_D_PIN, S6_D_PIN, S7_D_PIN, S8_D_PIN);
	R6091U_Init(&IMU, &huart2);

}


void button (void){

	if (BUTTON1 == 0){
		LED2_ON;
		mmode++;
		while(BUTTON1 == 0) {
			LED2_ON;
		}
		switch(mmode){
		case 1:
			sprintf(uartbuf, "Mode 1: Test all wheels forward direction, Press PB2 to start\r\n");
			UART_DMA_PrintString(&huart5,uartbuf);
			break;
		case 2:
			sprintf(uartbuf, "Mode 2: Test all wheels backward direction\r\n");
			UART_DMA_PrintString(&huart5,uartbuf);
			break;
		case 3:
			sprintf(uartbuf, "Mode 3: Stop all wheels\r\n");
			UART_DMA_PrintString(&huart5,uartbuf);
			break;
		case 4:
			sprintf(uartbuf, "Mode 4: Print all wheels encoder value and angle\r\n");
			UART_DMA_PrintString(&huart5,uartbuf);
			break;
		case 5:
			sprintf(uartbuf, "Mode 5: Set KCD PTD. Turn all wheels in positive encoder value for 10 turns\r\n");
			UART_DMA_PrintString(&huart5,uartbuf);
			osDelay(50);
			sprintf(uartbuf, "short press PB2 to select wheel diameter 12.5cm\r\n");
			UART_DMA_PrintString(&huart5,uartbuf);
			osDelay(50);
			sprintf(uartbuf, "or long press PB2 for more than 300ms to select wheel diameter 15cm\r\n");
			UART_DMA_PrintString(&huart5,uartbuf);
			osDelay(50);
			sprintf(uartbuf, "Press PB2 again to set KCD PTD\r\n");
			UART_DMA_PrintString(&huart5,uartbuf);
			break;
		case 6:
			sprintf(uartbuf, "\tMode 6: Print all wheels max speed\r\n");
			UART_DMA_PrintString(&huart5,uartbuf);
			break;
		default:
			mmode = 0;
		}
		LED2_OFF;
	} else if (BUTTON2 == 0) {
		float a, b, c, d, wheelDiameter; a=b=c=d=0.0;
		testCounter1 = 0;
		while(BUTTON2 == 0){
			LED2_ON;
		}
		LED2_OFF;
		switch(mmode){
		case 0:
			LED2=1;
			break;

		case 1:
			ins.instruction = RNS_PDC;
			ins.ins_buffer[0].data = 4000;
			ins.ins_buffer[1].data = 4000;
			ins.ins_buffer[2].data = 4000;
			ins.ins_buffer[3].data = 4000;

			APPApply(&ins);
			APPStart(&ins);
			sys.activate=1;
			LED3_ON;
			break;

		case 2:
			ins.instruction = RNS_PDC;
			ins.ins_buffer[0].data =-4000;
			ins.ins_buffer[1].data =-4000;
			ins.ins_buffer[2].data =-4000;
			ins.ins_buffer[3].data =-4000;

			APPApply(&ins);
			APPStart(&ins);
			sys.activate=1;
			LED3_OFF;
			LED4_ON;
			break;

		case 3:
			APPStop();
			LED4_OFF;
			break;

		case 4:
			while(BUTTON2 == 1){
				//				if(HAL_UART_GetState(&huart5) != HAL_UART_STATE_BUSY_TX)
//				sprintf(uartbuf,"x=%.2f,y=%.2f,A=%.2f,B=%.2f,C=%.2f,D=%.2f,w=%.2f,error=%d\r\n", // @suppress("Float formatting support")
//						fXEncData, fYEncData, fFLeftPosData, fFRightPosData, fBLeftPosData,fBRightPosData,fyaw,errorCounter);//in meter
				sprintf(uartbuf,"x=%ld,y=%ld,A=%ld,B=%ld,C=%ld,D=%ld,w=%.2f,error=%d\r\n", // @suppress("Float formatting support")
							QEIRead(QEI2),QEIRead(QEI5),QEIRead(QEI6),QEIRead(QEI4),QEIRead(QEI1),QEIRead(QEI3),fyaw,errorInit);//in meter
				//				sprintf(uartbuf,"Rec: %x , Check: %x \r\n",(uint8_t)(IMU.Buffer[12],(uint8_t)( IMU.Buffer[0] + IMU.Buffer[1] + IMU.Buffer[2] + IMU.Buffer[3] + IMU.Buffer[4] + IMU.Buffer[5]+ IMU.Buffer[6] + IMU.Buffer[7] + IMU.Buffer[8] + IMU.Buffer[9] + IMU.Buffer[10] + IMU.Buffer[11])));
				UART_DMA_PrintString(&huart5,uartbuf);
				LED5_ON;
			}
			while(BUTTON2 == 0);
			mmode = 0;
			LED5_OFF;
			break;
		case 5:
			if(testCounter1 >= 60) wheelDiameter = 0.15;
			else wheelDiameter = 0.125;
			sprintf(uartbuf, "Wheel of diameter %.2f selected\r\n", wheelDiameter);
			UART_DMA_PrintString(&huart5, uartbuf);
			osDelay(2000);
			while(BUTTON2 == 1){
				sprintf(uartbuf,"A=%.2f,B=%.2f,C=%.2f,D=%.2f\r\n", // @suppress("Float formatting support")
						fFLeftPosData, fFRightPosData, fBLeftPosData,fBRightPosData);
				UART_DMA_PrintString(&huart5,uartbuf);
				LED5_ON;
			}
			fFKcd = fFLeftPosData/fFRightPosData;
			fFPtd = wheelDiameter*3.142/(fFLeftPosData/10);
			fBKcd = fBLeftPosData/fBRightPosData;
			fBPtd = wheelDiameter*3.142/(fBLeftPosData/10);
			sprintf(uartbuf,"KCD PTD set\r\n");
			UART_DMA_PrintString(&huart5,uartbuf);
			sprintf(uartbuf,"You may check maximum speed now\r\n");
			UART_DMA_PrintString(&huart5,uartbuf);
			while(BUTTON2 == 0);
			mmode = 6;
			LED5_OFF;
		case 6:
			while(BUTTON2 == 1){
				if(fFLeftVel  > a) a = fFLeftVel;
				if(fFRightVel > b) b = fFRightVel;
				if(fBLeftVel  > c) c = fBLeftVel;
				if(fBRightVel > d) d = fBRightVel;
				sprintf(uartbuf,"A=%.2f,B=%.2f,C=%.2f,D=%.2f\r\n", a,b,c,d);//in meter
				UART_DMA_PrintString(&huart5,uartbuf);
				LED5_ON;
			}
			while(BUTTON2 == 0);
			mmode = 0;
			LED5_OFF;
			break;
		default:
			mmode = 0;
			break;
		}
	}
}


void CAN1_RX0_IRQHandler()
{

	HAL_CAN_IRQHandler(&hcan1);

}

void CAN2_RX1_IRQHandler()
{

	HAL_CAN_IRQHandler(&hcan2);

}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t aData[8];

	if(hcan == &hcan1){
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0 , &CAN1RxMessage, aData);
		switch (CAN1RxMessage.StdId) {

		case mainboard_TO_RNS:
			memcpy(&insData_receive, &aData, CAN1RxMessage.DLC);
			rcv_buf_flag = 0;
			break;

		case mainboard_TO_RNS_buf1:
			memcpy(&rcv_buf[0], &aData, CAN1RxMessage.DLC);
			break;

		case mainboard_TO_RNS_buf2:
			memcpy(&rcv_buf[2], &aData, CAN1RxMessage.DLC);
			rcv_buf_flag = 1;
			break;

		case mainboard_TO_RNS_buf3 :
			memcpy(&rcv_buf[4], &aData, CAN1RxMessage.DLC);
			rcv_buf_flag=2;
			break;

		case mainboard_TO_RNS_buf4 :
			memcpy(&rcv_buf[6], &aData, CAN1RxMessage.DLC);
			rcv_buf_flag=3;
			LED4=0;
			break;

		case CONT_data:
			main_board_1_data_receive.common_instruction = RNS_CONTROLLER;
			main_board_1_data_receive.common_buffer[0].byte1 = aData[0];
			main_board_1_data_receive.common_buffer[0].byte2 = aData[1];
			main_board_1_data_receive.common_buffer[0].byte3 = aData[2];
			main_board_1_data_receive.common_buffer[0].byte4 = aData[3];
			main_board_1_data_receive.common_buffer[1].data = 0;
			main_board_1_data_receive.common_buffer[2].data = 0;
			main_board_1_data_receive.common_buffer[3].data = 0;
			insData_receive[0]=19;

			break;

		default:
			break;
		}

		if (insData_receive[0] == 1) {
			main_board_1_data_receive.common_instruction = insData_receive[1];
		}

		if (insData_receive[0] == 17) {
			if (rcv_buf_flag == 1 && (insData_receive[1] < RNS_PARAM_5 || insData_receive[1]>RNS_INS_PARAM)) {
				main_board_1_data_receive.common_instruction = insData_receive[1];
				main_board_1_data_receive.common_buffer[0].data = rcv_buf[0].data;
				main_board_1_data_receive.common_buffer[1].data = rcv_buf[1].data;
				main_board_1_data_receive.common_buffer[2].data = rcv_buf[2].data;
				main_board_1_data_receive.common_buffer[3].data = rcv_buf[3].data;
			}
			if(rcv_buf_flag == 3){
				main_board_1_data_receive.common_instruction = insData_receive[1];
				main_board_1_data_receive.common_buffer[0].data = rcv_buf[0].data;
				main_board_1_data_receive.common_buffer[1].data = rcv_buf[1].data;
				main_board_1_data_receive.common_buffer[2].data = rcv_buf[2].data;
				main_board_1_data_receive.common_buffer[3].data = rcv_buf[3].data;
				main_board_1_data_receive.common_buffer[4].data = rcv_buf[4].data;
				main_board_1_data_receive.common_buffer[5].data = rcv_buf[5].data;
				main_board_1_data_receive.common_buffer[6].data = rcv_buf[6].data;
			}
		}
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	}
}


void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)

{
	uint8_t aData[8];

	if(hcan == &hcan2){

		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1 , &CAN2RxMessage, aData);
		switch(CAN2RxMessage.StdId){

		case mainboard_TO_RNS:
			memcpy(&insData_receive, &aData, CAN2RxMessage.DLC);
			rcv_buf_flag2 = 0;
			break;

		case mainboard_TO_RNS_buf1:
			memcpy(&rcv_buf[0], &aData, CAN2RxMessage.DLC);
			break;

		case mainboard_TO_RNS_buf2:
			memcpy(&rcv_buf[2], &aData, CAN2RxMessage.DLC);
			rcv_buf_flag2=1;
			break;

		case mainboard_TO_RNS_buf3 :
			memcpy(&rcv_buf[4], &aData, CAN2RxMessage.DLC);
			rcv_buf_flag2=2;
			break;

		case mainboard_TO_RNS_buf4 :
			memcpy(&rcv_buf[6], &aData, CAN2RxMessage.DLC);
			rcv_buf_flag2=3;
			LED4=0;
			break;

		case CONT_data:
			main_board_1_data_receive.common_instruction = RNS_CONTROLLER;
			main_board_1_data_receive.common_buffer[0].byte1 = aData[0];
			main_board_1_data_receive.common_buffer[0].byte2 = aData[1];
			main_board_1_data_receive.common_buffer[0].byte3 = aData[2];
			main_board_1_data_receive.common_buffer[0].byte4 = aData[3];
			main_board_1_data_receive.common_buffer[1].data = 0;
			main_board_1_data_receive.common_buffer[2].data = 0;
			main_board_1_data_receive.common_buffer[3].data = 0;
			insData_receive[0]=19;
			break;

		default:
			break;

		}

		if (insData_receive[0] == 1) {
			main_board_1_data_receive.common_instruction = insData_receive[1];

		}

		if (insData_receive[0] == 17) {
			if (rcv_buf_flag2 == 1 && (insData_receive[1] < RNS_PARAM_5 || insData_receive[1]>RNS_INS_PARAM)) {
				main_board_1_data_receive.common_instruction = insData_receive[1];
				main_board_1_data_receive.common_buffer[0].data = rcv_buf[0].data;
				main_board_1_data_receive.common_buffer[1].data = rcv_buf[1].data;
				main_board_1_data_receive.common_buffer[2].data = rcv_buf[2].data;
				main_board_1_data_receive.common_buffer[3].data = rcv_buf[3].data;
			}
			if(rcv_buf_flag2==3){
				main_board_1_data_receive.common_instruction = insData_receive[1];
				main_board_1_data_receive.common_buffer[0].data = rcv_buf[0].data;
				main_board_1_data_receive.common_buffer[1].data = rcv_buf[1].data;
				main_board_1_data_receive.common_buffer[2].data = rcv_buf[2].data;
				main_board_1_data_receive.common_buffer[3].data = rcv_buf[3].data;
				main_board_1_data_receive.common_buffer[4].data = rcv_buf[4].data;
				main_board_1_data_receive.common_buffer[5].data = rcv_buf[5].data;
				main_board_1_data_receive.common_buffer[6].data = rcv_buf[6].data;
				main_board_1_data_receive.common_buffer[7].data = rcv_buf[7].data;
			}
		}

		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	}

}
