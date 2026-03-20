

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */

//HANDLES
osThreadId_t heartBeatHandle;
osThreadId_t movementHandle;
osThreadId_t servoHandle;
osThreadId_t pneumaticsHandle;

//SEMAPHORES & MUTEXES

//ATTRIBUTE DEFINITIONS
const osThreadAttr_t heartBeatTaskAttributes = {
		.name = "heartBeatTask",
		.stack_size = 512,
		.priority = (osPriority_t) osPriorityIdle,
};

const osThreadAttr_t movementTaskAttributes = {
		.name = "movementTask",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t servoTaskAttributes = {
		.name = "servoTask",
		.stack_size = 1024,
		.priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t pneumaticsTaskAttributes = {
		.name = "pneumaticsTask",
		.stack_size = 1024,
		.priority = (osPriority_t) osPriorityNormal,
};

//GLOBAL VARIABLES

///COMMON MOVEMENT & ANGLE VARIABLES
float FL, FR, BL, BR;
float speedCap;
volatile float currentHeading = 0, targetHeading = 0;

///SERVO VARIABLES
SERVO_t serv1, serv2, serv3, serv4;
volatile uint8_t servoControlActive = 0;

///PNEUMATICS VARIABLES
uint8_t pneumaticsToggle = 0;

///PERSPECTIVE CONTROL VARIABLES
volatile float referenceHeading = 0.0;
uint8_t perspectiveControlMode = 0;
uint8_t perspectiveControlToggle = 0;

//FUNCTION PROTOTYPES
void botInit(void);
void analogueMovement(void);
void perspectiveControlProcess(float* x, float* y);

//TASK PROTOTYPES
void HeartBeat(void *argument);
void MovementTask(void *argument);
void servoTask(void *argument);
void pneumaticsTask(void *arguement);

//FUNCTION DEFINITIONS
void botInit(void) {
	PSxSlaveInit(&ps4, &hi2c1);

	ServoxInit(&serv1, &htim3, TIM3_CHANNEL4_PIN, TIM_CHANNEL_4);
	ServoxInit(&serv2, &htim3, TIM3_CHANNEL3_PIN, TIM_CHANNEL_3);
	ServoxInit(&serv3, &htim9, TIM9_CHANNEL1_PIN, TIM_CHANNEL_1);
	ServoxInit(&serv4, &htim9, TIM9_CHANNEL2_PIN, TIM_CHANNEL_2);

	RNSEnquire(RNS_X_Y_IMU_LSA, &rns);
	targetHeading = rns.enq.enq_buffer[0].data;
	currentHeading = rns.enq.enq_buffer[0].data;
	referenceHeading = rns.enq.enq_buffer[0].data;
}

//Processes user data for movement
//NOTE: STILL HAVE NOT IMPLEMENTED IMU LOCK
void analogueMovement(void) {
	float tempFL, tempFR, tempBL, tempBR;
	float y = ps4.joyL_y, rot_y = 0.0;
	float x = ps4.joyL_x, rot_x = 0.0;
	float w = 0.0;
	float deadzone = 0.1;

	if (fabs(x) < deadzone) x = 0.0;
	if (fabs(y) < deadzone) y = 0.0;

	if (perspectiveControlMode) perspectiveControlProcess(&x, &y);

	tempFL = -x + y + w;
	tempFR =  x + y - w;
	tempBL =  x + y + w;
	tempBR = -x + y - w;

	float currentMax = fabs(tempFL);
	if (fabs(tempFR) > currentMax) currentMax = fabs(tempFR);
	if (fabs(tempBL) > currentMax) currentMax = fabs(tempBL);
	if (fabs(tempBR) > currentMax) currentMax = fabs(tempBR);

	float scale = (currentMax > 1.0f) ? (speedCap / currentMax) : speedCap;

	if (x == 0.0 && y == 0.0 && fabs(ps4.joyR_x) <= deadzone && fabs(w) < 0.05) {
			FL = FR = BL = BR = 0;
	} else {
		FL = tempFL * scale;
		FR = tempFR * scale;
		BL = tempBL * scale;
		BR = tempBR * scale;
	}
}

void perspectiveControlProcess(float* x, float* y) {
	float referenceAngle = currentHeading - referenceHeading;
	if (referenceAngle > 180) referenceAngle -= 360;
	if (referenceAngle < -180) referenceAngle += 360;

	referenceAngle = referenceAngle * (M_PI / 180.0f);

	float rot_x = (*x * cosf(referenceAngle)) + (*y * sinf(referenceAngle));
	float rot_y = (*x * -1 * sinf(referenceAngle)) + (*y * cosf(referenceAngle));

	*x = rot_x;
	*y = rot_y;
}

//TASK DEFINITIONS
void HeartBeat(void *argument) {
	for (;;) {
		led2 = !led2;
		osDelay(250);
	}
}

void MovementTask(void *argument) {
//still haven't fully incorporated any of the movement mechanics
}

void servoTask(void *argument) {
	for (;;) {
		float stick = ps4.joyL_x;
		float ang = 500 + (stick + 1)*1000;
		if (ps4.button & R1) {
			servoControlActive = 1;
			ServoSetPulse(&serv1, ang);
		} else if (ps4.button & L1) {
			servoControlActive = 1;
			ServoSetPulse(&serv2, ang);
		} else if (ps4.an_R2 > 0) {
			servoControlActive = 1;
			ServoSetPulse(&serv3, ang);
		} else if (ps4.an_L2 > 0) {
			servoControlActive = 1;
			ServoSetPulse(&serv4, ang);
		} else {
			servoControlActive = 0;
		}
		osDelay(20);
	}
}

void pneumaticsTask(void *arguement) {
	for (;;) {
		if ((ps4.button & CIRCLE) && !pneumaticsToggle) {
			pneumaticsToggle = 1;
//			HAL_GPIO_TogglePin(IP8_PIN);
		} else if (!(ps4.button & CIRCLE) && pneumaticsToggle) {
			pneumaticsToggle = 0;
		}
		osDelay(20);
	}
}

int main(void)
{
	set();
	botInit();
	osKernelInitialize();

	//THREAD CREATION
	heartBeatHandle = osThreadNew(HeartBeat, NULL, &heartBeatTaskAttributes);
	movementHandle = osThreadNew(MovementTask, NULL, &movementTaskAttributes);
	servoHandle = osThreadNew(servoTask, NULL, &servoTaskAttributes);
	pneumaticsHandle = osThreadNew(pneumaticsTask, NULL, &pneumaticsTaskAttributes);

	osKernelStart();

	while(1);

}

void TIM6_DAC_IRQHandler(void)
{
	led1=!led1;
	HAL_TIM_IRQHandler(&htim6);
}


/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler(void)
{


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

