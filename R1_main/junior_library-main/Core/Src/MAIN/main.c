

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */

//HANDLES
osThreadId_t heartBeatHandle;
osThreadId_t controlHandle;
osThreadId_t functionalHandle;
osThreadId_t IMUHandle;
osThreadId_t servoArmsHandle;

//SEMAPHORES & MUTEXES
osSemaphoreId_t IMUSempahore;
osMutexId_t IMUDataMutex;

//ATTRIBUTE DEFINITIONS
const osThreadAttr_t heartBeatTaskAttributes = {
		.name = "heartBeatTask",
		.stack_size = 512,
		.priority = (osPriority_t) osPriorityIdle,
};

const osThreadAttr_t controlTaskAttributes = {
		.name = "controlTask",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t functionalTaskAttributes = {
		.name = "functionalTask",
		.stack_size = 1024,
		.priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t IMUTaskAttributes = {
		.name = "IMUTask",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t servoArmsTaskAttributes = {
		.name = "servoArmsTask",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityAboveNormal,
};

const osSemaphoreAttr_t IMUSemAttributes = {
		.name = "IMUSem"
};

const osMutexAttr_t IMUDataSemAttributes = {
		.name = "IMUDataMux"
};

//GLOBAL VARIABLES

///COMMON MOVEMENT & ANGLE VARIABLES
float FL, FR, BL, BR;
float speedCap;
volatile float currentHeading = 0, targetHeading = 0;
float deadzone = 0.1;

///SERVO VARIABLES
volatile uint8_t servoControlActive = 0;

///PNEUMATICS VARIABLES
#define PNEUMATICVALVE1 IP1_PIN //to be changed later, also make sure to reinitialize them as output pins
#define PNEUMATICVALVE2 IP5_PIN //to be changed later
uint8_t pneumaticsToggle = 0;

///PERSPECTIVE CONTROL VARIABLES
volatile float referenceHeading = 0.0;
uint8_t perspectiveControlMode = 1;
uint8_t perspectiveControlToggle = 0;

//IMU LOCK VARIABLES
PID_t IMULock;
float wPID = 0.0, error = 0.0;
float kp = 1.0, ki = 0.0, kd = 0.0;
uint8_t IMULockMode = 1;
uint8_t IMULockToggle = 0;
volatile uint8_t IMUDataReady = 0;

//SERVO ARMS VARIABLES
SERVO_t serv1;
SERVO_t serv2;
uint8_t servoArmsToggle = 0;
uint8_t servoArmsClosed = 0;
uint8_t servoArmsMotorToggle = 0;
uint8_t armMotorsON = 0;

//KFS VARIABLES
#define IRSENSOR1 IP10_PIN //to be changed later (active low)
#define IRSENSOR2 IP11_PIN //to be changed later (active low)
#define MOTORTOPLAYER BDC1 //to be changed later
#define MOTORBOTLAYER BDC2 //to be changed later
uint8_t topMotToggle = 0;
uint8_t botMotToggle = 0;
uint8_t topMotKilled = 1;
uint8_t botMotKilled = 1;
uint8_t depositToggle = 0;
uint8_t depositMode = 0;
volatile int mot1pwm = 0;
volatile int mot2pwm = 0;


//FUNCTION PROTOTYPES
void botInit(void);
void analogueMovement(void);
void perspectiveControlProcess(float* x, float* y);
void IMUPIDProcessing(void);
void IMULockProcessing(float* w, float deadzone);

//TASK PROTOTYPES
void HeartBeat(void *argument);
void controlTask(void *argument);
void functionalTask(void *arguement);
void IMUTask(void *argument);
void servoArmsTask(void *argument);
void KFSTask(void *argument);

//FUNCTION DEFINITIONS
void botInit(void) {
	PSxSlaveInit(&ps4, &hi2c1);

	PIDSourceInit(&error, &wPID, &IMULock);
	PIDGainInit(0.02, 1.0, 1.0/180.0, 1.0, kp, ki, kd, 100.0, &IMULock);

	ServoxInit(&serv1, &htim1, TIM1_CHANNEL3_PIN, TIM_CHANNEL_3);
	ServoxInit(&serv2, &htim1, TIM1_CHANNEL4_PIN, TIM_CHANNEL_4);

	RNSEnquire(RNS_X_Y_IMU_LSA, &rns);
	targetHeading = rns.enq.enq_buffer[0].data;
	currentHeading = rns.enq.enq_buffer[0].data;
	referenceHeading = rns.enq.enq_buffer[0].data;
}

//Processes user data for movement
void analogueMovement(void) {
	float tempFL, tempFR, tempBL, tempBR;
	float y = ps4.joyL_y;
	float x = ps4.joyL_x;
	float w = 0.0;

	if (fabs(x) < deadzone) x = 0.0;
	if (fabs(y) < deadzone) y = 0.0;

	if (IMULockMode) IMULockProcessing(&w, deadzone);
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

void IMUPIDProcessing(void) {
	float tempError = 0;
	RNSEnquire(RNS_X_Y_IMU_LSA, &rns);
	currentHeading = rns.enq.enq_buffer[0].data;
	tempError = targetHeading - currentHeading;
	if (tempError > 180) tempError -= 360;
	if (tempError < -180) tempError += 360;
	error = tempError;
}

void IMULockProcessing(float* w, float deadzone) {
	if (fabs(ps4.joyR_x) < deadzone) {
		*w = wPID;
	} else {
		*w = ps4.joyR_x;
		targetHeading = currentHeading;
	}
}

//TASK DEFINITIONS
void HeartBeat(void *argument) {
	for (;;) {
		led2 = !led2;
		osDelay(250);
	}
}

void controlTask(void *argument) {
	for (;;) {
		uint8_t isMoving = 0;
		WriteBDC(&BDC1, mot1pwm);
		SHIFTREGShift(&SR);


		//EMERGANCY BUTTON
		if (ps4.button & PS) {
			RNSStop(&rns);
			NVIC_SystemReset();
		}

		//STANDARD MOVEMENT
		osMutexAcquire(IMUDataMutex, osWaitForever);
		if (fabs(ps4.joyL_x) > 0.15 || fabs(ps4.joyL_y) > 0.15 || fabs(ps4.joyR_x) > 0.15 || fabs(error) > 1.0) {
			if (servoControlActive) {
				RNSStop(&rns);
				osMutexRelease(IMUDataMutex);
				return;
			}
			analogueMovement();
			RNSVelocity(FL, FR, BL, BR, &rns);
			isMoving = 1;
		}
		osMutexRelease(IMUDataMutex);

		//STOP ALL MOVEMENT IF NO ACTIVITY
		if (!isMoving) RNSStop(&rns);

		//SLOW MODE
		if (ps4.button & CROSS) speedCap = 0.5;
		else speedCap = 1.0;

		//IMU LOCK
		if ((ps4.button & SHARE) && !IMULockToggle) {
			IMULockToggle = 1;
			IMULockMode = !IMULockMode;
		} else if (!(ps4.button & SHARE) && IMULockToggle) IMULockToggle = 0;

		//PERSPECTIVE CONTROL
		if ((ps4.button & OPTION) && !perspectiveControlToggle) {
			perspectiveControlToggle = 1;
			perspectiveControlMode = !perspectiveControlMode;
			referenceHeading = currentHeading;
		} else if (!(ps4.button & OPTION) && perspectiveControlToggle) perspectiveControlToggle = 0;

		osDelay(20);
	}
}

void functionalTask(void *arguement) {
	for (;;) {
		//PNEUMATICS
		if ((ps4.button & CIRCLE) && !pneumaticsToggle) {
			pneumaticsToggle = 1;
			HAL_GPIO_TogglePin(PNEUMATICVALVE1);
			HAL_GPIO_TogglePin(PNEUMATICVALVE2);
		} else if (!(ps4.button & CIRCLE) && pneumaticsToggle) pneumaticsToggle = 0;

		//KFS TOP MOTOR
		if ((ps4.button & UP) && !topMotToggle) {
			topMotToggle = 1;
			if (mot1pwm == 20000) mot1pwm = 0;
			else mot1pwm = 20000;
		} else if (!(ps4.button & UP) && topMotToggle) topMotToggle = 0;

		//KFS BOT MOTOR
		if ((ps4.button & DOWN) && !botMotToggle) {
			botMotToggle = 1;
			if (mot2pwm == 20000) mot2pwm = 0;
			else mot2pwm = 20000;
		} else if (!(ps4.button & DOWN) && botMotToggle) botMotToggle = 0;
		osDelay(20);
	}
}

void IMUTask(void *argument) {
	for (;;) {
		if (osSemaphoreAcquire(IMUSempahore, osWaitForever) == osOK) {
			osMutexAcquire(IMUDataMutex, osWaitForever);
			IMUPIDProcessing();
			PID(&IMULock);
			osMutexRelease(IMUDataMutex);
		}
	}
}

void servoArmsTask(void *argument) {
	for (;;) {
		if ((ps4.button & SQUARE) && !servoArmsToggle) {
			servoArmsToggle = 1;
			if (servoArmsClosed) {
				servoArmsClosed = 0;
				ServoSetPulse(&serv1, 500);
				ServoSetPulse(&serv2, 500);
			} else {
				servoArmsClosed = 1;
				ServoSetPulse(&serv1, 2000);
				ServoSetPulse(&serv2, 2000);
			}
		} else if (!(ps4.button & SQUARE) && servoArmsToggle) servoArmsToggle = 0;

		if ((ps4.button & LEFT) && !servoArmsMotorToggle) {
			servoArmsMotorToggle = 1;
			armMotorsON = !armMotorsON;
			if (!armMotorsON) {
				WriteBDC(&BDC3, 500);
				WriteBDC(&BDC4, 500);
			} else {
				StopBDC(&BDC3);
				StopBDC(&BDC4);
				SHIFTREGShift(&SR);
			}
		} else if (!(ps4.button & LEFT) && servoArmsMotorToggle) servoArmsMotorToggle = 0;
		osDelay(20);
	}
}

void KFSTask(void *argument) {
	for (;;) {
		if ((ps4.button & RIGHT) && !depositToggle) {
			depositToggle = 1;
			depositMode = !depositMode;
		} else if (!(ps4.button & RIGHT) && depositToggle) depositToggle= 0;

//		if (!HAL_GPIO_ReadPin(IRSENSOR1) && !depositMode) StopBDC(&MOTORTOPLAYER);
//		if (!HAL_GPIO_ReadPin(IRSENSOR2) && !depositMode && topMotKilled) StopBDC(&MOTORBOTLAYER);
		osDelay(20);
	}
}

int main(void)
{
	set();
	botInit();
	osKernelInitialize();

	//SEMAPHORES & MUTEX CREATION
	IMUSempahore = osSemaphoreNew(1, 0, &IMUSemAttributes);
	IMUDataMutex = osMutexNew(&IMUDataSemAttributes);

	//THREAD CREATION
	heartBeatHandle = osThreadNew(HeartBeat, NULL, &heartBeatTaskAttributes);
	controlHandle = osThreadNew(controlTask, NULL, &controlTaskAttributes);
	functionalHandle = osThreadNew(functionalTask, NULL, &functionalTaskAttributes);
	IMUHandle = osThreadNew(IMUTask, NULL, &IMUTaskAttributes);
	servoArmsHandle = osThreadNew(servoArmsTask, NULL, &servoArmsTaskAttributes);

	osKernelStart();

	while(1);

}

void TIM6_DAC_IRQHandler(void)
{
	led1=!led1;
	osSemaphoreRelease(IMUSempahore);

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

