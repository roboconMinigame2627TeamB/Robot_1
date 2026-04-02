

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

//KFS VARIABLES
#define IRSENSOR1 IP10_PIN //to be changed later (active low)
#define IRSENSOR2 IP11_PIN //to be changed later (active low)
#define MOTORTOPLAYER BDC1 //to be changed later
#define MOTORBOTLAYER BDC2 //to be changed later
uint8_t topMotToggle = 0;
uint8_t botMotToggle = 0;
uint8_t bothMotToggle = 0;
uint8_t topMotKilled = 1;
uint8_t botMotKilled = 1;
uint8_t depositToggle = 0;
uint8_t depositMode = 0;
volatile int mot1pwm = 0; //top belt motor
volatile int mot2pwm = 0; //bot belt motor

//SERVO ARMS VARIABLES
SERVO_t serv1;
SERVO_t serv2;
uint8_t servoArmsMotorToggle = 0;
uint8_t armMotorsON = 0;
int servoPos = 1500;
volatile int mot3pwm = 0;
volatile int mot4pwm = 0;

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

//motor pid tuning
volatile float aw = 0.0, bw = 0.0, cw = 0.0, dw = 0.0;
//volatile float kp = 0.0, ki = 0.0, kd = 0.0;
volatile int mode = 2;
volatile int tune_finish = 0;

char buffer[64];
char rx_buf[64];
volatile uint8_t rx_idx = 0;
volatile uint8_t cmd_ready = 0;
uint8_t uart5_rx;
////

void process_command() {
    if (!cmd_ready) return;

    char cmd = rx_buf[0];
    float val = atof((char*)&rx_buf[1]); // Convert everything after the first char to float

    switch(cmd) {
        case 'v': case 'V':
        	aw = val;
        	bw = val;
        	cw = val;
        	dw = val;
            RNSVelocity(aw, bw, cw, dw, &rns);
            break;
        case 'p': case 'P':
            kp = val;


            RNSSet(&rns, RNS_B_RIGHT_VEL_PID, kp, ki, kd);

            break;
        case 'i': case 'I':
            ki = val;


            RNSSet(&rns, RNS_B_RIGHT_VEL_PID, kp, ki, kd);

            break;
        case 'd': case 'D':
            kd = val;


            RNSSet(&rns, RNS_B_RIGHT_VEL_PID, kp, ki, kd);

            break;
        case 'n': case 'N': // Send 'n' to advance to the Next motor
            mode++;
            aw = 0; bw = 0; cw = 0; dw = 0; // Stop previous motor
            RNSVelocity(aw, bw, cw, dw, &rns);

            if(mode > 3) {
                mode = 0;
                tune_finish = 1;
            }
            break;
    }
    rx_idx = 0;
    cmd_ready = 0;
}

float actual_V = 0.0;
float target_V = 0.0;

void tune_motor() {
    uint32_t last_call = 0;
    tune_finish = 0;
    mode = 0;
    HAL_UART_Receive_IT(&huart5, &uart5_rx, 1);

    aw = 0; bw = 0; cw = 0; dw = 0;
    RNSVelocity(0, 0, 0, 0, &rns);
    while(!tune_finish) {
        process_command();
        uint32_t now = HAL_GetTick();
        if (now - last_call >= 20) {

        	RNSEnquire(RNS_VEL_BOTH, &rns);

            if (mode == 0)      { actual_V = rns.RNS_data.common_buffer[0].data; target_V = aw; }
            else if (mode == 1) { actual_V = rns.RNS_data.common_buffer[1].data; target_V = bw; }
            else if (mode == 2) { actual_V = rns.RNS_data.common_buffer[2].data; target_V = cw; }
            else if (mode == 3) { actual_V = rns.RNS_data.common_buffer[3].data; target_V = dw; }

//            sprintf(buffer, "%.2f,%.2f\r\n", actual_V, target_V);
            sprintf(buffer, "%.2f,%.2f,%.2f,%.2f\r\n", rns.RNS_data.common_buffer[0].data,rns.RNS_data.common_buffer[1].data,rns.RNS_data.common_buffer[2].data,rns.RNS_data.common_buffer[3].data);
            UARTPrintString(&huart5, buffer);

            last_call = now;
        }
    }
    RNSStop(&rns);
}

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
		WriteBDC(&BDC2, mot2pwm);
		WriteBDC(&BDC3, mot3pwm);
		WriteBDC(&BDC4, mot4pwm);
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

		if ((ps4.button & TRIANGLE) && !bothMotToggle) {
			bothMotToggle = 1;
			if (mot1pwm == 20000 || mot2pwm == 20000) {
				topMotKilled = 1;
				botMotKilled = 1;
				mot1pwm = 0;
				mot2pwm = 0;
			}
			else {
				topMotKilled = 0;
				botMotKilled = 0;
				mot1pwm = 20000;
				mot2pwm = 20000;
			}
		} else if (!(ps4.button & TRIANGLE) && bothMotToggle) bothMotToggle = 0;

		//KFS TOP MOTOR
		if ((ps4.button & UP) && !topMotToggle) {
			topMotToggle = 1;
			if (mot1pwm == 20000) {
				topMotKilled = 1;
				mot1pwm = 0;
			}
			else {
				topMotKilled = 0;
				mot1pwm = 20000;
			}
		} else if (!(ps4.button & UP) && topMotToggle) topMotToggle = 0;

		//KFS BOT MOTOR
		if ((ps4.button & DOWN) && !botMotToggle) {
			botMotToggle = 1;
			if (mot2pwm == 20000) {
				botMotKilled = 1;
				mot2pwm = 0;
			}
			else {
				botMotKilled = 0;
				mot2pwm = 20000;
			}
		} else if (!(ps4.button & DOWN) && botMotToggle) botMotToggle = 0;

		//TOGGLEING DEPOSIT MODE (KFS MOTOR OVERRIDE)
		if ((ps4.button & RIGHT) && !depositToggle) {
			depositToggle = 1;
			depositMode = !depositMode;
		} else if (!(ps4.button & RIGHT) && depositToggle) depositToggle = 0;

		if (!HAL_GPIO_ReadPin(IRSENSOR1) && !depositMode) {
			topMotKilled = 1;
			mot1pwm = 0;
		}
		if (!HAL_GPIO_ReadPin(IRSENSOR2) && !depositMode && topMotKilled) {
			botMotKilled = 1;
			mot2pwm = 0;
		}
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
		const int servoMax = 1500;
		const int servoMin = 500;
		const int sweepSpeed = 15;

		if (ps4.button & R1) {
			servoPos += sweepSpeed;
			if (servoPos > servoMax) servoPos = servoMax;
		} else if (ps4.button & L1) {
			servoPos -= sweepSpeed;
			if (servoPos < servoMin) servoPos = servoMin;
		}

		ServoSetPulse(&serv1, servoPos);
		ServoSetPulse(&serv2, servoPos);

		if ((ps4.button & LEFT) && !servoArmsMotorToggle) {
			servoArmsMotorToggle = 1;
			armMotorsON = !armMotorsON;
			if (!armMotorsON) {
				mot3pwm = 500;
				mot4pwm = 500;
			} else {
				mot3pwm = 0;
				mot4pwm = 0;
			}
		} else if (!(ps4.button & LEFT) && servoArmsMotorToggle) servoArmsMotorToggle = 0;
		osDelay(20);
	}
}

int main(void)
{
	set();
//	tune_motor();
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

// Place this in your main.c (or wherever your user callbacks are defined)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // Check if the interrupt was triggered by UART5
    if (huart->Instance == UART5) {

        // Look for end-of-line characters (Enter key: Carriage Return or Line Feed)
        if (uart5_rx == '\r' || uart5_rx == '\n') {

            // Make sure we actually received data (prevents double-triggering on \r\n)
            if (rx_idx > 0) {
                rx_buf[rx_idx] = '\0'; // Null-terminate the buffer so atof() works correctly
                cmd_ready = 1;         // Signal process_command() to run
            }

        } else {
            // Append the new character to the buffer
            // We check against 63 to leave room for the '\0' terminator (buffer is 64)
            if (rx_idx < 63) {
                rx_buf[rx_idx] = uart5_rx;
                rx_idx++;
            }
        }

        // Re-arm the interrupt to listen for the next single byte
        HAL_UART_Receive_IT(&huart5, &uart5_rx, 1);
    }


}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
// If the UART crashes due to an Overrun Error (or any error),
// force it to clear the error and restart the receive interrupt.
if (huart->Instance == UART5) {
    // Clear the Overrun error flag (syntax might vary slightly based on STM32F4 family)
    __HAL_UART_CLEAR_OREFLAG(huart);

    // Restart the interrupt
    HAL_UART_Receive_IT(&huart5, &uart5_rx, 1);
}
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

