/*******************************************************************************
 * Title   : adapter.h
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 12/10
 *******************************************************************************
 * Description: includes all the important includes and pin definitions
 *
 * Version History:
 *  1.0 - converted to HAL library
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef SRC_ADAPTER_H_
#define SRC_ADAPTER_H_

/////////////#define mainboard3_3

/* Private variables ---------------------------------------------------------*/


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "BIOS/bios.h"
#include <math.h>
#include "CAN/can.h"
#include "MODN/MODN.h"
#include "PID/PID.h"
#include "ABT/ABT.h"
#include "I2C/i2c.h"
#include "SPI/SPI.h"
#include "PSx_Interface/PSx_Interface.h"
#include "RNS_interface/RNS_interface.h"
#include "ADC/adc.h"
#include "SERVO/servo.h"
#include "KF/KF.h"
#include "LASER/laser.h"
#include "STEPPER/stepper.h"
#include "SERVO_DRIVER/servo_driver.h"
#include "Moving_Average/mov_ave.h"
#include "Eeprom/eeprom.h"
#include "r6091u.h"


/* Exported functions prototypes ---------------------------------------------*/


/* Private defines -----------------------------------------------------------*/
//#define FREERTOS


#ifdef mainboard3_3
//#define LED1_PIN			GPIOC, GPIO_PIN_13
//#define LED2_PIN			GPIOC, GPIO_PIN_14
//#define LED3_PIN			GPIOC, GPIO_PIN_15
//#define LED4_PIN			GPIOD, GPIO_PIN_7
//#define LED5_PIN			GPIOA, GPIO_PIN_8
//#define LED6_PIN			GPIOC, GPIO_PIN_9
//#define LED7_PIN			GPIOB, GPIO_PIN_6
//#define LED8_PIN			GPIOB, GPIO_PIN_7

#define PB1_PIN				GPIOB, GPIO_PIN_5			// TIM4_CH2
#define PB2_PIN				GPIOE, GPIO_PIN_0
#define PB3_PIN				GPIOE, GPIO_PIN_4

#define IP1_PIN				GPIOE, GPIO_PIN_12          // TIM1_CH3N
#define IP2_PIN				GPIOE, GPIO_PIN_13          // TIM1_CH3
#define IP3_PIN				GPIOE, GPIO_PIN_14          // TIM1_CH4
#define IP4_PIN				GPIOE, GPIO_PIN_15          // TIM1_BKN
#define IP5_PIN				GPIOB, GPIO_PIN_14          // TIM12_CH1
#define IP6_PIN				GPIOB, GPIO_PIN_15          // TIM12_CH2
#define IP7_PIN				GPIOD, GPIO_PIN_10
#define IP8_PIN				GPIOD, GPIO_PIN_11
#define IP9_PIN				GPIOC, GPIO_PIN_8           // TIM8_CH3, TIM3_CH3
#define IP10_PIN			GPIOA, GPIO_PIN_10          // TIM1_CH3
#define IP11_PIN			GPIOD, GPIO_PIN_3
#define IP12_PIN			GPIOD, GPIO_PIN_4
#else
#define LED1_PIN			GPIOC, GPIO_PIN_13
#define LED2_PIN			GPIOC, GPIO_PIN_14
#define LED3_PIN			GPIOC, GPIO_PIN_15

#define PB1_PIN				GPIOB, GPIO_PIN_7			// TIM4_CH2
#define PB2_PIN				GPIOE, GPIO_PIN_0

#define IP1_PIN				GPIOE, GPIO_PIN_12
#define IP2_PIN				GPIOE, GPIO_PIN_13
#define IP3_PIN				GPIOE, GPIO_PIN_14
#define IP4_PIN				GPIOE, GPIO_PIN_15
#define IP5_PIN				GPIOB, GPIO_PIN_14
#define IP6_PIN				GPIOB, GPIO_PIN_15
#define IP7_PIN				GPIOD, GPIO_PIN_10
#define IP8_PIN				GPIOD, GPIO_PIN_11
#define IP9_PIN				GPIOC, GPIO_PIN_8
#define IP10_PIN			GPIOA, GPIO_PIN_10
#define IP11_PIN			GPIOD, GPIO_PIN_3
#define IP12_PIN			GPIOD, GPIO_PIN_4
#define IP13_PIN			GPIOD, GPIO_PIN_7
#define IP14_PIN			GPIOB, GPIO_PIN_5
#define IP15_PIN			GPIOB, GPIO_PIN_6
#endif

#define IP16_Analog1_PIN	GPIOC, GPIO_PIN_0
#define IP17_Analog2_PIN	GPIOC, GPIO_PIN_1
#define IP18_Analog3_PIN	GPIOC, GPIO_PIN_2
#define IP19_Analog4_PIN	GPIOC, GPIO_PIN_3
#define IP20_Analog5_PIN	GPIOC, GPIO_PIN_4
#define IP21_Analog6_PIN	GPIOC, GPIO_PIN_5

#define QEI1_PLUSEA_PIN		GPIOE , GPIO_PIN_9
#define QEI1_PLUSEB_PIN		GPIOE , GPIO_PIN_11

#define QEI2_PLUSEA_PIN		GPIOA , GPIO_PIN_5
#define QEI2_PLUSEB_PIN		GPIOB , GPIO_PIN_3

#define QEI3_PLUSEA_PIN		GPIOA , GPIO_PIN_6
#define QEI3_PLUSEB_PIN		GPIOA , GPIO_PIN_7

#define QEI4_PLUSEA_PIN		GPIOD , GPIO_PIN_12
#define QEI4_PLUSEB_PIN		GPIOD , GPIO_PIN_13

#define QEI5_PLUSEA_PIN		GPIOA , GPIO_PIN_0
#define QEI5_PLUSEB_PIN		GPIOA , GPIO_PIN_1

#define QEI6_PLUSEA_PIN		GPIOC , GPIO_PIN_6
#define QEI6_PLUSEB_PIN		GPIOC , GPIO_PIN_7

#define TIM5_CHANNEL1_PIN	GPIOA, GPIO_PIN_0
#define TIM5_CHANNEL2_PIN	GPIOA, GPIO_PIN_1
#define TIM5_CHANNEL3_PIN	GPIOA, GPIO_PIN_2
#define TIM5_CHANNEL4_PIN	GPIOA, GPIO_PIN_3

#define TIM9_CHANNEL1_PIN	GPIOE, GPIO_PIN_5
#define TIM9_CHANNEL2_PIN	GPIOE, GPIO_PIN_6

#define TIM3_ChANNEL1_PIN   GPIOA, GPIO_PIN_6
#define TIM3_CHANNEL3_PIN	GPIOB, GPIO_PIN_0
#define TIM3_CHANNEL4_PIN	GPIOB, GPIO_PIN_1

#define TIM1_CHANNEL3_PIN	GPIOE, GPIO_PIN_13
#define TIM1_CHANNEL4_PIN	GPIOE, GPIO_PIN_14

#define MUX1_INPUT_PIN 		GPIOE , GPIO_PIN_1
#define MUX1_S0_PIN 		GPIOE , GPIO_PIN_2
#define MUX1_S1_PIN 		GPIOE , GPIO_PIN_3
#define MUX1_S2_PIN 		GPIOE , GPIO_PIN_4

#define SR_SCK_PIN			GPIOE , GPIO_PIN_7
#define SR_RCK_PIN			GPIOE , GPIO_PIN_8
#define SR_SI_PIN			GPIOE , GPIO_PIN_10

#define SPI1_NSS_PIN		GPIOA, GPIO_PIN_4
#define SPI1_SCK_PIN		GPIOA, GPIO_PIN_5
#define SPI1_MISO_PIN		GPIOA, GPIO_PIN_6
#define SPI1_MOSI_PIN		GPIOA, GPIO_PIN_7

#define UART2_Tx			GPIOD , GPIO_PIN_5
#define UART2_Rx			GPIOD , GPIO_PIN_6

#define UART3_Tx			GPIOD , GPIO_PIN_9
#define UART3_Rx			GPIOD , GPIO_PIN_8

#define UART4_Tx			GPIOC , GPIO_PIN_10
#define UART4_Rx			GPIOC , GPIO_PIN_11

#define UART5_Tx			GPIOC , GPIO_PIN_12
#define UART5_Rx			GPIOD , GPIO_PIN_2

#define CAN1_Tx				GPIOD , GPIO_PIN_1
#define CAN1_Rx				GPIOD , GPIO_PIN_0

#define CAN2_Tx				GPIOB , GPIO_PIN_13
#define CAN2_Rx				GPIOB , GPIO_PIN_12


MUX_t MUX;
shiftreg_t SR;
RNS_interface_t rns;
BDC_t BDC1, BDC2, BDC3, BDC4, BDC5, BDC6, BDC7, BDC8;
uint8_t insData_receive[2];
PSxBT_t ps4;
MODN_t Modn;
ABT_t filter;
ADC_t adc;
LASER_t r_laser,l_laser;
KALMANFILTER_t kf_adc_r,kf_adc_l,kf_pres;
PID_t pid_laser_R,pid_laser_L,pid_pres,pid_z;
Srv_Drv_t srv_drv;
Mov_Ave_t mov_l_r,mov_l_l;
STP_t step1;
STP_t step2;
R6091U_t IMU;


#ifdef mainboard3_3
#define PB1			GPIOB_IN->bit5
#define PB2			GPIOE_IN->bit0
#define PB3			GPIOE_IN->bit4

#define LED1_PIN			GPIOB, GPIO_PIN_7
#define LED2_PIN			GPIOC, GPIO_PIN_15
#define LED3_PIN			GPIOA, GPIO_PIN_8
#define LED4_PIN			GPIOC, GPIO_PIN_9
#define LED5_PIN			GPIOC, GPIO_PIN_14
#define LED6_PIN			GPIOC, GPIO_PIN_13
#define LED7_PIN			GPIOD, GPIO_PIN_7
#define LED8_PIN			GPIOB, GPIO_PIN_6

#define led1		GPIOB_OUT->bit7  //12
#define led2		GPIOC_OUT->bit15 //6
#define led3		GPIOA_OUT->bit8  //9
#define led4		GPIOC_OUT->bit9  //10
#define led5		GPIOC_OUT->bit14 //5
#define led6		GPIOC_OUT->bit13 //4
#define led7		GPIOD_OUT->bit7  //8
#define led8		GPIOB_OUT->bit6  //11
#else

#define PB1 		GPIOB_IN->bit7
#define PB2 		GPIOE_IN->bit0

#define led1		GPIOC_OUT->bit13
#define led2		GPIOC_OUT->bit14
#define led3		GPIOC_OUT->bit15
#endif

//Global Declarations
float v1, v2, v3, v4, wr, xr, yr, orientation;    //MODN variables
float xpos, ypos, z;                              //Encoder Values
float a, b, c, d, pa, pb, pc, pd;  				  //Enquiry Values
volatile uint16_t  adc1_buf[7];
volatile uint16_t  adc2_buf[7];
volatile uint16_t  adc3_buf[7];

int counter;                                      //global timer variable
float speed;                                      //for manual control

union{
	float data;
	struct{
		char byte1;
		char byte2;
		char byte3;
		char byte4;
	};
}buf1_receive[2];

union{
	float data;
	struct{
		char byte1;
		char byte2;
		char byte3;
		char byte4;
	};
}buf2_receive[2];

typedef enum{
	RNS_PACKET,
	VESC_PACKET,
}PACKET_t;

void CAN_PROCESS(PACKET_t packet_src);
void Initialize (void);

#ifdef __cplusplus
}
#endif
#endif /* SRC_ADAPTER_H_ */
