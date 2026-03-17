/************************************************
 * Title   : LSA
 * Author  : Kai Sheng and Qiu Hui
 * Version : 1.20
 * Date    : 13 JULY 2017
 * **********************************************
 * Descriptions:
 *
 *
 * Version History:
 *
 * Bugs:
 *
 ************************************************/

#ifndef LSA_H_
#define LSA_H_

#include"../BIOS/bios.h"

/*LSA_COMMAND*/
#define Calibration							0x43	/*LSA_DATA: 0*/
#define Mode								0x4C	/*LSA_DATA: 0-1
										 	 	 	 0:LIGHT_ON 1:DARK_ON*/
#define Line_Threshold						0x54	/*LSA_DATA: 0-7*/
#define Junction_Width						0x4A	/*LSA_DATA: 3-8*/
#define UART_Address						0x41	/*LSA_DATA: 0-255*/
#define LCD_Backlight						0x42	/*LSA_DATA: 0-10*/
#define LCD_Contrast						0x53	/*LSA_DATA: 0-255*/
#define UART_Baudrate						0x52	/*LSA_DATA: 0-5
 	 	 	 	 	 	 	 	 	 	 	 	 	 0: 9600bps	1: 19200bps	2: 38400bps 3: 57600bps
 	 	 	 	 	 	 	 	 	 	 	 	 	 4: 115200bps 5: 230400bps*/
#define UART_Data_Output_Mode 				0x44	/*LSA_DATA: 0-3*/
#define Junction_Information				0x50	/*LSA_DATA: 0-1*/
#define Line_position						0x4F	/*LSA_DATA: 1-2*/
#define sensor_information					0x4F

typedef struct {

}LSA1_t;

typedef struct{

	union{
		uint8_t data;
		struct{
			uint8_t bit0:1;
			uint8_t bit1:1;
			uint8_t bit2:1;
			uint8_t bit3:1;
			uint8_t bit4:1;
			uint8_t bit5:1;
			uint8_t bit6:1;
			uint8_t bit7:1;
		};
	};

	uint8_t raw[8];
	int count;
	uint8_t LSAtmpbuff;

	GPIO_TypeDef * S1_GPIOx; uint16_t S1_pin;
	GPIO_TypeDef * S2_GPIOx; uint16_t S2_pin;
	GPIO_TypeDef * S3_GPIOx; uint16_t S3_pin;
	GPIO_TypeDef * S4_GPIOx; uint16_t S4_pin;
	GPIO_TypeDef * S5_GPIOx; uint16_t S5_pin;
	GPIO_TypeDef * S6_GPIOx; uint16_t S6_pin;
	GPIO_TypeDef * S7_GPIOx; uint16_t S7_pin;
	GPIO_TypeDef * S8_GPIOx; uint16_t S8_pin;

	uint8_t S1;
	uint8_t S2;
	uint8_t S3;
	uint8_t S4;
	uint8_t S5;
	uint8_t S6;
	uint8_t S7;
	uint8_t S8;

	uint8_t LSA_T;
	byte_t LSA_bits;
	float PosErr;
	uint8_t PosMemory;


}LSA_t;

enum{
	uart_mode_1,
	uart_mode_2,
	uart_mode_3
};

void LSAInit(	LSA_t* lsa,
				GPIO_TypeDef * S1GPIOx, uint16_t S1GPIOPin,
				GPIO_TypeDef * S2GPIOx, uint16_t S2GPIOPin,
				GPIO_TypeDef * S3GPIOx, uint16_t S3GPIOPin,
				GPIO_TypeDef * S4GPIOx, uint16_t S4GPIOPin,
				GPIO_TypeDef * S5GPIOx, uint16_t S5GPIOPin,
				GPIO_TypeDef * S6GPIOx, uint16_t S6GPIOPin,
				GPIO_TypeDef * S7GPIOx, uint16_t S7GPIOPin,
				GPIO_TypeDef * S8GPIOx, uint16_t S8GPIOPin);
void LSAErr_Handler(LSA_t* lsa);
void LSA_read(LSA_t* lsa);
void LSAUartDataHandle(LSA_t *LSA, uint8_t mode, UART_HandleTypeDef* huartx);
void LSACmd(uint8_t LSA_ADDRESS, uint8_t LSA_COMMAND, uint8_t LSA_DATA, UART_HandleTypeDef* huartx);

#endif /* LSA_H_ */
