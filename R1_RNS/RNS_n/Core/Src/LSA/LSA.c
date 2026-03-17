#include"LSA.h"



void LSAInit(	LSA_t* lsa,
		GPIO_TypeDef * S1GPIOx, uint16_t S1GPIOPin,
		GPIO_TypeDef * S2GPIOx, uint16_t S2GPIOPin,
		GPIO_TypeDef * S3GPIOx, uint16_t S3GPIOPin,
		GPIO_TypeDef * S4GPIOx, uint16_t S4GPIOPin,
		GPIO_TypeDef * S5GPIOx, uint16_t S5GPIOPin,
		GPIO_TypeDef * S6GPIOx, uint16_t S6GPIOPin,
		GPIO_TypeDef * S7GPIOx, uint16_t S7GPIOPin,
		GPIO_TypeDef * S8GPIOx, uint16_t S8GPIOPin		)
{



	GPIOPinsInit (S1GPIOx, S1GPIOPin, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLDOWN);
	GPIOPinsInit (S2GPIOx, S2GPIOPin, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLDOWN);
	GPIOPinsInit (S3GPIOx, S3GPIOPin, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLDOWN);
	GPIOPinsInit (S4GPIOx, S4GPIOPin, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLDOWN);
	GPIOPinsInit (S5GPIOx, S5GPIOPin, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLDOWN);
	GPIOPinsInit (S6GPIOx, S6GPIOPin, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLDOWN);
	GPIOPinsInit (S7GPIOx, S7GPIOPin, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLDOWN);
	GPIOPinsInit (S8GPIOx, S8GPIOPin, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLDOWN);


	lsa->S1_GPIOx 	= S1GPIOx;
	lsa->S1_pin 	= S1GPIOPin;
	lsa->S2_GPIOx 	= S2GPIOx;
	lsa->S2_pin 	= S2GPIOPin;
	lsa->S3_GPIOx 	= S3GPIOx;
	lsa->S3_pin 	= S3GPIOPin;
	lsa->S4_GPIOx 	= S4GPIOx;
	lsa->S4_pin 	= S4GPIOPin;
	lsa->S5_GPIOx 	= S5GPIOx;
	lsa->S5_pin 	= S5GPIOPin;
	lsa->S6_GPIOx 	= S6GPIOx;
	lsa->S6_pin 	= S6GPIOPin;
	lsa->S7_GPIOx 	= S7GPIOx;
	lsa->S7_pin 	= S7GPIOPin;
	lsa->S8_GPIOx 	= S8GPIOx;
	lsa->S8_pin 	= S8GPIOPin;

}
void LSA_read(LSA_t* lsa){
	lsa->LSA_bits.bit0 = lsa->S1 = HAL_GPIO_ReadPin(lsa->S1_GPIOx, lsa->S1_pin);
	lsa->LSA_bits.bit1 = lsa->S2 = HAL_GPIO_ReadPin(lsa->S2_GPIOx, lsa->S2_pin);
	lsa->LSA_bits.bit2 = lsa->S3 = HAL_GPIO_ReadPin(lsa->S3_GPIOx, lsa->S3_pin);
	lsa->LSA_bits.bit3 = lsa->S4 = HAL_GPIO_ReadPin(lsa->S4_GPIOx, lsa->S4_pin);
	lsa->LSA_bits.bit4 = lsa->S5 = HAL_GPIO_ReadPin(lsa->S5_GPIOx, lsa->S5_pin);
	lsa->LSA_bits.bit5 = lsa->S6 = HAL_GPIO_ReadPin(lsa->S6_GPIOx, lsa->S6_pin);
	lsa->LSA_bits.bit6 = lsa->S7 = HAL_GPIO_ReadPin(lsa->S7_GPIOx, lsa->S7_pin);
	lsa->LSA_bits.bit7 = lsa->S8 = HAL_GPIO_ReadPin(lsa->S8_GPIOx, lsa->S8_pin);
	lsa->LSA_T = lsa->S1 + lsa->S2 + lsa->S3 + lsa->S4 +lsa->S5 +lsa->S6 + lsa->S7 + lsa->S8;

}
void LSAErr_Handler(LSA_t* lsa){
	lsa->S1 = HAL_GPIO_ReadPin(lsa->S1_GPIOx, lsa->S1_pin);
	lsa->S2 = HAL_GPIO_ReadPin(lsa->S2_GPIOx, lsa->S2_pin);
	lsa->S3 = HAL_GPIO_ReadPin(lsa->S3_GPIOx, lsa->S3_pin);
	lsa->S4 = HAL_GPIO_ReadPin(lsa->S4_GPIOx, lsa->S4_pin);
	lsa->S5 = HAL_GPIO_ReadPin(lsa->S5_GPIOx, lsa->S5_pin);
	lsa->S6 = HAL_GPIO_ReadPin(lsa->S6_GPIOx, lsa->S6_pin);
	lsa->S7 = HAL_GPIO_ReadPin(lsa->S7_GPIOx, lsa->S7_pin);
	lsa->S8 = HAL_GPIO_ReadPin(lsa->S8_GPIOx, lsa->S8_pin);

	lsa->LSA_T = lsa->S1 + lsa->S2 + lsa->S3 + lsa->S4 +lsa->S5 +lsa->S6 + lsa->S7 + lsa->S8;

	if(lsa->LSA_T > 0){

		lsa->PosErr = (lsa->S1 * 1.0 + lsa->S2 * 2.0 + lsa->S3 * 3.0 + lsa->S4 * 4.0 + lsa->S5 * 5.0 +
				lsa->S6 * 6.0 + lsa->S7 * 7.0 + lsa->S8 * 8.0 )/(lsa->LSA_T) - 4.5;
		(lsa->PosErr < 0.0)? (lsa->PosMemory = 0) : (lsa->PosMemory = 1);

	}else

		(lsa->PosMemory == 0)? (lsa->PosErr = -5) : (lsa->PosErr = 5);

}
void LSAUartDataHandle(LSA_t *LSA, uint8_t mode, UART_HandleTypeDef* huartx){

	if ( mode == uart_mode_1 || mode == uart_mode_2 ) {
		LSA->data = ReadUART(huartx);
	}else if (mode == uart_mode_3) {
		LSA->LSAtmpbuff = ReadUART(huartx);
		if ( LSA->LSAtmpbuff == 0x00 ) {
			LSA->count =0;
		} else {
			LSA->raw[LSA->count++] = LSA->LSAtmpbuff;
		}
	}
}

void LSACmd(uint8_t LSA_ADDRESS, uint8_t LSA_COMMAND, uint8_t LSA_DATA, UART_HandleTypeDef* huartx){

	UARTPrintString(huartx, &LSA_ADDRESS);
	UARTPrintString(huartx, &LSA_COMMAND);
	UARTPrintString(huartx, &LSA_DATA);
	UARTPrintString(huartx, (LSA_ADDRESS + LSA_COMMAND+ LSA_DATA));

}
