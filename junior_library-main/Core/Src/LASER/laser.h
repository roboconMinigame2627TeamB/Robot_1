
/*******************************************************************************
 * Title   : Laser (
 * Author  : KLok
 * Version : 1.00
 * Date    : Sept 2020
 *******************************************************************************
 * Description:
 * - Combined with ADC and kalman_filter(KF) to fully used it
 * -
 *
 * Version History:
 * 1.00 by Klok
 * - Basic function of laser calibration
 *
 * 1.1 by Anas
 * - Added Check for distance and way to manually tune
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef SRC_LASER_LASER_H_
#define SRC_LASER_LASER_H_

#include "../BIOS/bios.h"

/**************************************************
 * 		Structure							  	  *
 *************************************************/

typedef struct{
	float min_value;
	float max_value;
	float min_distance;
	float max_distance;
	float *input;
	float *output;
	float kcd;
	uint16_t count;


}LASER_t;





/**************************************************
 * 		Enumerator							  	  *
 *************************************************/
//typedef enum {
//	QEI1 = 1,
//	QEI2 = 2,
//	QEI3 = 3,
//	QEI4 = 4,
//	QEI5 = 5,
//	QEI6 = 6
//}QEI_TypeDef;
//
//typedef enum{
//	QEI_No_Swap = 0,
//	QEI_Swap
//}QEI_Direction_TypeDef;

/**************************************************
 * 		Extern	variables					  	  *
 *************************************************/
//extern qei_TypeDef BIOS_QEI1;
//extern qei_TypeDef BIOS_QEI2;
//extern qei_TypeDef BIOS_QEI3;
//extern qei_TypeDef BIOS_QEI4;
//extern qei_TypeDef BIOS_QEI5;
//extern qei_TypeDef BIOS_QEI6;
//extern enc_TypeDef qei;

/**************************************************
 * 		Function Prototype			  			  *
 *************************************************/

void LaserInit (float min_value, float max_value,float min_distance, float max_distance,
				float *input, float *output,LASER_t *laser);
void LaserUpdate_min_value(uint16_t min_value, LASER_t *laser);
void LaserUpdate_max_value(uint16_t max_value, LASER_t *laser);
void LaserUpdate_min_distance(float min_distance, LASER_t *laser);
void LaserUpdate_max_distance(float max_distance, LASER_t *laser);
void LaserUpdate_ratio(float max_distance, LASER_t *laser);
void Laser (LASER_t *laser);
int  Chk_Laser_Dis(LASER_t *laser, float Des_Dis,float laser_tol);







#endif /* SRC_LASER_LASER_H_ */
