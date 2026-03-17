/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "laser.h"
/*********************************************/




/*********************************************/
/*          Variable                         */
/*********************************************/

/*********************************************/

/*********************************************/
/*           Subroutine Function             */
/*********************************************/

/*
 * Function Name		: laser_init
 * Function Description : This function is called to calibrate laser_sensor.
 * Function Remarks		: User need to initialize ADC and KF first before calling this function. To Tune Max Distance use 1 meter ruler to check accuracy.
 * Function Arguments	: min_value			minimum value of ADC/KF return (float for KF)
 * 						  max_value			maximum value of ADC/KF return (float for KF)
 * 						  min_distance 		minimum distance,in meter when min_value detected (can take from encoder)
 * 						  max_distance		maximum distance,in meter when max_value detected (can take from encoder)
 * 						  input				pointer to input of laser (can be either from ADC or KF )
 * 						  output			pointer to output of laser
 * 						  laser				pointer to structure LASER_t
 * Function Return		: None
 * Function Example		: LaserInit(121, 4035, 0.46, 2.4, &L_laser, &L_distance, &Left_laser);
 */

void LaserInit (float min_value, float max_value,
				float min_distance, float max_distance,
				float *input, float *output,
				LASER_t *laser){

	laser->min_value = min_value;
	laser->max_value = max_value;
	laser->min_distance = min_distance;
	laser->max_distance = max_distance;
	laser->input = input;
	laser->output = output;

	laser->kcd=(max_distance-min_distance)/(max_value-min_value);

	laser->count=0;

}

/*
 * Function Name		: LaserUpdate_min_value
 * Function Description : This function is called to update min_value structure laser_t
 * Function Remarks		: User need to initialize LaserInit first before calling this function.
 * Function Arguments	: min_value			minimum value of ADC/KF return
 * 						  laser				pointer to structure laser_t
 * Function Return		: None
 * Function Example		: LaserUpdate_min_value(121, &L_distance);
 */
void LaserUpdate_min_value(uint16_t min_value, LASER_t *laser){
	laser->min_value = min_value;
}

/*
* Function Name			: LaserUpdate_max_value
* Function Description  : This function is called to update max_value of structure laser_t
* Function Remarks		: User need to initialize LaserInit first before calling this function.
* Function Arguments	: max_value			maximum value of ADC/KF return
* 						  laser				pointer to structure laser_t
* Function Return		: None
* Function Example		: LaserUpdate_max_value(2024, &L_distance);
*/
void LaserUpdate_max_value(uint16_t max_value, LASER_t *laser){
	laser->max_value = max_value;
}

/*
* Function Name			: LaserUpdate_min_distance
* Function Description  : This function is called to update min_distance of structure laser_t
* Function Remarks		: User need to initialize LaserInit first before calling this function.
* Function Arguments	: min_distance 		minimum distance,in meter when min_value detected (can take from encoder)
* 						  laser				pointer to structure laser_t
* Function Return		: None
* Function Example		: LaserUpdate_min_distance(0.38, &L_distance);
*/
void LaserUpdate_min_distance(float min_distance, LASER_t *laser){
	laser->min_distance = min_distance;
}

/*
* Function Name			: LaserUpdate_max_distance
* Function Description  : This function is called to update max_distance of structure laser_t
* Function Remarks		: User need to initialize LaserInit first before calling this function.
* Function Arguments	: max_distance		maximum distance,in meter when max_value detected (can take from encoder
* 						  laser				pointer to structure laser_t
* Function Return		: None
* Function Example		: LaserUpdate_max_distance(4.38, &L_distance);
*/
void LaserUpdate_max_distance(float max_distance, LASER_t *laser){
	laser->max_distance = max_distance;
}

/*
* Function Name			: LaserUpdate_ratio
* Function Description  : This function is called to update ratio of structure laser_t
* Function Remarks		: user must must call this after update any value
* Function Arguments	: laser				pointer to structure laser_t
* Function Return		: None
* Function Example		: LaserUpdate_max_distance(4.38, &L_distance);
*/
void LaserUpdate_ratio(float max_distance, LASER_t *laser){
	laser->kcd=((laser->max_distance - laser->min_distance)/(laser->max_value - laser->min_value));
}

/*
* Function Name			: Laser
* Function Description  : This function is called to update and calculate laser distance
* Function Remarks		: user can call this in timer or anywhere
* 							(*(laser->input - laser->min_value) * laser->kcd) used to minus min_value
* 							because kcd already minus the min_value
* 							EXP: min_value=1000  max_value=3000  	max-min=2000
* 								 min_distance=0  max_distance=100	max-min=100m
* 								 kcd=(100-0)/(3000-1000) = 100/2000
* 								 if get value of 1001 = (1001-1000)*(100/2000) =  0.05m
* 								 if get value of 2999 = (2999-1000)*(100/2000) = 99.95m
* Function Arguments	: laser				pointer to structure laser_t
* Function Return		: None
* Function Example		: Laser(&left_laser);
*/
void Laser(LASER_t *laser){

//	laser->stored[laser->count++]=*(laser->input);
//
//	for(laser->count2=0;laser->count2<10;laser->count2++){
//		laser->sum+=laser->stored[laser->count2];
//	}
//	laser->average=(float)laser->sum/10.0;
	if(*(laser->input)<=(laser->min_value)){
		*(laser->input)=(laser->min_value);
	}
	if(*(laser->input)>=(laser->max_value)){
		*(laser->input)=(laser->max_value);
	}
	*(laser->output)=((*(laser->input) - laser->min_value) * laser->kcd);

	if(laser->count==10) laser->count=0;
}

int  Chk_Laser_Dis(LASER_t *laser, float Des_Dis,float laser_tol){

	if(*(laser->output) >= (Des_Dis - laser_tol) && *(laser->output) <= (Des_Dis + laser_tol)){
		return 1;
	}else{
		return 0;
	}
}





