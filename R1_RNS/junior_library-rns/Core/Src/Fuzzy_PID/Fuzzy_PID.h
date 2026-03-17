/*
 * Fuzzy_PID.h
 *
 *  Created on: 29 Jun 2018
 *      Author: user
 */

#ifndef SRC_FUZZY_PID_FUZZY_PID_H_
#define SRC_FUZZY_PID_FUZZY_PID_H_



/*********************************************/
/*          Include Header                   */
/*********************************************/
#include <math.h>
/*********************************************/



/*********************************************/
/*          Define                           */
/*********************************************/

/*********************************************/




/*********************************************/
/*          Enumarator                      _ */
/*********************************************/
enum {NB = -3, NM, NS, ZO, PS, PM, PB};
enum {U_MAX = 0, E_MAX, EC_MAX, ES_MAX, E_MIN, EC_MIN, ES_MIN,
	  KP_B, KI_B, KD_B, KP_P, KI_P, KD_P};
/*********************************************/




/*********************************************/
/*          Variable                         */
/*********************************************/


typedef struct{
	float *input;
	float *output;
	float err, prev_err;
	float errc;
	float errs;
	int   errCal, errcCal;
	int   set_Kp, set_Ki, set_Kd;
	float Kp, Ki, Kd;

	float K[13];

	int isfirst ;
	float outp ;
	float outt;

}FuzzyPID_t;


/*********************************************/




/*********************************************/
/*           Function Prototype              */
/*********************************************/
void FuzzyPIDSourceInit (float *in, float *out, FuzzyPID_t *fpid);
void FuzzyPIDGainInit(float ku, float err_max, float errc_max, float errs_max,
					  float kp_base, float ki_base, float kd_base,
					  float kp_param, float ki_param, float kd_param, FuzzyPID_t *fpid);
void FuzzyPIDGainSet (unsigned char parameter, float value, FuzzyPID_t *fpid);
void FuzzyPID (FuzzyPID_t *fpid);


/*********************************************/


#endif /* SRC_FUZZY_PID_FUZZY_PID_H_ */
