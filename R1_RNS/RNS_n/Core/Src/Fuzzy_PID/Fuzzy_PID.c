/*
 * Fuzzy_PID.c
 *
 *  Created on: 29 Jun 2018
 *      Author: user
 */


/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "Fuzzy_PID.h"
/*********************************************/


/*********************************************/
/*          Variable                         */	/*based on China 2016 Coding*/
/*********************************************/
//int Kp_Rules[7][7] =   {{PB,PB,PM,PM,PS,ZO,ZO},
//						{PB,PB,PM,PS,PS,ZO,NS},
//						{PM,PM,PM,PS,ZO,NS,NS},
//						{PM,PM,PS,ZO,NS,NM,NM},
//						{PS,PS,ZO,NS,NS,NM,NM},
//						{PS,ZO,NS,NM,NM,NM,NB},
//						{ZO,ZO,NM,NM,NM,NB,NB}};

//int Ki_Rules[7][7] =   {{NB,NB,NB,NB,NM,ZO,ZO},		//refer to document
//						{NB,NB,NB,NB,NM,ZO,ZO},
//						{NM,NM,NM,NM,ZO,PS,PS},
//						{NM,NM,NS,ZO,PS,PM,PM},
//						{NS,NS,ZO,PM,PM,PM,PM},
//						{ZO,ZO,PS,PS,PM,PB,PB},
//						{ZO,ZO,PS,PM,PM,PB,PB}};

//int Kd_Rules[7][7] =   {{PS,NS,NB,NB,NB,NM,PS},
//						{PS,NS,NB,NM,NM,NS,ZO},
//						{ZO,NS,NM,NM,NS,NS,ZO},
//						{ZO,NS,NS,NS,NS,NS,ZO},
//						{ZO,ZO,ZO,ZO,ZO,ZO,ZO},
//						{PB,NS,PS,PS,PS,PS,PB},
//						{PB,PM,PM,PM,PS,PS,PB}};



/*********************************************/
/*           Subroutine Function             */
/*********************************************/
void FuzzyPIDSourceInit(float *in, float *out, FuzzyPID_t *fpid)
{
	fpid->input = in;
	fpid->output = out;
	fpid->isfirst = 1;
	fpid->outp = 0.0;

}

void FuzzyPIDGainInit(float ku, float err_max, float errc_max, float errs_max,
					  float kp_base, float ki_base, float kd_base,
					  float kp_param, float ki_param, float kd_param, FuzzyPID_t *fpid)
{
	fpid->K[U_MAX]  = ku;
	fpid->K[E_MAX]  = err_max;	fpid->K[E_MIN] = -err_max;
	fpid->K[EC_MAX] = errc_max;	fpid->K[EC_MIN] = -errc_max;
	fpid->K[ES_MAX] = errs_max;	fpid->K[ES_MIN] = -errs_max;
	fpid->K[KP_B]   = kp_base;
	fpid->K[KI_B]   = ki_base;
	fpid->K[KD_B]   = kd_base;
	fpid->K[KP_P]   = kp_param;
	fpid->K[KI_P]   = ki_param;
	fpid->K[KD_P]   = kd_param;
}

void FuzzyPIDGainSet(unsigned char parameter, float value, FuzzyPID_t *fpid)
{
	fpid->K[parameter] = value;
}

void FuzzyPID(FuzzyPID_t *fpid)
{
//	static int isfirst = 1;
//	static float outp = 0.0;
	int Kp_Rules[7][7] =   {{PB,PB,PM,PM,PS,ZO,ZO},
							{PB,PB,PM,PS,PS,ZO,NS},
							{PM,PM,PM,PS,ZO,NS,NS},
							{PM,PM,PS,ZO,NS,NM,NM},
							{PS,PS,ZO,NS,NS,NM,NM},
							{PS,ZO,NS,NM,NM,NM,NB},
							{ZO,ZO,NM,NM,NM,NB,NB}};

	int Ki_Rules[7][7] =   {{NB,NB,NB,NB,NM,ZO,ZO},		//refer to document
							{NB,NB,NB,NB,NM,ZO,ZO},
							{NM,NM,NM,NM,ZO,PS,PS},
							{NM,NM,NS,ZO,PS,PM,PM},
							{NS,NS,ZO,PM,PM,PM,PM},
							{ZO,ZO,PS,PS,PM,PB,PB},
							{ZO,ZO,PS,PM,PM,PB,PB}};

	int Kd_Rules[7][7] =   {{PS,NS,NB,NB,NB,NM,PS},
							{PS,NS,NB,NM,NM,NS,ZO},
							{ZO,NS,NM,NM,NS,NS,ZO},
							{ZO,NS,NS,NS,NS,NS,ZO},
							{ZO,ZO,ZO,ZO,ZO,ZO,ZO},
							{PB,NS,PS,PS,PS,PS,PB},
							{PB,PM,PM,PM,PS,PS,PB}};

	fpid->err	= *fpid->input;
	fpid->errc 	= fpid->err - fpid->prev_err;
	fpid->prev_err = fpid->err;

	if (fpid->isfirst){
		fpid->errs = 0;
		*fpid->output = 0;
		fpid->isfirst = 0;
	} else{
		fpid->errs 	+= fpid->err;

		if(fabs(fpid->err) > fpid->K[E_MAX])
			(fpid->err > 0)   ? (fpid->err = fpid->K[E_MAX])   : (fpid->err = fpid->K[E_MIN]);
		if(fabs(fpid->errc) > fpid->K[EC_MAX])
			(fpid->errc > 0)  ? (fpid->errc = fpid->K[EC_MAX]) : (fpid->errc = fpid->K[EC_MIN]);
		if(fabs(fpid->errs) > fpid->K[ES_MAX])
			(fpid->errs > 0)  ? (fpid->errs = fpid->K[ES_MAX]) : (fpid->errs = fpid->K[ES_MIN]);

		(fpid->err > 0)	 ? (fpid->errCal  = (int)(fpid->err * 3.0  / fpid->K[E_MAX] + 0.5f))  : (fpid->errCal  = (int)(fpid->err * 3.0  / fpid->K[E_MAX] - 0.5f));
		(fpid->errc > 0) ? (fpid->errcCal = (int)(fpid->errc * 3.0 / fpid->K[EC_MAX] + 0.5f)) : (fpid->errcCal = (int)(fpid->errc * 3.0 / fpid->K[EC_MAX] - 0.5f));

		fpid->set_Kp = Kp_Rules[fpid->errCal + 3][fpid->errcCal + 3];
		fpid->set_Ki = Ki_Rules[fpid->errCal + 3][fpid->errcCal + 3];
		fpid->set_Kd = Kd_Rules[fpid->errCal + 3][fpid->errcCal + 3];

		fpid->Kp = fpid->K[KP_B] + fpid->set_Kp * fpid->K[KP_P];
		fpid->Ki = fpid->K[KI_B] + fpid->set_Ki * fpid->K[KI_P];
		fpid->Kd = fpid->K[KD_B] + fpid->set_Kd * fpid->K[KD_P];

		fpid->outp = (((fpid->err * fpid->Kp) + (fpid->errs * fpid->Ki) - (fpid->errc * fpid->Kd)) / fpid->K[E_MAX]);
		fpid->outt = fpid->outp;
		fpid->outp = fpid->outp * fpid->K[U_MAX];

		if (fabs(fpid->outp) > fpid->K[U_MAX])
			(fpid->outp > 0) ? (fpid->outp = fpid->K[U_MAX]) : (fpid->outp = -fpid->K[U_MAX]);
		*fpid->output = fpid->outp;
	}
}
