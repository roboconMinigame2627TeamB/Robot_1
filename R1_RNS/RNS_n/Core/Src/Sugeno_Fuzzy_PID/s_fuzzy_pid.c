#include "s_fuzzy_pid.h"



void S_FuzzyInit(S_Fuzzy_t *Fuzzy,PID_t *PID,float Max_KP,float Max_KI,float Max_KD, float Error_Scale){

	Fuzzy->PID = PID;
	Fuzzy->MaxKP = Max_KP;
	Fuzzy->MaxKI = Max_KI;
	Fuzzy->MaxKD = Max_KD;
	Fuzzy->Error_scale = Error_Scale;
	Fuzzy->previous_error=0;
}


void Fuzzification(float Input,float Output[]){

	float Membership[7] = {0, 0, 0, 0, 0, 0, 0};
		// NB
		if (Input <= -3){
			Membership[0] = 1;
		}
		else if (-3 < Input && Input <= -2){
			Membership[0] = (-2 - Input) / 1 ;
		}
		// NM
		if (-3 <= Input && Input <= -2){
			Membership[1] = Input + 3;
		}
		else if( -2 < Input && Input<= -1){
			Membership[1] = (-1 - Input) / 1;
		}
		// NS
		if (-2 <= Input && Input <= -1){
			Membership[2] = (Input + 2) / 1;
		}
		else if (-1 < Input && Input <= 0){
			Membership[2] = (0 - Input) / 1;
		}
		// ZE
		if (-1 <= Input && Input <= 0){
			Membership[3] = (Input + 1) / 1;
		}
		else if ( 0 < Input && Input <= 1){
			Membership[3] = (1 - Input) / 1;
		}
		// PS
		if (0 <= Input && Input <= 1){
			Membership[4] = (Input - 0) / 1;
		}
		else if (1 < Input && Input <= 2){
			Membership[4] = (2 - Input) / 1;
		}
		// PM
		if (1 <= Input && Input <= 2){
			Membership[5] = (Input - 1) / 1;
		}
		else if (2 < Input && Input <= 3){
			Membership[5] = 3 - Input;
		}
		//PB
		if (2 <= Input && Input < 3){
			Membership[6] = (Input - 2) / 1;
		}
		else if (3 <= Input){
			Membership[6] = 1;
		}

//		memcpy(Output,Membership,6*sizeof(float));
		memcpy(Output,Membership,7*sizeof(float));
}

float Defuzzification(float Weights[]){

   /* Sugeno-style
	# input is a list of membership
	# [NB, NM, NS, ZE, PS, PM, PB]
	# output must be rescaled
	*/

	float numerator = 0;
	float denominator = 0;
	if (Fuzzy_Sum(Weights) == 0){
		return 0;
	}
	for (int i = 0; i<7; i++){
		numerator += Weights[i] * (i - 3);
		denominator += Weights[i];
	}
	return numerator / denominator;
}

void S_FuzzyLoop(S_Fuzzy_t *Fuzzy){

	enum {NB=0,NM,NS,ZE,PS,PM,PB};
/*
 *  6|
 *  5|
 *  4|
 *  3|
 *  2|
 *  1|
 *   |--------------------------
 */
	int Fuzzy_Rules_P[7][7] = {
		{PB, PB, PM, PM, PS, ZE, ZE},
		{PB, PB, PM, PS, PS, ZE, ZE},
		{PM, PM, PM, PS, ZE, NS, NS},
		{PM, PM, PS, ZE, NS, NM, NM},
		{PS, PS, ZE, NS, NS, NM, NM},
		{PS, ZE, NS, NM, NM, NM, NB},
		{ZE, ZE, NM, NM, NM, NB, NB}
	};

	int Fuzzy_Rules_I[7][7] = {
		{PS, NS, NB, NB, NB, NM, PS},
		{PS, NS, NB, NM, NM, NS, ZE},
		{ZE, NS, NM, NM, NS, NS, ZE},
		{ZE, NS, NS, NS, NS, NS, ZE},
		{ZE, ZE, ZE, ZE, ZE, ZE, ZE},
		{PB, NS, PS, PS, PS, PS, PB},
		{PB, PM, PM, PM, PS, PS, PB}
	};

	int Fuzzy_Rules_D[7][7] = {
		{NB, NB, NM, NM, NS, ZE, ZE},
		{NB, NB, NM, NS, NS, ZE, ZE},
		{NB, NM, NS, NS, ZE, PS, PS},
		{NM, NM, NS, ZE, PS, PM, PM},
		{NM, NS, ZE, PS, PS, PM, PB},
		{ZE, ZE, PS, PS, PM, PB, PB},
		{ZE, ZE, PS, PM, PM, PB, PB}
	};


	float fuzzy_P[7] = {0, 0, 0, 0, 0, 0, 0};
	float fuzzy_I[7] = {0, 0, 0, 0, 0, 0, 0};
	float fuzzy_D[7] = {0, 0, 0, 0, 0, 0, 0};

	float fuzzy_Error[7] = {0, 0, 0, 0, 0, 0, 0};
	float fuzzy_d_Error[7] = {0, 0, 0, 0, 0, 0, 0};

	float Error = *(Fuzzy->PID->error);
	float d_Error = Error - Fuzzy->previous_error;
	Fuzzy->d_Error = d_Error;

	Fuzzification(Error/Fuzzy->Error_scale, fuzzy_Error);
	Fuzzification(d_Error/(Fuzzy->Error_scale*2), fuzzy_d_Error);

	for(int i = 0; i<7; i++){
		for(int j = 0; j<7; j++){
			fuzzy_P[Fuzzy_Rules_P[i][j]] = Fuzzy_or(Fuzzy_and(fuzzy_Error[i], fuzzy_d_Error[j]), fuzzy_P[Fuzzy_Rules_P[i][j]]);
			fuzzy_I[Fuzzy_Rules_I[i][j]] = Fuzzy_or(Fuzzy_and(fuzzy_Error[i], fuzzy_d_Error[j]), fuzzy_I[Fuzzy_Rules_I[i][j]]);
			fuzzy_D[Fuzzy_Rules_D[i][j]] = Fuzzy_or(Fuzzy_and(fuzzy_Error[i], fuzzy_d_Error[j]), fuzzy_D[Fuzzy_Rules_D[i][j]]);
		}
	}

	float delta_P = Defuzzification(fuzzy_P);
	float delta_I = Defuzzification(fuzzy_I);
	float delta_D = Defuzzification(fuzzy_D);

	Fuzzy->delta_P = delta_P;
	Fuzzy->delta_I = delta_I;
	Fuzzy->delta_D = delta_D;

	Fuzzy->PID->K[KP] = Fuzzy_and( Fuzzy_or(0, Fuzzy->PID->K[KP] + delta_P) ,  Fuzzy->MaxKP);
	Fuzzy->PID->K[KI] = Fuzzy_and(Fuzzy_or(0, Fuzzy->PID->K[KI] + delta_I), Fuzzy->MaxKI);
	Fuzzy->PID->K[KD] = Fuzzy_and(Fuzzy_or(0, Fuzzy->PID->K[KD] + delta_D), Fuzzy->MaxKD);
//	PIDCoeffCalc(Fuzzy->PID);
	Fuzzy->previous_error = Error;//added
}


float Fuzzy_Sum(float Weights[]){
	float sum;
	for (int i = 0; i<7;i++){
		sum+=Weights[i];
	}
	return sum;
}

float Fuzzy_and(float x, float y){
		return (x>y)?y:x;
}

float Fuzzy_or(float x, float y){
		return (x>y)?x:y;
}
