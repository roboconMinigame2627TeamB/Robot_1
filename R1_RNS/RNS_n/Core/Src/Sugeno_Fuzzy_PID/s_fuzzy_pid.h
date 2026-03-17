#ifndef SUGENO_FUZZY_PID_S_FUZZY_PID_H_
#define SUGENO_FUZZY_PID_S_FUZZY_PID_H_

#include "../PID/PID.h"

typedef struct {

	float MaxKP;
	float MaxKI;
	float MaxKD;
	float Error_scale;

	float e_membership[6];
	float de_membership[6];
	float previous_error;

	float d_Error;
	float delta_P;
	float delta_I;
	float delta_D;
	PID_t *PID;
}S_Fuzzy_t;

void S_FuzzyInit(S_Fuzzy_t *Fuzzy,PID_t *PID,float Max_KP,float Max_KI,float Max_KD, float Error_Scale);
void Fuzzification(float Input,float Output[]);
float Defuzzification(float Weights[]);
void S_FuzzyLoop(S_Fuzzy_t *Fuzzy);
float Fuzzy_Sum(float Weights[]);
float Fuzzy_and(float x, float y);
float Fuzzy_or(float x, float y);
#endif /* SUGENO_FUZZY_PID_S_FUZZY_PID_H_ */
