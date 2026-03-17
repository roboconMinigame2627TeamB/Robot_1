
#include "include.h"

void RNSVelocity(float M1,float M2, float M3, float M4){
	sys.vel_flag = 1;
	fFLeftVelR  = M1;
	fFRightVelR = M2;
	fBLeftVelR  = M3;
	fBRightVelR = M4;
}

void RNSStop(void){
	sys.vel_flag =0;
	fFLeftVelR  = 0;
	fFRightVelR = 0;
	fBLeftVelR  = 0;
	fBRightVelR = 0;
}

