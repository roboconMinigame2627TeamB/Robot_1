
/*********************************************/
/*          Include Header                   */
/*********************************************/

#include "common.h"
#include "math.h"

void set(void) {

	Initialize();
//	PSxInitDMA(&ps4,&hi2c1);
	TIMxInit(&htim6, 20000, 84, 5, 0); //20ms
	RNS_config(&hcan1);
//	MODNRobotBaseInit(MODN_FWD_OMNI, 2.0, 1.0, &Modn);
//	MODNRobotVelInit(&xr, &yr, &wr, &Modn);
//	MODNWheelVelInit(&v1, &v2, &v3, &v4, &Modn);
	led2 = 1;
	led3 = 1;
}


void RNS_config(CAN_HandleTypeDef* hcanx) {
	RNSInit(hcanx, &rns);

	//Encoder dcba(0-swap, 1-keep)  BDC dcba(0-keep, 1-swap) //0x00 0x00 0x
	RNSSet(&rns, RNS_DEVICE_CONFIG, (float) 0b00010101, (float) fwd_omni, (float) roboconPID);
	RNSSet(&rns, RNS_X_Y_ENC_CONFIG, 0.125 / 4000 * 3.142, 1.0, 0.125 / 4000 * 3.142, 1.0); //1.0 for nonswap , 2.0 for swap
	RNSSet(&rns, RNS_F_KCD_PTD, 203.20885/ 204.50492, (float)(0.125 * 3.142 / 203.20885));
	RNSSet(&rns, RNS_B_KCD_PTD, 203.56232/ 203.60160, (float)(0.125 * 3.142 / 203.56232));

	RNSSet(&rns, RNS_F_LEFT_VEL_SATEU, 1.0, 1.0 / 17.9120, 19999.0);
	RNSSet(&rns, RNS_F_RIGHT_VEL_SATEU, 1.0, 1.0 / 20.7897, 19999.0);
	RNSSet(&rns, RNS_B_LEFT_VEL_SATEU, 1.0, 1.0 / 18.3077, 19999.0);
	RNSSet(&rns, RNS_B_RIGHT_VEL_SATEU, 1.0, 1.0 / 18.7605, 19999.0);

	RNSSet(&rns, RNS_F_LEFT_VEL_PID,  5.0, 3.86, 0.0);
	RNSSet(&rns, RNS_F_RIGHT_VEL_PID, 4.6, 3.13, 0.0);
	RNSSet(&rns, RNS_B_LEFT_VEL_PID,  4.78, 3.32, 0.0);
	RNSSet(&rns, RNS_B_RIGHT_VEL_PID, 5.03, 3.50, 0.0);

	RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_BASE, 0.2, 0.2, 0.2);
	RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_PARAM, 0.02, 0.02, 0.02);

	RNSSet(&rns, RNS_PPInit); //Path Planning
	RNSSet(&rns, RNS_PPPathPID, 1.0, 0.5, 0.5);
	RNSSet(&rns, RNS_PPEndPID, 0.5, 0.1, 0.7);
	RNSSet(&rns, RNS_PPZPID, 1.0, 0.05, 0.2, 5.5);
	RNSSet(&rns, RNS_PPSetCRV_PTS, 10.0);         // Change No. of Points in the Curved Path
}


