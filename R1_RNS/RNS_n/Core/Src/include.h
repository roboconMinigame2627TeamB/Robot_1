#ifndef INCLUDE_H_
#define INCLUDE_H_
#include "cmsis_os.h"
#include "BIOS/bios.h"
//#include "main.h"
#include "common.h"
#include "task.h"
#include "interrupt.h"
#include "Fuzzy_PID/Fuzzy_PID.h"
#include "state.h"
#include "application.h"
#include "I2C/i2c.h"
#include "system.h"
#include "instruction.h"
#include "CAN/can.h"
#include "LSA/LSA.h"
#include "PID/PID.h"
#include "ABT/ABT.h"
#include "CAN/can.h"
#include "PP/PP.h"
#include "math.h"
#include "user.h"
#include "R6091U/r6091u.h"
#include "Sugeno_Fuzzy_PID/s_fuzzy_pid.h"
//#include "VESC_CAN/vesc_interface.h"

#define VelOn					(velo1 || velo2 || velo3)

#define x_fin					PathPlan.point3_p1.p_fin.x
#define y_fin					PathPlan.point3_p1.p_fin.y
#define z_fin					PathPlan.point3_p1.p_fin.z

#define prev_x_ref				PathPlan.point3_p1.prev_p_ref.x
#define prev_y_ref				PathPlan.point3_p1.prev_p_ref.y
#define prev_z_ref				PathPlan.point3_p1.prev_p_ref.z


/**********************************
          * System*
 *********************************/

comm_t	main_board_1_data_receive;         /*from commonc*/

shiftreg_t  SR;
//IMU_SPI_t	IMU;
LSA_t		LSA_A, LSA_B, LSA_C, LSA_D;
uint8_t     mmode;
yaww 		yawdata;

uint8_t insData_receive[2];
float_data_t rcv_buf[8];
uint8_t rcv_buf_flag,rcv_buf_flag2;
char buf[200];

ins_t ins;                                  /*from application.c*/
param_t param;
enq_t enq;
ODN_data_t odn;
LSAdata_t LsaData;

BDC_t BDC1,BDC2,BDC3,BDC4;

float LF_dist;
float rotspeed;
float LF_vel;
float LF_junc;
char buf[200];

uint8_t status, data;

uint8_t insData_send[2];                    /*from state.c*/
char uartbuff[100];
uint8_t feedback_ins;

unsigned char event;
unsigned char state;
char bufff[200];                            /*from system.c*/
flag_t sys;
User_flag_t UF;
ABT_t fleft_pos_data, fright_pos_data, bleft_pos_data, bright_pos_data;
ABT_t x_data, y_data;
FuzzyPID_t fuz_fleft_vel, fuz_fright_vel, fuz_bleft_vel, fuz_bright_vel;
PID_t fleft_vel, fright_vel, bleft_vel, bright_vel;
S_Fuzzy_t fuz_fleft_pid,fuz_fright_pid,fuz_bleft_pid,fuz_bright_pid;
PID_t lf_dist, lf_rotate, lf_fwd;
PID_t imu_rotate;
LSA_t *LSA_FRONT, *LSA_BACK, *LSA_LEFT, *LSA_RIGHT;
POINT_data_t rcvPoint;
PathPlan_t pp;
R6091U_t IMU;
float ppx,ppy;
float NEW_ARRAY[40][4];

float fFPtd, fFKcd, fBPtd,fBKcd;
float xPtd, yPtd;
float robot_angle, store_lsa, store_imu;
float LFtargetDist;
int Dir;
uint16_t junction_count,  temp_junc, LFtargetJunc;
float LF_vel;
int LF_angle;
int AngleTargetDeg,tempyaw,remain_angle;
int x_case, y_case, z_case;
float xstate, ystate, zstate;
float xDis,yDis,zDis;

uint8_t joy_x, joy_y, an_L2, an_R2;
float joyX, joyY, joyL_2, joyR_2;
float x_vel, y_vel, w_vel;
float x_base_vel, y_base_vel, w_base_vel;
float vel1, vel2, vel3, vel4;

int UserF1,UserF2,UserF3,UserF4,UserF5;
int UCase;

extern float fFLeftPosGain[3];
extern float fFRightPosGain[3];
extern float fBLeftPosGain[3];
extern float fBRightPosGain[3];
extern float fXPosGain[3];
extern float fYPosGain[3];

/* ABT input/output */
float fFLeftPosData, fFRightPosData, fBLeftPosData, fBRightPosData; 	/* Raw data, ABT input*/
float fFLeftPos, fFRightPos, fBLeftPos, fBRightPos;					 	/* Position, ABT output, pos PID feedback */
float fFLeftVel, fFRightVel, fBLeftVel, fBRightVel;				 		/* Velocity, ABT output */
float fFLeftAcc, fFRightAcc, fBLeftAcc, fBRightAcc;				 		/* Acceleration, ABT output*/
float fyaw;
float fyawraw;

float fXEncData, fYEncData;
float fXPos, fYPos;			/* Position, ABT output, pos PID feedback */
float fXVel, fYVel;			/* Velocity, ABT output */
float fXAcc, fYAcc;			/* Acceleration, ABT output*/

/* U_MAX,E_MAX,EC_MAX,ES_MAX,KP_B,KI_B,KD_B,KP_P,KI_P,KD_P */
extern float fFuzFLeftVG[10];
extern float fFuzFRightVG[10];
extern float fFuzBLeftVG[10];
extern float fFuzBRightVG[10];

/* SAT, KE, KU, KP, KI, KD, KN */
extern float fFLeftVG[7];
extern float fFRightVG[7];
extern float fBLeftVG[7];
extern float fBRightVG[7];

extern float fLFDistG[7];
extern float fRotateG[7];
extern float fFwdG[7];

extern float fAngleG[7];

/* PID input/output */
float fFLeftVelR, fFRightVelR, fBLeftVelR, fBRightVelR;						/* Velocity PID reference */
float fFLeftVelU, fFRightVelU, fBLeftVelU, fBRightVelU;						/* Velocity PID output */
float fFLeftVelErr, fFRightVelErr, fBLeftVelErr, fBRightVelErr;				/* Velocity PID error */

float fLFDistErr,fRotateErr,fFwdErr;
float fLFDistU,fRotateU,fFwdU;

float fAngleErr,fAngleU;
unsigned int u2rx_count;
int yaw;
unsigned int header, lastbyte;
unsigned int checksum;
char rcvdata;
char c[25];
int s,NumPoint;

int testCounter, testCounter1, testCounter2, errorInit;
float bug;
/**********************************
          *User*
 *********************************/
int timcount;
char uartbuf[512];
int icount;/*For UART*/
#endif /* INCLUDE_H_ */
