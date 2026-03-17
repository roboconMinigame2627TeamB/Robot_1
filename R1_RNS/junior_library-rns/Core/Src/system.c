
/*********************************************/
/*          Include Header                   */
/*********************************************/

#include "include.h"

/*********************************************/
/*          Variable                         */
/*********************************************/


/*********************************************/
/*           Private Function Prototype      */
/*********************************************/

float Sin(float angle);
float Cos(float angle);
void LineFollow(float vel, float angle, float rotate_value, float w);

/*********************************************/
/*           Subroutine Function             */
/*********************************************/

void SYSSystemInit(void)
{
	/* Left position ABT*/
	ABTInit(SAMPLE_TIME, fFLeftPosGain[0], fFLeftPosGain[1], fFLeftPosGain[2], &fFLeftPosData, &fFLeftPos, &fFLeftVel, &fFLeftAcc, &fleft_pos_data);
	ABTEstimateInit(&fleft_pos_data);

	ABTInit(SAMPLE_TIME, fBLeftPosGain[0], fBLeftPosGain[1], fBLeftPosGain[2], &fBLeftPosData, &fBLeftPos, &fBLeftVel, &fBLeftAcc, &bleft_pos_data);
	ABTEstimateInit(&bleft_pos_data);

	/* Right position ABT */
	ABTInit(SAMPLE_TIME, fFRightPosGain[0], fFRightPosGain[1], fFRightPosGain[2], &fFRightPosData, &fFRightPos, &fFRightVel, &fFRightAcc, &fright_pos_data);
	ABTEstimateInit(&fright_pos_data);

	ABTInit(SAMPLE_TIME, fBRightPosGain[0], fBRightPosGain[1], fBRightPosGain[2], &fBRightPosData, &fBRightPos, &fBRightVel, &fBRightAcc, &bright_pos_data);
	ABTEstimateInit(&bright_pos_data);

	/* X & Y position ABT */
	ABTInit(SAMPLE_TIME, fXPosGain[0], fXPosGain[1], fXPosGain[2], &fXEncData ,&fXPos, &fXVel, &fXAcc, &x_data);
	ABTEstimateInit(&x_data);

	ABTInit(SAMPLE_TIME, fYPosGain[0], fYPosGain[1], fYPosGain[2], &fYEncData, &fYPos, &fYVel, &fYAcc, &y_data);
	ABTEstimateInit(&y_data);

	//FUZZYPID
	/* Left velocity PID*/
	FuzzyPIDSourceInit(&fFLeftVelErr, &fFLeftVelU, &fuz_fleft_vel);
	FuzzyPIDGainInit(fFuzFLeftVG[0], fFuzFLeftVG[1], fFuzFLeftVG[2], fFuzFLeftVG[3], fFuzFLeftVG[4], fFuzFLeftVG[5], fFuzFLeftVG[6], fFuzFLeftVG[7],fFuzFLeftVG[8],fFuzFLeftVG[9], &fuz_fleft_vel);

	FuzzyPIDSourceInit(&fBLeftVelErr, &fBLeftVelU, &fuz_bleft_vel);
	FuzzyPIDGainInit(fFuzBLeftVG[0], fFuzBLeftVG[1], fFuzBLeftVG[2], fFuzBLeftVG[3], fFuzBLeftVG[4], fFuzBLeftVG[5], fFuzBLeftVG[6], fFuzBLeftVG[7], fFuzBLeftVG[8], fFuzBLeftVG[9], &fuz_bleft_vel);

	/* Right velocity PID*/
	FuzzyPIDSourceInit(&fFRightVelErr, &fFRightVelU, &fuz_fright_vel);
	FuzzyPIDGainInit(fFuzFRightVG[0], fFuzFRightVG[1], fFuzFRightVG[2], fFuzFRightVG[3], fFuzFRightVG[4], fFuzFRightVG[5], fFuzFRightVG[6], fFuzFRightVG[7], fFuzFRightVG[8], fFuzFRightVG[9], &fuz_fright_vel);

	FuzzyPIDSourceInit(&fBRightVelErr, &fBRightVelU, &fuz_bright_vel);
	FuzzyPIDGainInit(fFuzBRightVG[0], fFuzBRightVG[1], fFuzBRightVG[2], fFuzBRightVG[3], fFuzBRightVG[4], fFuzBRightVG[5], fFuzBRightVG[6], fFuzBRightVG[7], fFuzBRightVG[8], fFuzBRightVG[9], &fuz_bright_vel);

	//ROBOCONPID
	/* Left velocity PID*/
	PIDSourceInit(&fFLeftVelErr, &fFLeftVelU, &fleft_vel);
	PIDDelayInit(&fleft_vel);
	PIDGainInit(SAMPLE_TIME, fFLeftVG[0], fFLeftVG[1], fFLeftVG[2], fFLeftVG[3], fFLeftVG[4], fFLeftVG[5], fFLeftVG[6], &fleft_vel);

	PIDSourceInit(&fBLeftVelErr, &fBLeftVelU, &bleft_vel);
	PIDDelayInit(&bleft_vel);
	PIDGainInit(SAMPLE_TIME, fBLeftVG[0], fBLeftVG[1], fBLeftVG[2], fBLeftVG[3], fBLeftVG[4], fBLeftVG[5], fBLeftVG[6], &bleft_vel);

	/* Right velocity PID*/
	PIDSourceInit(&fFRightVelErr, &fFRightVelU, &fright_vel);
	PIDDelayInit(&fright_vel);
	PIDGainInit(SAMPLE_TIME, fFRightVG[0], fFRightVG[1], fFRightVG[2], fFRightVG[3], fFRightVG[4], fFRightVG[5], fFRightVG[6], &fright_vel);

	PIDSourceInit(&fBRightVelErr, &fBRightVelU, &bright_vel);
	PIDDelayInit(&bright_vel);
	PIDGainInit(SAMPLE_TIME, fBRightVG[0], fBRightVG[1], fBRightVG[2], fBRightVG[3], fBRightVG[4], fBRightVG[5], fBRightVG[6], &bright_vel);

	//Line follow
	/* Distance PID*/
	PIDSourceInit(&fLFDistErr, &fLFDistU, &lf_dist);
	PIDGainInit(SAMPLE_TIME, fLFDistG[0], fLFDistG[1], fLFDistG[2], fLFDistG[3], fLFDistG[4], fLFDistG[5], fLFDistG[6], &lf_dist);
	PIDDelayInit(&lf_dist);

	/* Rotate PID*/
	PIDSourceInit(&fRotateErr, &fRotateU, &lf_rotate);
	PIDGainInit(SAMPLE_TIME, fRotateG[0], fRotateG[1], fRotateG[2], fRotateG[3], fRotateG[4], fRotateG[5], fRotateG[6], &lf_rotate);
	PIDDelayInit(&lf_rotate);

	/*  */
	PIDSourceInit(&fFwdErr, &fFwdU, &lf_fwd);
	PIDGainInit(SAMPLE_TIME, fFwdG[0], fFwdG[1], fFwdG[2], fFwdG[3], fFwdG[4], fFwdG[5], fFwdG[6], &lf_fwd);
	PIDDelayInit(&lf_fwd);

	//IMU Rotate PID
	PIDSourceInit(&fAngleErr, &fAngleU, &imu_rotate);
	PIDGainInit(SAMPLE_TIME, fAngleG[0], fAngleG[1], fAngleG[2], fAngleG[3], fAngleG[4], fAngleG[5], fAngleG[6], &imu_rotate);
	PIDDelayInit(&imu_rotate);

	//	VESCInit(40000,1,0.0037,&vesc);
	S_FuzzyInit(&fuz_fleft_pid, &fleft_vel, 2.5, 2.5, 1.0, 3.0);
	S_FuzzyInit(&fuz_fright_pid, &fright_vel, 2.5, 2.5, 1.0, 3.0);
	S_FuzzyInit(&fuz_bleft_pid, &bleft_vel, 2.5, 2.5, 1.0, 3.0);
	S_FuzzyInit(&fuz_bright_pid, &bright_vel, 2.5, 2.5, 1.0, 3.0);

}

void SYSSystem5ms(void)
{
	/* Obtain position for left and right */

	fFLeftPosData = fFPtd * (QEIRead(QEI6) - MIN_POSCNT);
	fFRightPosData = fFKcd * fFPtd * (QEIRead(QEI4) - MIN_POSCNT);

	fBLeftPosData = fBPtd * (QEIRead(QEI1) -MIN_POSCNT);
	fBRightPosData = fBKcd * fBPtd * (QEIRead(QEI3) - MIN_POSCNT);

	fXEncData = xPtd * (QEIRead(QEI2) - MIN_POSCNT);
	fYEncData = yPtd * (QEIRead(QEI5) - MIN_POSCNT);

	/* ABT filter */
	ABT(&fleft_pos_data);
	ABT(&fright_pos_data);

	ABT(&bleft_pos_data);
	ABT(&bright_pos_data);

	ABT(&x_data);
	ABT(&y_data);

	fyaw = (fyaw >= 0.0) ? fyaw : 360.0 + fyaw;

	PathPlan(&pp);

	if (sys.lf_flag) {

		LSAErr_Handler(&LSA_A);
		LSAErr_Handler(&LSA_B);
		LSAErr_Handler(&LSA_C);
		LSAErr_Handler(&LSA_D);

		if(LSA_RIGHT->LSA_T == 8 || LSA_FRONT->LSA_T == 8){
			if(sys.junc_flag == 0){
				junction_count++;
				sys.junc_flag = 1;
			}
		} else{
			sys.junc_flag = 0;
		}
	}

	if (sys.activate){

		//LineFollowStart//
		if(sys.lf_flag){

			sys.busy_flag = 1;

			if(Dir == DIR_LEFT || Dir == DIR_RIGHT){

				if((LSA_LEFT->LSA_T != 0) && (LSA_RIGHT->LSA_T != 0)){
					fFwdErr = LSA_RIGHT->PosErr - LSA_LEFT->PosErr;
					store_lsa = LSA_RIGHT->PosErr + LSA_LEFT->PosErr;
					store_imu = fyaw;
					robot_angle = store_lsa;
				}else{
					robot_angle = store_lsa + store_imu - fyaw;
				}

				fLFDistErr = LFtargetDist - fXEncData;

			} else if(Dir == DIR_FRONT || Dir == DIR_BACK) {

				if((LSA_FRONT->LSA_T != 0) && (LSA_BACK->LSA_T != 0)){
					fFwdErr = LSA_FRONT->PosErr - LSA_BACK->PosErr;
					store_lsa = LSA_FRONT->PosErr + LSA_BACK->PosErr;
					store_imu = fyaw;
					robot_angle = store_lsa;
				}else{
					robot_angle = store_lsa + store_imu - fyaw;
				}

				fLFDistErr = LFtargetDist - fYEncData;
			}

			fRotateErr = robot_angle;

			PID(&lf_dist);
			PID(&lf_rotate);
			PID(&lf_fwd);

			if(sys.dist_flag){//DISTANCE LINE FOLLOW
				if(sys.lf_right || sys.lf_front){
					if(Dir == DIR_RIGHT){
						if(fXEncData < LFtargetDist){
							LineFollow(fLFDistU + 0.2 ,fFwdU, fRotateU, 1.5);
						} else if(fRotateErr != 0.0){
							LineFollow(0.0,0.0,fRotateU,2.0);
						} else {
							LineFollow(0.0,0.0,0.0,0.0);
							sys.busy_flag = 0;
						}
					}else if(Dir == DIR_FRONT){
						if(fYEncData < LFtargetDist){
							LineFollow(fLFDistU + 0.2 ,fFwdU,fRotateU, 1.5);
						} else if(fRotateErr != 0.0){
							LineFollow(0.0,0.0,fRotateU,2.0);
						} else {
							LineFollow(0.0,0.0,0.0,0.0);
							sys.busy_flag = 0;
						}
					}
				}else if(sys.lf_left || sys.lf_back){
					if(Dir == DIR_LEFT){
						if(fXEncData > LFtargetDist){
							LineFollow(-fLFDistU - 0.2 ,-fFwdU,fRotateU, 1.5);
						} else if(fRotateErr != 0.0){
							LineFollow(0.0,0.0,fRotateU,2.0);
						} else {
							LineFollow(0.0,0.0,0.0,0.0);
							sys.busy_flag = 0;
						}
					} else if(Dir == DIR_BACK){
						if(fYEncData > LFtargetDist){
							LineFollow(-fLFDistU - 0.2 ,-fFwdU,fRotateU, 1.5);
						} else if(fRotateErr != 0.0){
							LineFollow(0.0,0.0,fRotateU,2.0);
						} else {
							LineFollow(0.0,0.0,0.0,0.0);
							sys.busy_flag = 0;
						}
					}
				}
			} else if(sys.lf_junc){//JUNCTION LINE FOLLOW
				if(sys.lf_right || sys.lf_front){
					if(junction_count < LFtargetJunc){
						LineFollow(fLFDistU + 0.2,fFwdU,fRotateU, 1.5);
					} else if(fRotateErr != 0.0){
						LineFollow(0.0,0.0,fRotateU,2.0);
					} else {
						LineFollow(0.0,0.0,0.0, 0.0);
						sys.busy_flag = 0;
					}
				} else if(sys.lf_left || sys.lf_back){
					if(junction_count < LFtargetJunc){
						LineFollow(-fLFDistU - 0.2,-fFwdU,fRotateU, 1.5);
					} else if(fRotateErr != 0.0){
						LineFollow(0.0,0.0,fRotateU,2.0);
					} else {
						LineFollow(0.0,0.0,0.0,0.0);
						sys.busy_flag = 0;
					}
				}
			}
		}
		//LineFollowEnd//

		//ROBOT ROTATE USING IMU Start - testing //+ve:clkwise,-ve:anti-clkwise,max:+-180
		if (sys.rotate){
			sys.busy_flag = 1;

			fAngleErr = (float)(AngleTargetDeg - (int16_t)fyaw);


			if((int16_t)fAngleErr < -360){
				tempyaw = (int16_t)fyaw - 360;
				fAngleErr = (float)(AngleTargetDeg - tempyaw);
			}else if((int16_t)fAngleErr > 360){
				tempyaw = 360 + (int16_t)fyaw;
				fAngleErr = (float)(AngleTargetDeg - tempyaw);
			}

			PID(&imu_rotate);

			fFLeftVelR  = fAngleU;
			fBLeftVelR  = fAngleU;
			fFRightVelR = -fAngleU;
			fBRightVelR = -fAngleU;

			if(fAngleErr == 0.0)
				sys.busy_flag = 0;
		}

		//ROBOT ROTATE USING IMU End - testing

		//PathPLanningStart//
		if(sys.odnstart_flag && pp.pp_start) {

			switch (dev_cfg.base_type){

			case tri_omni:
				fFLeftVelR  = pp.v1;
				fBLeftVelR  = pp.v3;
				fFRightVelR = pp.v2;
				fBRightVelR = 0.00;
				break;

			case fwd_omni:
				fFLeftVelR  = pp.v2;
				fFRightVelR = pp.v1;
				fBLeftVelR  = pp.v3;
				fBRightVelR = pp.v4;
				break;

			default:
				break;
			}

		}
		//PathPlanningEnd//

		//UserStart
		if(UF.user){

		}
		//After finish running your task, reset UF.user flag to
		//enable it to go to UserFinish state

		//UserEnd
		if (sys.vel_flag){
			fFLeftVelErr = fFLeftVelR - fFLeftVel;
			fBLeftVelErr = fBLeftVelR - fBLeftVel;
			fFRightVelErr = fFRightVelR - fFRightVel;
			fBRightVelErr = fBRightVelR - fBRightVel;

			if(dev_cfg.PID_type == fuzzyPID){
				FuzzyPID(&fuz_fleft_vel);
				FuzzyPID(&fuz_bleft_vel);
				FuzzyPID(&fuz_fright_vel);
				FuzzyPID(&fuz_bright_vel);
			} else if(dev_cfg.PID_type == roboconPID){
				PID(&fleft_vel);
				PID(&bleft_vel);
				PID(&fright_vel);
				PID(&bright_vel);
			}else if(dev_cfg.PID_type == s_fuzzyPID){
				S_FuzzyLoop(&fuz_fleft_pid);
				S_FuzzyLoop(&fuz_fright_pid);
				S_FuzzyLoop(&fuz_bleft_pid);
				S_FuzzyLoop(&fuz_bright_pid);

				PID(&fleft_vel);
				PID(&bleft_vel);
				PID(&fright_vel);
				PID(&bright_vel);
			}
		}
	}

}
void SYSSystemAct(void)
{
	WriteBDC(&BDC1,(int32_t)fFLeftVelU);
	WriteBDC(&BDC2,(int32_t)fFRightVelU);
	WriteBDC(&BDC3,(int32_t)fBLeftVelU);
	WriteBDC(&BDC4,(int32_t)fBRightVelU);
	//	VESCPDC(fFLeftVelU/20000,fFRightVelU/20000,fBLeftVelU/20000,fBRightVelU/20000, &vesc);
}

/*********************************************/
/*           Private Function 			     */
/*********************************************/

void LineFollow(float vel, float angle, float rotate_value, float w){

	if(Dir == DIR_LEFT || Dir == DIR_RIGHT){
		if(dev_cfg.base_type == fwd_omni){
			fFLeftVelR  =  vel * Cos((int)angle + 45) + rotate_value * w;
			fFRightVelR =  vel * Sin((int)angle - 135) - rotate_value * w;
			fBLeftVelR  =  vel * Sin((int)angle - 135) + rotate_value * w;
			fBRightVelR =  vel * Cos((int)angle + 45) - rotate_value * w;
		}else if(dev_cfg.base_type == tri_omni){
			fFLeftVelR  =  vel * Cos((int)angle +60) + rotate_value * w;
			fFRightVelR =  vel * Cos((int)angle +120) - rotate_value * w;
			fBLeftVelR  =  vel * Cos((int)angle + 0) - rotate_value * w;
		}
	} else if(Dir == DIR_FRONT || Dir == DIR_BACK){
		if(dev_cfg.base_type == fwd_omni){
			fFLeftVelR  =  vel * Cos((int)angle - 45) + rotate_value * w;
			fFRightVelR =  vel * Sin((int)angle + 135) - rotate_value * w;
			fBLeftVelR  =  vel * Sin((int)angle + 135) + rotate_value * w;
			fBRightVelR =  vel * Cos((int)angle - 45) - rotate_value * w;
		} else if(dev_cfg.base_type ==  tri_omni){
			fFLeftVelR  =  vel * Sin((int)angle +60) + rotate_value * w;
			fFRightVelR =  vel * Sin((int)angle +120) - rotate_value * w;
			fBLeftVelR  =  vel * Sin((int)angle + 0) - rotate_value * w;
		}
	}
}

float Sin(float angle){
	float pi, sin;
	pi = angle * 3.141592654 / 180;
	sin=sinf(pi);
	return sin;
}

float Cos(float angle){
	float pi, cos;
	pi = angle * 3.141592654 / 180;
	cos = cosf(pi);
	return cos;
}
