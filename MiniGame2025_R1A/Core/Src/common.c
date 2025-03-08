
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
	MODNRobotBaseInit(MODN_FWD_OMNI, 1.0, 1.0, &Modn); // d is half of wheel base(distance btw frt and bck). e is half of track width (left & right)
	MODNRobotVelInit(&xr, &yr, &wr, &Modn);
	MODNWheelVelInit(&v1, &v2, &v3, &v4, &Modn);
	PSxSlaveInit(&ps4,&hi2c1);

//	R6091U_Init(&imu,&huart2);
//	R6091U_Handler(&imu);
	PSx_SlaveHandler(&ps4);

	PIDSourceInit (&Err_l, &F_l, &ltt);
	PIDDelayInit (&ltt);
	PIDGainInit(0.02, 1.0, 1/20000.0, 1.0, 0.0, 0.0, 0.0, 2*3.1415926*10.0,  &ltt);

	PIDSourceInit (&Err_u, &F_u,  &utt);
	PIDDelayInit (&utt);
	PIDGainInit(0.02, 1.0, 1/20000.0, 1.0, 0.0, 0.0, 0.0, 2*3.1415926*10.0,  &utt);

	ServoxInit(&servo_blk_1, &htim3, GPIOA, GPIO_PIN_0, TIM_CHANNEL_4);
	ServoxInit(&servo_blk_2, &htim3, GPIOA, GPIO_PIN_1, TIM_CHANNEL_3);
	ServoxInit(&servo_red, &htim9, GPIOA, GPIO_PIN_2, TIM_CHANNEL_1);

	ServoInitAngle(&servo_red, 500 , 2500);
	ServoInitAngle(&servo_blk_1, 500 , 2500);
	ServoInitAngle(&servo_blk_2, 500, 2500);

	QEIReset(QEI1); // LOWER TT
	QEIReset(QEI4); // UPPER TT

	QEIWrite(QEI1, MIN_POSCNT);
	QEIWrite(QEI4, MIN_POSCNT);

	float ttlowerPosGain[3] = {0.8, 0.4, 0.2};
	float ttupperPosGain[3] = {0.8, 0.4, 0.2};

	ABTInit(SAMPLE_TIME, ttlowerPosGain[0], ttlowerPosGain[1], ttlowerPosGain[2], &tt_lowerEncData ,&tt_lowerPos, &tt_lowerVel, &tt_lowerAcc, &tt_lower_data);
	ABTEstimateInit(&tt_lower_data);

	ABTInit(SAMPLE_TIME, ttupperPosGain[0], ttupperPosGain[1], ttupperPosGain[2], &tt_upperEncData, &tt_upperPos, &tt_upperVel, &tt_upperAcc, &tt_upper_data);
	ABTEstimateInit(&tt_upper_data);

	tt_lower = 0.0;
	tt_upper = 0.0;

	led2 = 1;
	led3 = 1;
}

void RNS_config(CAN_HandleTypeDef* hcanx) {
	RNSInit(hcanx, &rns);

	//Encoder dcba(0-swap, 1-keep)  BDC dcba(0-keep, 1-swap) //0x00 0x00 0x
	RNSSet(&rns, RNS_DEVICE_CONFIG, (float) 0b00101101, (float) fwd_omni, (float) roboconPID);
	RNSSet(&rns, RNS_X_Y_ENC_CONFIG, 0.05 / 4000 * 3.142, 1.0, 0.05 / 4000 * 3.142, 1.0); //1.0 for nonswap , 2.0 for swap // 0.05 = diameter of external wheel// 4000 pulse per revolusion
	RNSSet(&rns, RNS_F_KCD_PTD, 203.20885/ 204.50492, (float)(0.125 * 3.142 / 203.20885)); // as reminder pulse is 500 not 200 and diameter wheel is 0.125m , so 200 is for tuning raw, not actual
	RNSSet(&rns, RNS_B_KCD_PTD, 203.56232/ 203.60160, (float)(0.125 * 3.142 / 203.56232));

	RNSSet(&rns, RNS_F_LEFT_VEL_SATEU, 1.0, 1.0 / 17.9120, 19999.0); // 17.9120 is max speed of each respective motor
	RNSSet(&rns, RNS_F_RIGHT_VEL_SATEU, 1.0, 1.0 / 20.7897, 19999.0);
	RNSSet(&rns, RNS_B_LEFT_VEL_SATEU, 1.0, 1.0 / 18.3077, 19999.0);
	RNSSet(&rns, RNS_B_RIGHT_VEL_SATEU, 1.0, 1.0 / 18.7605, 19999.0);

	RNSSet(&rns, RNS_F_LEFT_VEL_PID,  0.4, 4.2, 0.00);
	RNSSet(&rns, RNS_F_RIGHT_VEL_PID, 0.6, 4.2, 0.01);
	RNSSet(&rns, RNS_B_LEFT_VEL_PID,  0.4, 4.5, 0.00);
	RNSSet(&rns, RNS_B_RIGHT_VEL_PID, 0.6, 3.8, 0.01);

	RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_BASE, 0.2, 0.2, 0.2);
	RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_PARAM, 0.02, 0.02, 0.02);

	RNSSet(&rns, RNS_PPInit); //Path Planning
	RNSSet(&rns, RNS_PPPathPID, 0.015, 0.00, 0.0);
	RNSSet(&rns, RNS_PPEndPID, 0.005, 0.00, 0.1);
	RNSSet(&rns, RNS_PPZPID, 0.015, 0.0, 0.0, 5.5);
	RNSSet(&rns, RNS_PPSetCRV_PTS, 20.0);         // Change No. of Points in the Curved Path
}


