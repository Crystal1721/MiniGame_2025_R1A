
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
//	MODNRobotBaseInit(MODN_FWD_OMNI, 1.0, 1.0, &Modn); // d is half of wheel base(distance btw frt and bck). e is half of track width (left & right)
//	MODNRobotVelInit(&xr, &yr, &wr, &Modn);
//	MODNWheelVelInit(&v1, &v2, &v3, &v4, &Modn);
	PSxSlaveInit(&ps4,&hi2c1);

//	R6091U_Init(&imu,&huart2);
//	R6091U_Handler(&imu);
	PSx_SlaveHandler(&ps4);

	PIDSourceInit (&Err_angle, &F_angle, &imu_rotate);
	PIDDelayInit (&imu_rotate);
	PIDGainInit(0.02, 1.0, 1/180.0, 180, 0.05, 0.0, 0.0,  2*3.1415926*10.0,  &imu_rotate);

	ServoxInit(&servo_blk_1, &htim3, GPIOA, GPIO_PIN_0, TIM_CHANNEL_4);
	ServoxInit(&servo_blk_2, &htim3, GPIOA, GPIO_PIN_1, TIM_CHANNEL_3);
	ServoxInit(&servo_red, &htim9, GPIOA, GPIO_PIN_2, TIM_CHANNEL_1);

	ServoInitAngle(&servo_red, 500 , 2500);
	ServoInitAngle(&servo_blk_1, 500 , 2500);
	ServoInitAngle(&servo_blk_2, 500, 2500);


	led2 = 1;
	led3 = 1;
}

void RNS_config(CAN_HandleTypeDef* hcanx) {
	RNSInit(hcanx, &rns);

	//Encoder dcba(0-swap, 1-keep)  BDC dcba(0-keep, 1-swap) //0x00 0x00 0x
	RNSSet(&rns, RNS_DEVICE_CONFIG, (float) 0b00000000, (float) fwd_omni, (float) roboconPID);
	RNSSet(&rns, RNS_X_Y_ENC_CONFIG, 0.05 / 3016.0* 3.142, 1.0, 0.05 / 4040.0 * 3.142, 1.0); //1.0 for nonswap , 2.0 for swap // 0.05 = diameter of external wheel// 4000 pulse per revolusion
	RNSSet(&rns, RNS_F_KCD_PTD, 517.0/506.0, (float)(0.125 * 3.142 / 517.0)); // as reminder pulse is 500 not 200 and diameter wheel is 0.125m , so 200 is for tuning raw, not actual
	RNSSet(&rns, RNS_B_KCD_PTD, 518.0/520.0, (float)(0.125 * 3.142 / 518.0));

	RNSSet(&rns, RNS_F_LEFT_VEL_SATEU, 1.0, 1.0 / 17.9120, 19999.0); // 17.9120 is max speed of each respective motor
	RNSSet(&rns, RNS_F_RIGHT_VEL_SATEU, 1.0, 1.0 / 19.7897, 19999.0);
	RNSSet(&rns, RNS_B_LEFT_VEL_SATEU, 1.0, 1.0 / 18.3077, 19999.0);
	RNSSet(&rns, RNS_B_RIGHT_VEL_SATEU, 1.0, 1.0 / 18.7605, 19999.0);

	RNSSet(&rns, RNS_F_LEFT_VEL_PID,  2.85, 3.2, 0.055);
	RNSSet(&rns, RNS_F_RIGHT_VEL_PID, 3.23, 4.0, 0.055);
	RNSSet(&rns, RNS_B_LEFT_VEL_PID,  3.13, 3.7, 0.056);
	RNSSet(&rns, RNS_B_RIGHT_VEL_PID, 3.08, 3.75, 0.056);

	RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_BASE, 0.2, 0.2, 0.2);
	RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_PARAM, 0.02, 0.02, 0.02);

	RNSSet(&rns, RNS_PPInit); //Path Planning
	RNSSet(&rns, RNS_PPPathPID, 0.015, 0.00, 0.0);
	RNSSet(&rns, RNS_PPEndPID, 0.005, 0.00, 0.1);
	RNSSet(&rns, RNS_PPZPID, 0.015, 0.0, 0.0, 5.5);
	RNSSet(&rns, RNS_PPSetCRV_PTS, 20.0);         // Change No. of Points in the Curved Path
}


