

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */

#include<math.h>


#define tuning_PID
#define perspective_control
#define PS4

#define MAX_V 4.0
#define POINTS 1
#define MAX_PWM 4000


/* test wheel velocity */
float vel1,vel2,vel3,vel4;
/* ps4 variable */
int analog = 0;
int vel = 0;
/* pid tuning variable*/
int busy = 0;
char wheel_or_imu;
/* perspective control variable and path planning */
float set_yaw, yaw_offset, yaw_angle, x_offset, y_offset;
int pers_ctl_flag, imu_lock= 0;

int servo_flag = 0;

float point[POINTS][7] = {
//		{2.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0},
//		{2.0, -1.0, 1.0, -90.0, 1.0, 1.0, 0.0},
//		{2.0, -1.0, 0.0, -180.0, 1.0, 1.0, 0.0},
//		{2.0, 0.0, 0.0, -270.0, 1.0, 0.0, 0.0},
//		{2.0, -1.0, 1.0, 0.0, 1.0, 1.0, 0.0},
		{1.5, 0.0, 2.0, 0.0, 1.0, 1.0, 1.5}
};


/* Task handles */
osThreadId_t xMotionTaskHandle;
osThreadId_t xServoTaskHandle;
osThreadId_t xSensorTaskHandle;
osThreadId_t xPPTaskHandle ;

/* Semaphore Handles*/
osSemaphoreId_t Data_CountSemphrHandle;

/* Queue Handles*/
osMessageQueueId_t sensor_QueueHandle;

/* Mutex Handles*/
osMutexId_t offsetMutex;

// ret
osStatus_t ret;
osPriority_t priority;

typedef struct{

	float current_yaw;
	float current_x_axis;
	float current_y_axis;

}sensor;

/* Function prototypes */
void Controller(void *argument);
void Motion(void *argument);
void Servo(void *argument);
void Sensor(void *argument);
void PathPlanning(void *argument);
void tune_pid(void);


int main(void)
{


	set();
	ServoSetAngle(&servo_blk_1,0);
	ServoSetAngle(&servo_blk_2,0);
	ServoSetAngle(&servo_red,135);
	/* Definition of tasks */
	const osThreadAttr_t Servo_Task_attributes = {
			.name = "ServoTask",
			.stack_size = 512 *  4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t Sensor_Task_attributes = {
			.name = "SensorTask",
			.stack_size = 512 *  4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t Motion_Task_attributes = {
			.name = "MotionTask",
			.stack_size = 512 *  4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t PP_Task_attributes = {
			.name = "PPTask",
			.stack_size = 512 *  4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	/* Definition of Queue */
	const osMessageQueueAttr_t sensor_Queue_attributes = {
			.name = "SensorQueue"
	};

	/* Definition of Semaphore */
	const osSemaphoreAttr_t Data_CountSemphr_attributes = {
				.name = "DatCountSemphr"
	};

	/* Definition of Mutex */
	const osMutexAttr_t Offset_Mutex_attributes = {
				.name = "OffsetMutex"
	};

	/* Kernel init */
	osKernelInitialize();

	/*  create Counting Semaphore */
	Data_CountSemphrHandle = osSemaphoreNew(1, 0, &Data_CountSemphr_attributes);
	/*  create Queue = send data btw tasks */
	sensor_QueueHandle = osMessageQueueNew (10, sizeof(sensor), &sensor_Queue_attributes);
	/*  create Queue = send data btw tasks */
	offsetMutex = osMutexNew (&Offset_Mutex_attributes);

	/*  create task  == all are in ready state */
	xServoTaskHandle = osThreadNew(Servo, NULL, &Servo_Task_attributes);
	xSensorTaskHandle = osThreadNew(Sensor, NULL, &Sensor_Task_attributes);
	xMotionTaskHandle = osThreadNew(Motion, NULL, &Motion_Task_attributes);
	xPPTaskHandle = osThreadNew(PathPlanning, NULL, &PP_Task_attributes);
	/* kernel start*/
	osKernelStart();

}

void Motion(void *argument)
{

	sensor receive;

	while(1)
	{
		PIDDelayInit (&imu_rotate);
		PIDDelayInit (&x_axis);
		PIDDelayInit (&y_axis);
		 osMessageQueueGet(sensor_QueueHandle, &receive, NULL, osWaitForever);
		 osSemaphoreAcquire(Data_CountSemphrHandle, osWaitForever);
			/* Process data */
			switch(ps4.button){
				case UP:
					RNSVelocity(2.0,2.0,2.0,2.0,&rns);
	//				RNSPDC(MAX_PWM,MAX_PWM,MAX_PWM,MAX_PWM,&rns);
					while(ps4.button == UP);
					break;
				case DOWN:
					RNSVelocity(-2.0,-2.0,-2.0,-2.0,&rns);
	//				RNSPDC(-MAX_PWM,-MAX_PWM,-MAX_PWM,-MAX_PWM,&rns);
					while(ps4.button == DOWN);
					break;
				case LEFT:
					RNSVelocity(-2.0,2.0,2.0,-2.0,&rns);
	//				RNSPDC(-MAX_PWM,MAX_PWM,MAX_PWM,-MAX_PWM,&rns);
					while(ps4.button == LEFT);
					break;
				case RIGHT:
					RNSVelocity(2.0,-2.0,-2.0,2.0,&rns);
	//				RNSPDC(MAX_PWM,-MAX_PWM,-MAX_PWM,MAX_PWM,&rns);
					while(ps4.button == RIGHT);
					break;
				case OPTION:
					pers_ctl_flag = 0;
					imu_lock = 0;
					analog = 1;
					vel = 0;
					while(ps4.button == OPTION);
					break;
				case TRIANGLE:
					pers_ctl_flag = 0;
					imu_lock = 0;
					analog = 0;
					vel = 1;
					while(ps4.button == TRIANGLE);
					break;
				case R1:
					pers_ctl_flag = 1;
					imu_lock = 0;
					analog = 0;
					vel = 0;
					yaw_offset = receive.current_yaw;
					while(ps4.button == R1);
					break;
				case L1:
					imu_lock = 1;
					pers_ctl_flag = 0;
					analog = 0;
					vel = 0;

					yaw_offset = receive.current_yaw;
					x_offset = receive.current_x_axis;
					y_offset = receive.current_y_axis;
					while(ps4.button == L1);
					break;
				case PS:
					RNSStop(&rns);
					PIDDelayInit (&imu_rotate);
					PIDDelayInit (&x_axis);
					PIDDelayInit (&y_axis);
					analog = 0;
					vel = 0;
					pers_ctl_flag = 0;
					imu_lock = 0;
					HAL_NVIC_SystemReset();
					while(ps4.button == PS);
					break;
				default:
					RNSStop(&rns);
					break;
				}
			if(analog == 1)
			{
				led2 = 0;
				yr = ps4.joyL_y;
				xr = -ps4.joyR_x;
				wr = ps4.joyR_2 - ps4.joyL_2;
				MODN(&Modn);
	//			RNSVelocity((MAX_V*v1),(MAX_V*v2), (MAX_V*v3), (MAX_V*v4), &rns);
				RNSPDC(MAX_PWM*v1,MAX_PWM*v2,MAX_PWM*v3,MAX_PWM*v4,&rns);
			}
			if (vel == 1)
			{
				RNSVelocity(1.0,1.0,1.0,1.0,&rns);
			}

			if(pers_ctl_flag == 1)
			{
				yr = -ps4.joyL_y;
				xr = ps4.joyR_x;
				wr = ps4.joyR_2 - ps4.joyL_2;

				yaw_angle = receive.current_yaw;

				float yaw_rad = yaw_angle * M_PI /180.0;

				float temp_fwd = yr*cos(yaw_rad) + xr*sin(yaw_rad);
				xr = -yr*sin(yaw_rad) + xr*cos(yaw_rad);
				yr = temp_fwd;

				MODN(&Modn);
	//			RNSVelocity(v1, v2, v3, v4, &rns);
				RNSPDC(MAX_PWM*v1,MAX_PWM*v2,MAX_PWM*v3,MAX_PWM*v4,&rns);
			}
			if(imu_lock)
			{
				led3 = 0;
				Err_x = x_offset - receive.current_x_axis;
				Err_y = y_offset - receive.current_y_axis;
				Err_angle = fmod(yaw_offset - receive.current_yaw + 540, 360) - 180;
				xr = F_x;
				yr = F_y;
				wr = F_angle;
				MODN(&Modn);
	//			RNSVelocity(v1, v2, v3, v4, &rns);
				RNSPDC(MAX_PWM*v1,MAX_PWM*v2,MAX_PWM*v3,MAX_PWM*v4,&rns);

		}

		osDelay(15);
	}
}

void Servo(void *argument)
{

	while(1)
	{

		if(servo_flag)
		{
			ServoSetAngle(&servo_blk_1,135);
			ServoSetAngle(&servo_blk_2,135);

		}else
		{
			ServoSetAngle(&servo_blk_1,0);
			ServoSetAngle(&servo_blk_2,0);
		}
		if(ps4.button == SQUARE)
		{
			ServoSetAngle(&servo_red,0);
			while(ps4.button == SQUARE);
		}
		else
		{
			ServoSetAngle(&servo_red,135);
			servo_flag = 0;
		}
		osDelay(20);
	}

}

void Sensor(void *argument)
{

	sensor sender;
	while(1)
	{
		/* write value of wheel en, imu, x,y enc, and laser and write in queue */
		if(RNSEnquire(RNS_X_Y_IMU_LSA, &rns) == 1)
		{
			osMutexAcquire(offsetMutex, osWaitForever);
			sender.current_yaw =  rns.RNS_data.common_buffer[0].data;
			osMutexRelease(offsetMutex);
		}
		if(RNSEnquire(RNS_X_Y_POS, &rns) == 1)
		{
			osMutexAcquire(offsetMutex, osWaitForever);
			sender.current_x_axis= rns.RNS_data.common_buffer[0].data;
			sender.current_y_axis= rns.RNS_data.common_buffer[1].data;
			osMutexRelease(offsetMutex);
		}
		if(Sensor_P == 0)
		{
			servo_flag = 1;
		}
		else
		{
			servo_flag = 0;
		}
		if (osMessageQueuePut(sensor_QueueHandle, &sender, 0,osWaitForever) != osOK) {
		   led3 = 1;
		}
		char response[80];
		snprintf(response, sizeof(response), "x: %.3f y: %.3f a: %.3f\n", sender.current_x_axis, sender.current_y_axis, sender.current_yaw);
		UARTPrintString(&huart2, response);
		osDelay(25);
	}
}

void PathPlanning(void *argument)
{

	analog = 0;
	vel = 0;
	pers_ctl_flag = 0;
	imu_lock = 0;


	while(1)
	{

		if(ps4.button == TOUCH)
		{
			while(ps4.button == TOUCH);
			led2 = 0;
			priority = osThreadGetPriority(xPPTaskHandle);
			osThreadSetPriority(xPPTaskHandle, priority + 5);
			RNSPPstart(point,POINTS,&rns);
		}
		if(rns.RNS_data.common_instruction!=RNS_BUSY)
		{
//			osThreadTerminate(xPPTaskHandle);
//			 osThreadSuspend(xPPTaskHandle);
			osThreadSetPriority(xPPTaskHandle, priority - 6);
			led2 =! led2;
		}
//		osDelay(30);
	}
}

void TIM6_DAC_IRQHandler(void)
{

	ret = osSemaphoreRelease(Data_CountSemphrHandle);
	led1=!led1;
	PSxConnectionHandler(&ps4);
	PID(&imu_rotate);
	PID(&x_axis);
	PID(&y_axis);
	HAL_TIM_IRQHandler(&htim6);
}


//void tune_pid(void){
//
//	if (HAL_UART_Receive(&huart2, buffer_g, 25, 150) == HAL_OK)
//	{
//		busy = 1;
//		RNSStop(&rns);
//		RNSSet(&rns, RNS_RESET_POS);
//		HAL_Delay(500);
//
//
//		float p_value, i_value, d_value;
//
//	//		    format "1 p 0.00 i 0.000 d 0.000"
//		if (sscanf((char*)buffer_g, "%c p %f i %f d %f", & wheel_or_imu, &p_value, &i_value, &d_value) == 4) {
//			PID_t *targetWheel = NULL;
//			PID_t *IMU_Rotate= NULL;
//
//			switch (wheel_or_imu) {
//				case '1': targetWheel = &w1; break;
//				case '2': targetWheel = &w2; break;
//				case '3': targetWheel = &w3; break;
//				case '4': targetWheel = &w4; break;
//				case 'a': IMU_Rotate = &imu_rotate; break;
//				default:
//					busy = 0;
//					break;
//			}
//
//			if((wheel_or_imu >= '1' && wheel_or_imu <= '4'))
//			{
//				 PIDGainSet(KP, p_value, targetWheel);
//				 PIDGainSet(KI, i_value, targetWheel);
//				 PIDGainSet(KD, d_value, targetWheel);
//			}
//			else if(wheel_or_imu == 'a')
//			{
//				PIDGainSet(KP, p_value, IMU_Rotate);
//				PIDGainSet(KI, i_value, IMU_Rotate);
//				PIDGainSet(KD, d_value, IMU_Rotate);
//			}
//			char response[80];
//			snprintf(response, sizeof(response), "Updated: r_%c p=%.3f i=%.3f d=%.3f\n", wheel_or_imu, p_value, i_value, d_value);
//			UARTPrintString(&huart2, response);
//		}
//
//		busy = 0;
//	}
//
//	if (busy == 0) {
//		static float prev_KP[5] = {-1, -1, -1, -1, -1};
//		static float prev_KI[5] = {-1, -1, -1, -1, -1};
//		static float prev_KD[5] = {-1, -1, -1, -1, -1};
//
//		float new_KP[5] = {w1.K[KP], w2.K[KP], w3.K[KP], w4.K[KP], imu_rotate.K[KP]};
//		float new_KI[5] = {w1.K[KI], w2.K[KI], w3.K[KI], w4.K[KI], imu_rotate.K[KI]};
//		float new_KD[5] = {w1.K[KD], w2.K[KD], w3.K[KD], w4.K[KD], imu_rotate.K[KD]};
//
//		bool updateNeeded[5] = {false, false, false, false, false};
//
//		for (int i = 0; i < 5; i++) {
//			if (new_KP[i] != prev_KP[i] || new_KI[i] != prev_KI[i] || new_KD[i] != prev_KD[i]) {
//				updateNeeded[i] = true;
//				prev_KP[i] = new_KP[i];
//				prev_KI[i] = new_KI[i];
//				prev_KD[i] = new_KD[i];
//			}
//		}
//
//		if (updateNeeded[0]) RNSSet(&rns, RNS_F_LEFT_VEL_PID,  new_KP[0], new_KI[0], new_KD[0]);
//		if (updateNeeded[1]) RNSSet(&rns, RNS_F_RIGHT_VEL_PID, new_KP[1], new_KI[1], new_KD[1]);
//		if (updateNeeded[2]) RNSSet(&rns, RNS_B_LEFT_VEL_PID,  new_KP[2], new_KI[2], new_KD[2]);
//		if (updateNeeded[3]) RNSSet(&rns, RNS_B_RIGHT_VEL_PID, new_KP[3], new_KI[3], new_KD[3]);
//		if (updateNeeded[4]) RNSSet(&rns, RNS_ROTATE_PID, new_KP[4], new_KI[4], new_KD[4]);
//
//	}
//
//	char txbuffer[100];
//
//	if((wheel_or_imu >= '1' && wheel_or_imu <= '4'))
//	{
//		sprintf(txbuffer, "%.2f, %.2f, %.2f, %.2f\n", vel1, vel2, vel3, vel4);
//	}
//	else if(wheel_or_imu == 'a')
//	{
//		sprintf(txbuffer, "%.2f\n", current_yaw);
//	}
//
//	UARTPrintString(&huart2, txbuffer);
//}

/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler(void)
{


}
#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
