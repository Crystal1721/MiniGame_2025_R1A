/* Mini Game 2025 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */

#define MAX_V 4.0
#define POINTS 1
#define MAX_PWM 5000
#define CAM_WIDTH 640.0
#define FOV 50.0



/* test wheel velocity */
float vel1,vel2,vel3,vel4;

/* ps4 variable */
int analog = 0;
int vel = 0;

/* perspective control variable and path planning */
float set_yaw, yaw_offset, yaw_angle, x_offset, y_offset;
int pers_ctl_flag, imu_lock= 0;

int feeding_flag = 0;
int tt_flag = 1;
int start_feed = 0;
int pid_flag = 0;
float previous_yaw;

/* Task handles */
osThreadId_t xMotionTaskHandle;
osThreadId_t xFeedingTaskHandle;
osThreadId_t xSensorTaskHandle;
osThreadId_t xretrivesballTaskHandle ;

/* Semaphore Handles*/
osSemaphoreId_t Data_CountSemphrHandle;

/* Queue Handles*/
osMessageQueueId_t sensor_QueueHandle;

/* Mutex Handles*/
osMutexId_t sensorMutex;

// ret
osStatus_t sema1;
osStatus_t sema2;
osStatus_t queue1;
osStatus_t queue2;
osPriority_t priority;


HAL_StatusTypeDef ret;
/* struct for queue data btw two tasks */
typedef struct{

	float current_yaw;
	float current_x_axis;
	float current_y_axis;
	float v1;
	float v2;
	float v3;
	float v4;
	int Analog_sensor;

}sensor;




/* Function prototypes */
void Motion(void *argument);
void Feeding(void *argument);
void Sensor(void *argument);
void Retrivesball(void *argument);

int main(void)
{

	set();
	ServoSetAngle(&servo_blk_1,75);
	ServoSetAngle(&servo_blk_2,10);
	ServoSetAngle(&servo_red,0);

	/* Definition of tasks */
	const osThreadAttr_t Motion_Task_attributes = {
			.name = "MotionTask",
			.stack_size = 256*  4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t Feeding_Task_attributes = {
			.name = "FeedingTask",
			.stack_size = 650 *  4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t Sensor_Task_attributes = {
			.name = "SensorTask",
			.stack_size = 625 *  4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t RetriveBall_Task_attributes = {
			.name = "RetriveBall",
			.stack_size = 625 *  4,
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
	const osMutexAttr_t Sensor_Mutex_attributes = {
				.name = "sensorMutex"
	};

	/* Kernel init */
	osKernelInitialize();

	/*  create Counting Semaphore */
	Data_CountSemphrHandle = osSemaphoreNew(2, 0, &Data_CountSemphr_attributes);
	/*  create Queue = send data btw tasks */
	sensor_QueueHandle = osMessageQueueNew (20, sizeof(sensor), &sensor_Queue_attributes);
	/*  create Mutex = protect data */
	sensorMutex = osMutexNew (&Sensor_Mutex_attributes);

	/*  create task  == all are in ready state */
	xMotionTaskHandle = osThreadNew(Motion, NULL, &Motion_Task_attributes);
	xFeedingTaskHandle = osThreadNew(Feeding, NULL, &Feeding_Task_attributes);
	xSensorTaskHandle = osThreadNew(Sensor, NULL, &Sensor_Task_attributes);
	xretrivesballTaskHandle = osThreadNew(Retrivesball, NULL, &RetriveBall_Task_attributes);
	/* kernel start*/
	osKernelStart();

}




void Motion(void *argument)
{
	sensor receive;
	float inital_yaw_angle = 0;
	if(RNSEnquire(RNS_X_Y_IMU_LSA, &rns) == 1)
	{
		inital_yaw_angle = rns.RNS_data.common_buffer[0].data;
	}

	MODN_t modn;
	unsigned char base = MODN_FWD_OMNI;
	float xr,yr,wr;
	float vel1,vel2,vel3,vel4;

	MODNRobotBaseInit(base,0.5,0.5,&modn);
	MODNRobotVelInit(&xr,&yr,&wr,&modn);
	MODNWheelVelInit(&vel1,&vel2,&vel3, &vel4,&modn);

	while(1)
	{

		queue2 = osMessageQueueGet(sensor_QueueHandle, &receive, NULL, osWaitForever);
	    float current_yaw_angle = receive.current_yaw;

		float rad = (current_yaw_angle-inital_yaw_angle)*(M_PI/180.0f);

		float x_vel = (float) ps4.joyL_x;
		float y_vel = -(float) ps4.joyL_y;

		xr = x_vel*cos(rad) + (- y_vel*sin(rad));
		yr = -(x_vel*sin(rad) + y_vel*cos(rad));
		wr = -ps4.joyR_x;

		/* for tuning only */

//			wr = ps4.joyR_2 - ps4.joyL_2;
//			xr = ps4.joyR_x;
//			yr = ps4.joyL_y;


		MODN(&modn);


		if((ps4.joyL_x !=0 || ps4.joyL_y !=0|| ps4.joyR_x !=0) && pid_flag == 0)
		{
			RNSVelocity(vel1*2.0,vel2*2.0,vel3*2.0,vel4*2.0,&rns);

		}
		else if (pid_flag == 1)
		{
			RNSVelocity(vel1*1.0,vel2*1.0,vel3*1.0,vel4*1.0,&rns);
		}
		else
		{
			RNSStop(&rns);
		}

			/* Process data */

		switch(ps4.button)
		{
		case SQUARE: //  Stop the tt motor
			while(ps4.button == SQUARE);
			StopBDC(&BDC7);
			StopBDC(&BDC8);
			start_feed = 0;
			tt_flag = 0;
			pid_flag = 0;
			break;
		case PS:
			while(ps4.button == PS);
			StopBDC(&BDC7);
			StopBDC(&BDC8);
			RNSStop(&rns);
			HAL_NVIC_SystemReset();
			break;
		}
		osDelay(10);
	}
}

void Feeding(void *argument)
{
	/*
	 * 1) When the analog sensor is detected the ball,
	 *    the servo will feed to the r2
	 *
	 */
	/* Receive Data by Queue */

	while(1)
	{
		sema1 = osSemaphoreAcquire(Data_CountSemphrHandle, osWaitForever);
		if(feeding_flag && tt_flag)
		{
//			osDelay(200);
			feeding_flag = 0;
			start_feed = 1;
			pid_flag = 0;
			WriteBDC(&BDC8,950);
		}
		if(start_feed)
		{
			ServoSetAngle(&servo_red,(fabs(ps4.joyR_2))*100);
//			if(ps4.button == TOUCH)
//			{
//				while(ps4.button == TOUCH);
//				start_feed = 0;
//				tt_flag = 0;
//				pid_flag = 0;
////				StopBDC(&BDC8);
//			}
		}
		osDelay(5);
	}
}

void Retrivesball(void *argument)
{
	/*
	 *  1) press one button, servo retrive the ball
	 *  2) when servo is close, the timing belt start rotates
	 */

	while(1)
	{

		sema2 = osSemaphoreAcquire(Data_CountSemphrHandle, osWaitForever);
		switch(ps4.button)
		{
		case L1: // Open Servo
			while(ps4.button == L1);
			ServoSetAngle(&servo_blk_1,10);
			ServoSetAngle(&servo_blk_2,75);
//			yaw_offset = ((receive_pos.current_x - (CAM_WIDTH/2.0))*FOV/CAM_WIDTH) + receive.current_yaw;
			pid_flag = 1;
			tt_flag = 1;
			break;
		case R1: // Close Servo
			while(ps4.button == R1);
			ServoSetAngle(&servo_blk_1,75);
			ServoSetAngle(&servo_blk_2,10);
			osDelay(20);
			tt_flag = 1;
			pid_flag = 0;
			RNSStop(&rns);
			break;
		}

		if(tt_flag)
		{
			/* may add some manually control to control timing belt */
			if(ps4.button == UP)
			{
//				while(ps4.button == UP);
				WriteBDC(&BDC8,1700);
			}
			else if (ps4.button == DOWN)
			{
//				while(ps4.button == DOWN);
				WriteBDC(&BDC8,-950);
			}
			else if (ps4.button == TRIANGLE)
			{
//				while(ps4.button == DOWN);
				if(pid_flag){
					WriteBDC(&BDC7,950);
				}
				else{
					WriteBDC(&BDC7,1700);
				}
			}
			else if (ps4.button == CROSS)
			{
//				while(ps4.button == DOWN);
				WriteBDC(&BDC7,-950);
			}
			else if (ps4.button == (TRIANGLE|UP))
			{
//				while(ps4.button == DOWN);
				WriteBDC(&BDC7,1700);
				WriteBDC(&BDC8,1700);
			}
			else if (ps4.button == (DOWN|CROSS))
			{
//				while(ps4.button == DOWN);
				WriteBDC(&BDC7,-950);
				WriteBDC(&BDC8,-950);
			}else
			{
				StopBDC(&BDC7);
				StopBDC(&BDC8);
			}
		}
		/* start automation */
//		if(pid_flag)
//		{
//			if(ps4.button == UP)
//			{
////				while(ps4.button == UP);
//				WriteBDC(&BDC8,1550);
//			}
////			Err_angle = fmod(yaw_offset - receive.current_yaw + 540, 360) - 180;
////			wr = F_angle;
//////			yr = ps4.joyL_y;
////			MODN(&Modn);
////			RNSVelocity(v1*2.0,v2*2.0,v3*2.0,v4*2.0,&rns);
//		}
//		else{
//			RNSSet(&rns, RNS_DEVICE_CONFIG, (float) 0b00000000, (float) fwd_omni, (float) roboconPID);
//		}
		osDelay(15);
	}
}


void Sensor(void *argument)
{
	/*
	 * 1) Read the rns board's encoder values
	 * 2) Read the analog sensor
	 *
	 */
	/* Write Data in Queue */
	sensor sender;

	while(1)
	{
		/* write value of wheel en, imu, x,y enc, and laser and write in queue */
		if(RNSEnquire(RNS_X_Y_IMU_LSA, &rns) == 1)
		{
			osMutexAcquire(sensorMutex, osWaitForever);
			ret = HAL_UART_Receive(&huart3, (uint8_t*)py_buffer, 4,10);
			sscanf(py_buffer, "%f", &norm_cx);
			receive_pos.current_x =  norm_cx;
			sender.current_yaw =  rns.RNS_data.common_buffer[0].data ;
			sender.current_x_axis= rns.RNS_data.common_buffer[1].data;
			sender.current_y_axis= rns.RNS_data.common_buffer[2].data;
			osMutexRelease(sensorMutex);
		}
		if (RNSEnquire(RNS_VEL_BOTH,&rns) == 1)
		{
			osMutexAcquire(sensorMutex, osWaitForever);
			sender.v1 = rns.RNS_data.common_buffer[0].data;
			sender.v2 = rns.RNS_data.common_buffer[1].data;
			sender.v3 = rns.RNS_data.common_buffer[2].data;
			sender.v4 = rns.RNS_data.common_buffer[3].data;
			char response[80];
	//		snprintf(response, sizeof(response), "x: %.3f y: %.3f a: %.3f \n", sender.current_x_axis, sender.current_y_axis, sender.current_yaw);
			snprintf(response, sizeof(response), "%.3f,%.3f,%.3f,%.3f\n", fabs(sender.v1), sender.v2, sender.v3, fabs(sender.v4));
			UARTPrintString(&huart2, response);
			osMutexRelease(sensorMutex);
		}

		if(Sensor_P == 0)
		{
			feeding_flag = 1;
		}
		else
		{
			feeding_flag = 0;
		}
		if (osMessageQueuePut(sensor_QueueHandle, &sender, 0,osWaitForever) == osOK) {
			sema2 = osSemaphoreRelease(Data_CountSemphrHandle);
		}
		receive_pos.current_y =  norm_cy;
		osDelay(15);
	}
}


void TIM6_DAC_IRQHandler(void)
{

	sema1 = osSemaphoreRelease(Data_CountSemphrHandle);
	led1=!led1;
	if(pid_flag)
	{
		PID(&imu_rotate);
	}
	else{
		PIDDelayInit(&imu_rotate);
	}
	PSxConnectionHandler(&ps4);
	HAL_TIM_IRQHandler(&htim6);
}

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
