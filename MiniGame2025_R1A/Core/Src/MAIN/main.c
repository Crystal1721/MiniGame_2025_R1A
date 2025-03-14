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
int tt_flag = 0;
int start_feed = 0;
int pid_flag = 0;
float previous_angle = 0.0;

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
	int Analog_sensor;

}sensor;


sensor receive;

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
	RNSEnquire(RNS_X_Y_IMU_LSA, &rns);
	float inital_yaw_angle = (float)(rns.enq.enq_buffer[0].data);

	while(1)
	{
	    float current_yaw_angle = (float)(rns.enq.enq_buffer[0].data);

		float rad = (current_yaw_angle-inital_yaw_angle)*(M_PI/180.0f);

		float x_vel =(float) ps4.joyL_x;
		float y_vel =(float) ps4.joyL_y;

		xr = x_vel*cos(rad) +(- y_vel*sin(rad));
		yr = -(x_vel*sin(rad) + y_vel*cos(rad));
		wr = (float)ps4.joyR_x;
		MODN(&Modn);


		if(ps4.joyL_x !=0 || ps4.joyL_y !=0 || ps4.joyR_x !=0)
		{
			RNSVelocity(v1*2.0,v2*2.0,v3*2.0,v4*2.0,&rns);

		}
//		PIDDelayInit (&ltt);
//		PIDDelayInit (&utt);
			/* Process data */

		switch(ps4.button)
		{
		case SQUARE: //  Stop the tt motor
			while(ps4.button == SQUARE);
			StopBDC(&BDC7);
			StopBDC(&BDC8);
			break;
		case PS:
			while(ps4.button == PS);
			HAL_NVIC_SystemReset();
			RNSStop(&rns);
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
	sensor receive;

	while(1)
	{
		queue1 = osMessageQueueGet(sensor_QueueHandle, &receive, NULL, osWaitForever);
		sema1 = osSemaphoreAcquire(Data_CountSemphrHandle, osWaitForever);
		if(feeding_flag && tt_flag)
		{
			osDelay(200);
			tt_flag = 0;
			feeding_flag = 0;
			start_feed = 1;
			pid_flag = 0;
			StopBDC(&BDC7);
		}
		if(ps4.button == TOUCH && start_feed)
		{
			while(ps4.button == TOUCH);
			ServoSetAngle(&servo_red,100);
			osDelay(500);
			start_feed = 0;
			pid_flag = 0;
			StopBDC(&BDC8);
		}
		else
		{
			ServoSetAngle(&servo_red,0);
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
		queue2 = osMessageQueueGet(sensor_QueueHandle, &receive, NULL, osWaitForever);
		sema2 = osSemaphoreAcquire(Data_CountSemphrHandle, osWaitForever);
		switch(ps4.button)
		{
		case L1: // Open Servo
			while(ps4.button == L1);
			ServoSetAngle(&servo_blk_1,10);
			ServoSetAngle(&servo_blk_2,75);
			WriteBDC(&BDC7,750);
			yaw_offset = ((receive_pos.current_x - (CAM_WIDTH/2.0))*FOV/CAM_WIDTH);
			previous_angle =  receive.current_yaw;
			pid_flag = 1;
			tt_flag = 0;
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
			WriteBDC(&BDC7,1550);
			WriteBDC(&BDC8,1550);
		}
		/* start automation */
		if(pid_flag)
		{

			Err_angle = fmod(yaw_offset - receive.current_yaw -180 + 540, 360) - 180;
			wr = F_angle;
			MODN(&Modn);
//			RNSVelocity(v1,v2,v3,v4,&rns);
			RNSPDC(MAX_PWM*v1,MAX_PWM*v2,MAX_PWM*v3,MAX_PWM*v4,&rns);

		}
		osDelay(17);
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
			ret = HAL_UART_Receive(&huart3, (uint8_t*)py_buffer, 14,50);
			sscanf(py_buffer, "%f,%f", &norm_cx, &norm_cy);
			sender.current_yaw =  rns.RNS_data.common_buffer[0].data - 180;
			sender.current_x_axis= rns.RNS_data.common_buffer[1].data;
			sender.current_y_axis= rns.RNS_data.common_buffer[2].data;
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

		char response[80];
		snprintf(response, sizeof(response), "x: %.3f y: %.3f a: %.3f \n", sender.current_x_axis, sender.current_y_axis, sender.current_yaw);
		UARTPrintString(&huart2, response);


//		char python[80];
//		snprintf(python, sizeof(python), "Received X: %d, Y: %d\n", norm_cx, norm_cy);
		receive_pos.current_x =  norm_cx;
		receive_pos.current_y =  norm_cy;
		osDelay(30);
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
