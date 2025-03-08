/* Mini Game 2025 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */

#define MAX_V 4.0
#define POINTS 1
#define MAX_PWM 4000
#define Desired_Dis1 0.5
#define Desired_Dis2 0.4


/* test wheel velocity */
float vel1,vel2,vel3,vel4;

/* ps4 variable */
int analog = 0;
int vel = 0;

/* perspective control variable and path planning */
float set_yaw, yaw_offset, yaw_angle, x_offset, y_offset;
int pers_ctl_flag, imu_lock= 0;

int feeding_flag = 0;




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
osStatus_t ret;
osPriority_t priority;

/* struct for queue data btw two tasks */
typedef struct{

	float current_yaw;
	float current_x_axis;
	float current_y_axis;
	float current_tt_lower_pos;
	float current_tt_upper_pos;
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
	ServoSetAngle(&servo_blk_1,0);
	ServoSetAngle(&servo_blk_2,0);
	ServoSetAngle(&servo_red,135);
	/* Definition of tasks */
	const osThreadAttr_t Motion_Task_attributes = {
			.name = "MotionTask",
			.stack_size = 512 *  4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t Feeding_Task_attributes = {
			.name = "FeedingTask",
			.stack_size = 512 *  4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t Sensor_Task_attributes = {
			.name = "SensorTask",
			.stack_size = 512 *  4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t RetriveBall_Task_attributes = {
			.name = "RetriveBall",
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
	const osMutexAttr_t Sensor_Mutex_attributes = {
				.name = "sensorMutex"
	};

	/* Kernel init */
	osKernelInitialize();

	/*  create Counting Semaphore */
	Data_CountSemphrHandle = osSemaphoreNew(2, 0, &Data_CountSemphr_attributes);
	/*  create Queue = send data btw tasks */
	sensor_QueueHandle = osMessageQueueNew (10, sizeof(sensor), &sensor_Queue_attributes);
	/*  create Queue = send data btw tasks */
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


	while(1)
	{
//		PIDDelayInit (&ltt);
//		PIDDelayInit (&utt);
			/* Process data */
//		HAL_NVIC_SystemReset();

		osDelay(15);
	}
}

void Feeding(void *argument)
{
	/*
	 * 1) When the analog sensor is detected R2 with a safety distance,
	 *    the upper timing belt will rotates and feed the ball
	 *
	 */
	/* Receive Data by Queue */
	sensor receive1;

	while(1)
	{
		osMessageQueueGet(sensor_QueueHandle, &receive1, NULL, osWaitForever);
		osSemaphoreAcquire(Data_CountSemphrHandle, osWaitForever);
		if(feeding_flag && (ps4.button == TOUCH)) // to prevent the analog sensor misdetect the object
		{
			while(ps4.button == TOUCH);
			// start upper tt
			while(receive1.current_tt_upper_pos <= Desired_Dis2)
			{
				// either 1
				Err_u = Desired_Dis2 - receive1.current_tt_upper_pos;
				WriteBDC(&BDC7,20000*F_u);
				WriteBDC(&BDC8,20000*F_u);
				osDelay(10);
			}
			feeding_flag = 0;
			QEIReset(QEI1); // LOWER TT
			QEIReset(QEI4); // UPPER TT

			QEIWrite(QEI1, MIN_POSCNT);
			QEIWrite(QEI4, MIN_POSCNT);
		}
		else if(!feeding_flag && ps4.button == TOUCH)
		{
			while(!feeding_flag)
			{
//				RNSVelocity(); //slightly move forward until detect R2A
				osDelay(10);
			}
			RNSStop(&rns);
			led2 = 1;
		}
		if(ps4.button == SQUARE)
		{
			ServoSetAngle(&servo_red,0);
			while(ps4.button == SQUARE);
		}
		else
		{
			ServoSetAngle(&servo_red,135);
			feeding_flag = 0;
		}
		osDelay(20);
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
			sender.current_yaw =  rns.RNS_data.common_buffer[0].data;
			sender.current_x_axis= rns.RNS_data.common_buffer[1].data;
			sender.current_y_axis= rns.RNS_data.common_buffer[2].data;
			osMutexRelease(sensorMutex);
		}
		// calculate the pulse of the tt motor
		tt_lowerEncData = 0.05 / 4000 * 3.142 * (QEIRead(QEI1) - MIN_POSCNT);
		tt_upperEncData = 0.05 / 4000 * 3.142 * (QEIRead(QEI4) - MIN_POSCNT);

		ABT(&tt_lower_data);
		ABT(&tt_upper_data);

		sender.current_tt_lower_pos = tt_lowerPos;
		sender.current_tt_upper_pos = tt_upperPos;

		if(Sensor_P == 0)
		{
			feeding_flag = 1;
		}
		else
		{
			feeding_flag = 0;
		}

		if (osMessageQueuePut(sensor_QueueHandle, &sender, 0,osWaitForever) == osOK) {
			osSemaphoreRelease(Data_CountSemphrHandle);
		}
		char response[80];
		snprintf(response, sizeof(response), "x: %.3f y: %.3f a: %.3f ttl: %.3f ttu: %.3f\n", sender.current_x_axis, sender.current_y_axis, sender.current_yaw, sender.current_tt_lower_pos, sender.current_tt_upper_pos);
		UARTPrintString(&huart2, response);
		osDelay(25);
	}
}

void Retrivesball(void *argument)
{
	/*
	 *  1) press one button, servo retrive the ball
	 *  2) when servo is close, the timing belt start rotates
	 *  3) until the ball reached middle of the ball
	 */
	uint32_t lower_tt_flag = 0;
	sensor receive2;
	while(1)
	{
		osMessageQueueGet(sensor_QueueHandle, &receive2, NULL, osWaitForever);
		osSemaphoreAcquire(Data_CountSemphrHandle, osWaitForever);
		switch(ps4.button)
		{
		case L1: // Open Servo
			while(ps4.button == L1);
			lower_tt_flag = 0;
			ServoSetAngle(&servo_blk_1,135);
			ServoSetAngle(&servo_blk_2,135);
			lower_tt_flag = 0;
			break;
		case R1: // Close Servo
			while(ps4.button == R1);
			lower_tt_flag = 0;
			ServoSetAngle(&servo_blk_1,0);
			ServoSetAngle(&servo_blk_2,0);
			osDelay(50);
			lower_tt_flag = 1;
			break;
		}

		if(lower_tt_flag)
		{
			// start lower tt , lift the ball until half
			while(receive2.current_tt_lower_pos <= Desired_Dis1)
			{
				Err_l = Desired_Dis1 - receive2.current_tt_lower_pos;
				// either 1
				WriteBDC(&BDC7,20000*F_u);
				WriteBDC(&BDC8,20000*F_u);
				osDelay(10);
			}
			lower_tt_flag = 0;
			QEIReset(QEI1); // LOWER TT
			QEIReset(QEI4); // UPPER TT

			QEIWrite(QEI1, MIN_POSCNT);
			QEIWrite(QEI4, MIN_POSCNT);
		}
	}
}

void TIM6_DAC_IRQHandler(void)
{

	ret = osSemaphoreRelease(Data_CountSemphrHandle);
	led1=!led1;
	PSxConnectionHandler(&ps4);
	PID(&ltt);
	PID(&utt);
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
