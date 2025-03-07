/************************************************
 * Title   : PS3_INTERFACE
 * Author  : Kai Sheng, Qiu Hui,Leonard Chin, and Anas Amer
 * Version : 1.1
 * Date    : 29 JUNE 2017
 * **********************************************
 * Descriptions:
 * - used to interface with PS3 and PS4 through Arduino via I2C or UART
 *
 * Version History:
 * 1.0 - used to interface with PS3 through Arduino via I2C or UART
 *
 * 1.1 - converted to HAL library
 * Bugs:
 *
 ************************************************/

/***************************************
 * 		Include Libraries 			   *
 **************************************/
#ifndef PSX_INTERFACE_PSX_INTERFACE_H_
#define PSX_INTERFACE_PSX_INTERFACE_H_

#include "../BIOS/bios.h"
#include "../I2C/i2c.h"

/************************************************/
/*		    	Define         					*/
/************************************************/

#define 	PSxBTMAXLEN					40

#define 	psx_low_Lx					115.0
#define 	psx_high_Lx					140.0

#define 	psx_low_Ly					115.0
#define 	psx_high_Ly					140.0

#define 	psx_low_Rx					115.0
#define 	psx_high_Rx					140.0

#define 	psx_low_Ry					115.0
#define 	psx_high_Ry					140.0

#define 	joyL_up						(float)psxbt->leftjoy_y < psx_low_Ly
#define 	joyL_down					(float)psxbt->leftjoy_y > psx_high_Ly
#define 	joyL_left					(float)psxbt->leftjoy_x < psx_low_Lx
#define 	joyL_right					(float)psxbt->leftjoy_x > psx_high_Lx

#define 	joyR_up						(float)psxbt->rightjoy_y < psx_low_Ry
#define 	joyR_down					(float)psxbt->rightjoy_y > psx_high_Ry
#define 	joyR_left					(float)psxbt->rightjoy_x < psx_low_Rx
#define 	joyR_right					(float)psxbt->rightjoy_x > psx_high_Rx

#define SHARE	 0x01 //select button in PS3
#define	L3		 0x02 //press L joystick
#define R3		 0x04 //press R joystick
#define START1	 0x08 //no more in PS4
#define UP		 0x10
#define RIGHT	 0x20
#define DOWN	 0x40
#define LEFT	 0x80
#define L2		 0x0100
#define R2		 0x0200
#define L1  	 0x0400
#define R1 		 0x0800
#define TRIANGLE 0x1000
#define CIRCLE 	 0x2000
#define CROSS 	 0x4000
#define SQUARE 	 0x8000
#define OPTION   0x010000
#define TOUCH	 0x020000 //PS4
#define PS		 0x040000 //PS4


#define UP_SQUARE    0x8010
#define UP_CIRCLE    0x2010
#define UP_TRIANGLE  0x1010
#define UP_CROSS     0x4010

#define DOWN_SQUARE    0x8040
#define DOWN_CIRCLE    0x2040
#define DOWN_TRIANGLE  0x1040
#define DOWN_CROSS     0x4040

#define LEFT_SQUARE    0x8080
#define LEFT_CIRCLE    0x2080
#define LEFT_TRIANGLE  0x1080
#define LEFT_CROSS     0x4080

#define RIGHT_SQUARE    0x8020
#define RIGHT_CIRCLE    0x2020
#define RIGHT_TRIANGLE  0x1020
#define RIGHT_CROSS     0x4020

#define L1_UP			0x0410
#define R1_UP			0x0810
#define L1_RIGHT		0x0420
#define R1_RIGHT		0x0820
#define L1_DOWN			0x0440
#define R1_DOWN			0x0840
#define L1_LEFT			0x0480
#define R1_LEFT			0x0880
#define L1_TRIANGLE		0x1400
#define R1_TRIANGLE		0x1800
#define L1_CIRCLE       0x2400
#define R1_CIRCLE       0x2800
#define L1_CROSS        0x4400
#define R1_CROSS        0x4800
#define L1_SQUARE		0x8400
#define R1_SQUARE		0x8800
#define L1_R1           0x0C00

/************************************************/
/*				Struct         					*/
/************************************************/
typedef enum{
	PS3 = 3,
	PS4
}PSx;

typedef struct{

	uint8_t slave;
	uint8_t master;
	char ReceiveBuffer[PSxBTMAXLEN];
	volatile unsigned char x;
    volatile int flag;
    PSx psx1;

	volatile unsigned int leftjoy_x;
	volatile unsigned int leftjoy_y;
	volatile unsigned int rightjoy_x;
	volatile unsigned int rightjoy_y;

	volatile unsigned int an_L2;
	volatile unsigned int an_R2;

	volatile float joyR_y;
	volatile float joyR_x;
	volatile float joyL_y;
	volatile float joyL_x;
	volatile float joyR_2;
	volatile float joyL_2;

	I2C_HandleTypeDef* hi2cps4;
	DMA_HandleTypeDef* hi2cps4_rx_dma;
	UART_HandleTypeDef* huartps4;

	union{
		volatile unsigned int button;
		struct{
			char buf1;
			char buf2;
			char buf3;
			char buf4;
		};
	};
	volatile char state;
	uint8_t disconnected;
}PSxBT_t;

/**************************************************
 * 		STRUCTURE DEFINES					  	  *
 *************************************************/

/**************************************************
 * 		Function Prototype					  	  *
 *************************************************/

void PSxMasterInit(PSxBT_t *psxbt,I2C_HandleTypeDef* hi2cx,DMA_HandleTypeDef* hi2cx_rx_dma);
void PSxSlaveInit(PSxBT_t *psxbt,I2C_HandleTypeDef* hi2cx);
void PSx_MasterHandler(PSxBT_t *psxbt);
void PSx_SlaveHandler(PSxBT_t *psxbt);
void PSxConnectionHandler(PSxBT_t *psxbt);
void PSxBTGetXY(PSxBT_t *psxbt);

#endif /* PSX_INTERFACE_PSX_INTERFACE_H_ */
