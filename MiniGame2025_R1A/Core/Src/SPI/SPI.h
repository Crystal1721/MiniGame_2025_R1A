/*******************************************************************************
 * Title   : SPI.h
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 12/10
 *******************************************************************************
 * Description: Send and receive Data using SPI
 *
 * Version History: Haven't tested
 *
 * Bugs:
 *
 ******************************************************************************/


#ifndef IMU_SPI_H_
#define IMU_SPI_H_

/***************************************
 * 		Include Libraries 			   *
 **************************************/

#include "../BIOS/bios.h"

/***************************************
 * 		DEFINES						   *
 ***************************************/

#define SPI_FLAG_TIMEOUT	((uint32_t)0x1000)
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
/***************************************
 * 		STRUCTURE					   *
 ***************************************/

/**************************************************
 * 		Function Prototype			  			  *
 *************************************************/

void SPIxInit(SPI_HandleTypeDef* hspix, GPIO_TypeDef* GPIOx_NSS, uint16_t GPIO_Pin_NSS,uint32_t Mode, int InterruptEnable);
void SPITransmit(SPI_HandleTypeDef* hspix, uint8_t *data);
void SPIReceive(SPI_HandleTypeDef* hspix, uint8_t *data);
void SPI_Transmit_Receive(SPI_HandleTypeDef* hspix, uint8_t *txdata, uint8_t *rxdata);
#endif /* IMU_SPI_H_ */
