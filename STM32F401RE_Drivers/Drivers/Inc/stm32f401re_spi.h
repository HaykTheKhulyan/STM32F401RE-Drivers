/*
 * stm32f401re_spi.h
 *
 *  Created on: Nov 7, 2024
 *      Author: Hayk Khulyan
 */

#ifndef INC_STM32F401RE_SPI_H_
#define INC_STM32F401RE_SPI_H_

#include "stm32f401re.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						           SPI Structs
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Configuration structure for SPIx peripheral
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SCLKSpeed;
	uint8_t SPI_DataFrameFormat;
	uint8_t SPI_ClockPolarity;
	uint8_t SPI_ClockPhase;
	uint8_t SPI_SoftwareSlaveManagement;

} SPI_Config_t;

// handle structure for SPIx peripheral
typedef struct
{
	SPI_RegDef_t *SPIx_ptr;		// base address of SPIx(x = 1, 2, 3, 4) peripheral
	SPI_Config_t SPI_Config;

} SPI_Handle_t;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						           SPI APIs
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// peripheral clock setup
void SPI_PCLK_Control(SPI_RegDef_t *SPIx_ptr,
					   uint8_t en);

// init and de-init
void SPI_Init(SPI_Handle_t *SPI_Handle_ptr);
void SPI_DeInit(SPI_RegDef_t *SPIx_ptr);

// data send and receive
void SPI_SendData(SPI_RegDef_t *SPIx_ptr,
				  uint8_t *TxBuffer_ptr,
				  uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *SPIx_ptr,
					 uint8_t *RxBuffer_ptr,
					 uint32_t len);

// IRQ configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber,
							 uint8_t en);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,
							uint8_t IRQ_Priority);
void SPI_IRQHandling(SPI_Handle_t *SPI_Handle_ptr);

// other peripheral control functions


#endif /* INC_STM32F401RE_SPI_H_ */

















