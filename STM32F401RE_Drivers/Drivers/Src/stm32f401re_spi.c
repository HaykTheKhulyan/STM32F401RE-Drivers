/*
 * stm32f401re_spi.c
 *
 *  Created on: Nov 7, 2024
 *      Author: Hayk Khulyan
 */

#include "stm32f401re_spi.h"

// peripheral clock setup

/*******************************************************************************
 * @fn 			- SPI Peripheral Clock Control
 *
 * @brief		- Enable or disable the peripheral clock for the
 * 				  given SPIx peripheral
 *
 * @param		- base address of SPIx peripheral
 * @param 		- ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void SPI_PCLK_Control(SPI_RegDef_t *SPIx_ptr,
					   uint8_t en)
{
	if (en)
	{
		if (SPIx_ptr == SPI1) SPI1_PCLK_EN();
		if (SPIx_ptr == SPI2) SPI2_PCLK_EN();
		if (SPIx_ptr == SPI3) SPI3_PCLK_EN();
		if (SPIx_ptr == SPI4) SPI4_PCLK_EN();
	}
	else {
		if (SPIx_ptr == SPI1) SPI1_PCLK_DI();
		if (SPIx_ptr == SPI2) SPI2_PCLK_DI();
		if (SPIx_ptr == SPI3) SPI3_PCLK_DI();
		if (SPIx_ptr == SPI4) SPI4_PCLK_DI();
	}
}

// init and de-init

/*******************************************************************************
 * @fn 			- GPIO Peripheral Clock Control
 *
 * @brief		- Enable or disable the peripheral clock for the given GPIO port
 *
 * @param		- base address of GPIO port
 * @param 		- ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void SPI_Init(SPI_Handle_t *SPI_Handle_ptr)
{

}

/*******************************************************************************
 * @fn 			- GPIO Peripheral Clock Control
 *
 * @brief		- Enable or disable the peripheral clock for the given GPIO port
 *
 * @param		- base address of GPIO port
 * @param 		- ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void SPI_DeInit(SPI_RegDef_t *SPIx_ptr)
{

}

// data send and receive

/*******************************************************************************
 * @fn 			- GPIO Peripheral Clock Control
 *
 * @brief		- Enable or disable the peripheral clock for the given GPIO port
 *
 * @param		- base address of GPIO port
 * @param 		- ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void SPI_SendData(SPI_RegDef_t *SPIx_ptr,
				  uint8_t *TxBuffer_ptr,
				  uint32_t len)
{

}

/*******************************************************************************
 * @fn 			- GPIO Peripheral Clock Control
 *
 * @brief		- Enable or disable the peripheral clock for the given GPIO port
 *
 * @param		- base address of GPIO port
 * @param 		- ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *SPIx_ptr,
					 uint8_t *RxBuffer_ptr,
					 uint32_t len)
{

}

// IRQ configuration and ISR handling

/*******************************************************************************
 * @fn 			- GPIO Peripheral Clock Control
 *
 * @brief		- Enable or disable the peripheral clock for the given GPIO port
 *
 * @param		- base address of GPIO port
 * @param 		- ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber,
							 uint8_t en)
{

}

/*******************************************************************************
 * @fn 			- GPIO Peripheral Clock Control
 *
 * @brief		- Enable or disable the peripheral clock for the given GPIO port
 *
 * @param		- base address of GPIO port
 * @param 		- ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber,
							uint8_t IRQ_Priority)
{

}

/*******************************************************************************
 * @fn 			- GPIO Peripheral Clock Control
 *
 * @brief		- Enable or disable the peripheral clock for the given GPIO port
 *
 * @param		- base address of GPIO port
 * @param 		- ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void SPI_IRQHandling(SPI_Handle_t *SPI_Handle_ptr)
{

}

// other peripheral control functions



















