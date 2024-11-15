/*
 * stm32f401re_spi.c
 *
 *  Created on: Nov 7, 2024
 *      Author: Hayk Khulyan
 */

#include "stm32f401re_spi.h"

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *SPIx_ptr, uint32_t flag)
{
	if (SPIx_ptr->SR & flag) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

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
 * @fn 			- SPI Initialize
 *
 * @brief		- Enable or disable the peripheral clock for the given GPIO port
 *
 * @param		- pointer to SPI Handle struct
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void SPI_Init(SPI_Handle_t *SPI_Handle_ptr)
{
	/*
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SClkSpeed;
	uint8_t SPI_DataFrameFormat;
	uint8_t SPI_ClockPolarity;
	uint8_t SPI_ClockPhase;
	uint8_t SPI_SoftwareSlaveManagement;
	 */

	// temporary register where we will set the appropriate config bits
	// will later be copied into the SPI config register
	uint32_t temp_reg = 0;

	// SPI_DeviceMode config
	temp_reg |= SPI_Handle_ptr->SPI_Config.SPI_DeviceMode << 2;

	// SPI_BusConfig config
	if (SPI_Handle_ptr->
		SPI_Config.
		SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		temp_reg &= ~(1 << 15);
	}
	else if (SPI_Handle_ptr->
			 SPI_Config.
			 SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		temp_reg |= (1 << 15);
	}
	else if (SPI_Handle_ptr->
			 SPI_Config.
			 SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		temp_reg &= ~(1 << 15);
		temp_reg |=  (1 << 10);
	}

	// SPI_SClkSpeed config
	temp_reg |= (SPI_Handle_ptr->SPI_Config.SPI_SClkSpeed << 3);

	// SPI_DataFrameFormat config
	temp_reg |= (SPI_Handle_ptr->SPI_Config.SPI_DataFrameFormat << 11);

	// SPI_ClockPolarity config
	temp_reg |= (SPI_Handle_ptr->SPI_Config.SPI_ClockPolarity << 1);

	// SPI_ClockPhase config
	temp_reg |= (SPI_Handle_ptr->SPI_Config.SPI_ClockPhase << 0);

	SPI_Handle_ptr->SPIx_ptr->CR1 = temp_reg;
}

/*******************************************************************************
 * @fn 			- SPI Deinitialize
 *
 * @brief		- De-init the SPI peripheral through the RCC register
 *
 * @param		- base address of SPI device
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void SPI_DeInit(SPI_RegDef_t *SPIx_ptr)
{
	if (SPIx_ptr == SPI1) {
		RCC->APB2RSTR |=  (1 << 12);
		RCC->APB2RSTR &= ~(1 << 12);
	}
	else if (SPIx_ptr == SPI2) {
		RCC->APB1RSTR |=  (1 << 14);
		RCC->APB2RSTR &= ~(1 << 14);
	}
	if (SPIx_ptr == SPI3) {
		RCC->APB1RSTR |=  (1 << 15);
		RCC->APB1RSTR &= ~(1 << 15);
	}
	if (SPIx_ptr == SPI4) {
		RCC->APB2RSTR |=  (1 << 13);
		RCC->APB2RSTR &= ~(1 << 13);
	}
}

// data send and receive

/*******************************************************************************
 * @fn 			- SPI Send Data
 *
 * @brief		- Send data along the SPI peripheral
 *
 * @param		- base address of SPI peripheral
 * @param 		- pointer to data to send
 * @param		- length of data to send (in bytes)
 *
 * @return		- none
 *
 * @note		- This is a blocking call
 *
 ******************************************************************************/
void SPI_SendData(SPI_RegDef_t *SPIx_ptr,
				  uint8_t *TxBuffer_ptr,
				  uint32_t len)
{
	// if length is greater than zero, we still have data to send
	while (len > 0)
	{
		// wait until the transmit buffer is empty
		uint8_t Tx_buffer_empty = SPI_GetFlagStatus(SPIx_ptr, SPI_TXE_FLAG);

		if (Tx_buffer_empty)
		{
			// check data frame format
			uint8_t DFF = (SPIx_ptr->CR1 & (1 << SPI_CR1_DFF));

			if (DFF == SPI_DFF_8BITS)
			{
				// read 1 byte of data from data to send
				uint8_t data_to_send = (*TxBuffer_ptr);

				// load the data into the SPI data register
				SPIx_ptr->DR = data_to_send;

				// increment TxBuffer_ptr so we can read the
				// following byte of data
				TxBuffer_ptr++;
				// decrement length (since there's one less byte of
				// data to send)
				len--;
			}
			else if (DFF == SPI_DFF_16BITS)
			{
				// read 2 bytes of data from data to send
				// cast TxBuffer_ptr to a 16 bit pointer, then dereference it
				uint16_t data_to_send = *((uint16_t*)TxBuffer_ptr);

				// load the data into the SPI data register
				SPIx_ptr->DR = data_to_send;

				// increment TxBuffer_ptr twice (since we
				// sent two bytes of data)
				TxBuffer_ptr++;
				TxBuffer_ptr++;
				// decrement length twice (since there's two less bytes of
				// data to send
				len--;
				len--;
			}
		}
	}
}

/*******************************************************************************
 * @fn 			- GPIO Peripheral Clock Control
 *
 * @brief		- Enable or disable the peripheral clock for the given GPIO port
 *
 * @param		- base address of SPI device
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



















