/*
 * stm32f401re_adc.c
 *
 *  Created on: Nov 21, 2024
 *      Author: Hayk Khulyan
 */

#include "stm32f401re_adc.h"

// ADC powered on by setting the ADON bit in the ADC_CR2 register
// conversion starts when either the SWSTART or JSWSTART bit is set
// select regular channels in the ADC_SQRx register
// write total number of conversions in the L[3:0] bits in ADCSQR1 register
/* in continuous conversion mode:
 * 		CONT bit at 1
 * 		after each conversion of a regular group of channels:
 * 			the last converted data are stored in the ADC_DR register
 * 			EOC (end of conversion) flag is set
 * 			interrupt is generated if the EOCIE bit is set
 * 		to perform faster conversion, reduce the ADC resolution through RES bits
 * 		input voltage is sampled for a number of ADCCLK cycles determined by
 * 			the SMP[2:0] bits in the ADC_SMPR1 and ADC_SMPR2 registers
*/

/*******************************************************************************
 * @fn 			- ADC Get Flag Status
 *
 * @brief		- returns the status of an ADC status flag
 *
 * @param		- base address of ADC peripheral
 * @param 		- the bit position of the flag in the ADC status register
 *
 * @return		- 0 if flag is not set, 1 if flag is set
 *
 * @note		- none
 *
 ******************************************************************************/
uint8_t ADC_GetFlagStatus(ADC_RegDef_t *ADC_ptr, uint32_t flag)
{
	if (ADC_ptr->SR & flag) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*******************************************************************************
 * @fn 			- ADC Peripheral Clock Control
 *
 * @brief		- Enable or disable the peripheral clock for the
 * 				  given ADC peripheral
 *
 * @param		- base address of ADC peripheral
 * @param 		- ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void ADC_PCLK_Control(ADC_RegDef_t *ADC_ptr,
					  uint8_t en) {
	if (en) ADC_PCLK_EN();
	else 	ADC_PCLK_DI();
}

/*******************************************************************************
 * @fn 			- ADC Initialize
 *
 * @brief		- Initialize the given ADC peripheral
 *
 * @param		- ADC handler structure
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void ADC_Init(ADC_Handle_t *ADC_Handle_ptr) {
	/*
	 * typedef struct
		{
			uint8_t ADC_ConversionMode;
			uint8_t ADC_Resolution;
			uint8_t ADC_Prescaler;

		} ADC_Config_t;
	 */

	// enable the clock for the ADC peripheral
	ADC_PCLK_Control(ADC_Handle_ptr->ADC_ptr, ENABLE);


	uint32_t CR2_temp_reg = 0;

	// configure continuous bit
	CR2_temp_reg |= (ADC_Handle_ptr->
					 ADC_Config.
					 ADC_ConversionMode << ADC_CR2_CONT);

	ADC_Handle_ptr->ADC_ptr->CR2 |= CR2_temp_reg;


	uint32_t CR1_temp_reg = 0;

	// configure resolution bits
	CR1_temp_reg |= (ADC_Handle_ptr->
					 ADC_Config.
					 ADC_Resolution << ADC_CR1_RES);

	ADC_Handle_ptr->ADC_ptr->CR1 |= CR1_temp_reg;


	uint32_t CCR_temp_reg = 0;

	// configure prescaler
	CCR_temp_reg |= (ADC_Handle_ptr->
					 ADC_Config.
					 ADC_Prescaler << ADC_CCR_ADCPRE);

	ADC_Handle_ptr->ADC_ptr->CCR |= CCR_temp_reg;

	// configure regular channel sequence length
//	uint32_t SQR1_temp_reg = 0;

//	SQR1_temp_reg |= (ADC_Handle_ptr->
//					  ADC_Config.
//					  ADC_RegularChannelSequenceLength << ADC_SQR1_L);

	// for now, hard code channel 0 to be first in the sequence
	// later, fix this driver so the user can select a custom regular sequence
	ADC_Handle_ptr->ADC_ptr->SQR[0] |= (1 << ADC_SQR1_L);

	// configure 1st channel in regular sequence
//	uint32_t SQR3_temp_reg = 0;
//
//	SQR3_temp_reg |= (0 << ADC_SQR3_SQ1);

	ADC_Handle_ptr->ADC_ptr->SQR[2] |= (0 << ADC_SQR3_SQ1);



}

/*******************************************************************************
 * @fn 			- ADC Deinitialization
 *
 * @brief		- Deinitialize the ADC peripheral through RCC reset register
 *
 * @param		- base address of ADC peripheral
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void ADC_DeInit(ADC_RegDef_t *ADC_ptr) {
	RCC->APB2RSTR |=  (1 << 8);
	RCC->APB2RSTR &= ~(1 << 8);
}

/*******************************************************************************
 * @fn 			- ADC Read
 *
 * @brief		- Reads a conversion (once available) from the ADC data register
 *
 * @param		- base address of ADC peripheral
 *
 * @return		- 16 bit read value from input register
 *
 * @note		- return type is 16 bit int, but data format will be one of the
 * 				  ADC resolution lengths (12, 10, 8, or 6 bits)
 * 				- this is a blocking call, since we wait for the EOC (end of
 * 				  conversion) bit to be set after a conversion has been done
 *
 ******************************************************************************/
uint16_t ADC_Read(ADC_RegDef_t *ADC_ptr) {
	while (ADC_GetFlagStatus(ADC_ptr, ADC_SR_EOC));

	uint16_t read_value = ADC_ptr->DR;

	return read_value;
}

/*******************************************************************************
 * @fn 			- ADC interrupt config
 *
 * @brief		- Sets the proper interrupt register values in the ARM Cortex M4
 * 				  processor
 *
 * @param		- The IRQ number we wish to configure
 * @param 		- ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void ADC_IRQInterruptConfig(uint8_t IRQNumber,
							 uint8_t en) {

}

/*******************************************************************************
 * @fn 			- ADC interrupt priority config
 *
 * @brief		- Sets the proper priority register values in the ARM Cortex M4
 * 				  processor
 *
 * @param		- The IRQ number we wish to configure
 * @param 		- The priority of the interrupt
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void ADC_IRQPriorityConfig(uint8_t IRQNumber,
							uint8_t IRQ_Priority) {

}

/*******************************************************************************
 * @fn 			- ADC interrupt request handling
 *
 * @brief		- Handles the ADC interrupt request by
 * 				  clearing EXTI PR register
 *
 * @param		- pin number of the interrupt we want to clear
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void ADC_IRQHandling(ADC_Handle_t *ADC_Handle_ptr) {

}

/*******************************************************************************
 * @fn 			- ADC Peripheral Control
 *
 * @brief		- Enable or disable the ADC and start conversion
 *
 * @param		- base address of ADC peripheral
 * @param 		- ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void ADC_PeripheralControl(ADC_RegDef_t *ADC_ptr, uint8_t en) {
	// ADON bit and SWSTART bit
	ADC_ptr->CR2 |= (1 << ADC_CR2_ADON);
	ADC_ptr->CR2 |= (1 << ADC_CR2_SWSTART);
}











