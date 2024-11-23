/*
 * stm32f401re_adc.h
 *
 *  Created on: Nov 21, 2024
 *      Author: Hayk Khulyan
 */

#ifndef INC_STM32F401RE_ADC_H_
#define INC_STM32F401RE_ADC_H_

#include "stm32f401re.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						           ADC Macros
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// @ADC_ConversionMode
#define ADC_CONVERSION_MODE_SINGLE 		0
#define ADC_CONVERSION_MODE_CONTINUOUS	1

// @ADC_Resolution
#define ADC_RESOLUTION_12BIT	0
#define ADC_RESOLUTION_10BIT	1
#define ADC_RESOLUTION_8BIT		2
#define ADC_RESOLUTION_6BIT		3

// ADC_Prescaler
#define ADC_CLK_DIV2	0
#define ADC_CLK_DIV4	1
#define ADC_CLK_DIV6	2
#define ADC_CLK_DIV8	3

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						           ADC Structs
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Configuration structure for ADC peripheral
typedef struct
{
	uint8_t ADC_ConversionMode;
	uint8_t ADC_Resolution;
	uint8_t ADC_Prescaler;

} ADC_Config_t;

// handle structure for ADC peripheral
typedef struct
{
	ADC_RegDef_t *ADC_ptr;		// base address of ADC peripheral
	ADC_Config_t ADC_Config;

} ADC_Handle_t;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						           ADC APIs
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// peripheral clock setup
void ADC_PCLK_Control(ADC_RegDef_t *ADC_ptr,
					  uint8_t en);

// init and de-init
void ADC_Init(ADC_Handle_t *ADC_Handle_ptr);
void ADC_DeInit(ADC_RegDef_t *ADC_ptr);

// data read
uint16_t ADC_Read(ADC_RegDef_t *ADC_ptr);

// IRQ configuration and ISR handling
void ADC_IRQInterruptConfig(uint8_t IRQNumber,
							 uint8_t en);
void ADC_IRQPriorityConfig(uint8_t IRQNumber,
							uint8_t IRQ_Priority);
void ADC_IRQHandling(ADC_Handle_t *ADC_Handle_ptr);

// other peripheral control functions
void ADC_PeripheralControl(ADC_RegDef_t *ADC_ptr, uint8_t en);

#endif /* INC_STM32F401RE_ADC_H_ */














