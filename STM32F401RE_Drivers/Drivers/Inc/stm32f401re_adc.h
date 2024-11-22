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
#define ADC_RESOLUTION_12BIT			0
#define ADC_RESOLUTION_10BIT			1
#define ADC_RESOLUTION_8BIT				2
#define ADC_RESOLUTION_6BIT				3

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

} ADC_Config_t;

// handle structure for ADC peripheral
typedef struct
{
	ADC_RegDef_t *ADC_ptr;		// base address of ADC peripheral
	ADC_Config_t ADC_Config;

} ADC_Handle_t;

#endif /* INC_STM32F401RE_ADC_H_ */
