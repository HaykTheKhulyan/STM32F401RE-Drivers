/*
 * stm32f401re_gpio.h
 *
 *  Created on: Oct 23, 2024
 *      Author: haykkhulyan
 */

#ifndef INC_STM32F401RE_GPIO_H_
#define INC_STM32F401RE_GPIO_H_

#include "stm32f401re.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						           GPIO Macros
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// GPIO pin numbers
#define GPIO_PIN_0			0		// pin 0
#define GPIO_PIN_1			1		// pin 1
#define GPIO_PIN_2			2		// .
#define GPIO_PIN_3			3		// .
#define GPIO_PIN_4			4		// .
#define GPIO_PIN_5			5		// .
#define GPIO_PIN_6			6		// .
#define GPIO_PIN_7			7		// .
#define GPIO_PIN_8			8		// .
#define GPIO_PIN_9			9		// .
#define GPIO_PIN_10			10		// .
#define GPIO_PIN_11			11		// .
#define GPIO_PIN_12			12		// .
#define GPIO_PIN_13			13		// .
#define GPIO_PIN_14			14		// .
#define GPIO_PIN_15			15		// pin 15

// GPIO pin modes
#define GPIO_MODE_INPUT 	0		// input mode
#define GPIO_MODE_OUTPUT 	1		// output mode
#define GPIO_MODE_ALTFUNC 	2		// alternate function mode
#define GPIO_MODE_ANALOG 	3		// analog mode

// GPIO interrupt modes
#define GPIO_MODE_IT_FT		4		// falling edge trigger
#define GPIO_MODE_IT_RT		5		// rising edge trigger
#define GPIO_MODE_IT_RFT	6		// rising and falling edge trigger

// GPIO output types
#define GPIO_OT_PP			0		// output push-pull
#define GPIO_OT_OD			1		// output open-drain

// GPIO output speeds
#define GPIO_OS_LOW			0		// output speed low
#define GPIO_OS_MED			1		// output speed medium
#define GPIO_OS_HIGH		2		// output speed high
#define GPIO_OS_VHIGH		3		// output speed very high

// GPIO internal pull-up/pull-down resistors
#define GPIO_PUPD_NONE		0		// no pull-up/pull-down resistors
#define GPIO_PUPD_PU		1		// pull-up
#define GPIO_PUPD_PD		2		// pull-down

// GPIO alternate function mode
#define GPIO_AF0			0		// alternate function 0
#define GPIO_AF1			1		// alternate function 1
#define GPIO_AF2			2		// .
#define GPIO_AF3			3		// .
#define GPIO_AF4			4		// .
#define GPIO_AF5			5		// .
#define GPIO_AF6			6		// .
#define GPIO_AF7			7		// .
#define GPIO_AF8			8		// .
#define GPIO_AF9			9		// .
#define GPIO_AF10			10		// .
#define GPIO_AF11			11		// .
#define GPIO_AF12			12		// .
#define GPIO_AF13			13		// .
#define GPIO_AF14			14		// .
#define GPIO_AF15			15		// alternate function 15


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						           GPIO Structs
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// pin configuration structure for GPIO pin
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinOutputType;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPUPDRControl;
	uint8_t GPIO_PinAlternateFunctionMode;

} GPIO_PinConfig_t;

// handle structure for GPIO pin
typedef struct
{
	GPIO_RegDef_t *GPIOx_ptr;
	GPIO_PinConfig_t GPIO_PinConfig;

} GPIO_Handle_t;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						           GPIO APIs
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// peripheral clock setup
void GPIO_PCLK_Control(GPIO_RegDef_t *GPIOx_ptr,
					   uint8_t en);

// init and de-init
void GPIO_Init(GPIO_Handle_t *GPIO_Handle_ptr);
void GPIO_DeInit(GPIO_RegDef_t *GPIOx_ptr);

// data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *GPIOx_ptr,
							  uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *GPIOx_ptr);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *GPIOx_ptr,
						   uint8_t pinNumber,
						   uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *GPIOx_ptr,
							uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *GPIOx_ptr,
	   	   	    		  uint8_t pinNumber);

// IRQ config and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,
							 uint8_t en);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,
							uint8_t IRQ_Priority);
void GPIO_IRQHandling(uint8_t pinNumber);




#endif /* INC_STM32F401RE_GPIO_H_ */






















