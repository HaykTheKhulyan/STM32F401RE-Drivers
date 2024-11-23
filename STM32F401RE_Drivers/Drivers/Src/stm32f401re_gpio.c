/*
 * stm32f401re_gpio.c
 *
 *  Created on: Oct 23, 2024
 *      Author: haykkhulyan
 */

#include "stm32f401re_gpio.h"

// peripheral clock setup

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
void GPIO_PCLK_Control(GPIO_RegDef_t *GPIOx_ptr,
					   uint8_t en)
{
	// if `en` is 1, enable the RCC clock of the GPIO pin at the address given
	// otherwise, disable the clock
	if (en)
	{
		if 		(GPIOx_ptr == GPIOA) GPIOA_PCLK_EN();
		else if (GPIOx_ptr == GPIOB) GPIOB_PCLK_EN();
		else if (GPIOx_ptr == GPIOC) GPIOC_PCLK_EN();
		else if (GPIOx_ptr == GPIOD) GPIOD_PCLK_EN();
		else if (GPIOx_ptr == GPIOE) GPIOE_PCLK_EN();
		else if (GPIOx_ptr == GPIOH) GPIOH_PCLK_EN();
	}
	else {
		if 		(GPIOx_ptr == GPIOA) GPIOA_PCLK_DI();
		else if (GPIOx_ptr == GPIOB) GPIOB_PCLK_DI();
		else if (GPIOx_ptr == GPIOC) GPIOC_PCLK_DI();
		else if (GPIOx_ptr == GPIOD) GPIOD_PCLK_DI();
		else if (GPIOx_ptr == GPIOE) GPIOE_PCLK_DI();
		else if (GPIOx_ptr == GPIOH) GPIOH_PCLK_DI();
	}
}

// init and de-init

/*******************************************************************************
 * @fn 			- GPIO Initialize
 *
 * @brief		- Initialize the given GPIO port
 *
 * @param		- GPIO handler structure
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void GPIO_Init(GPIO_Handle_t *GPIO_Handle_ptr)
{
	GPIO_PCLK_Control(GPIO_Handle_ptr->GPIOx_ptr, 1);

	uint32_t temp = 0;
	uint8_t pinNumber = GPIO_Handle_ptr->GPIO_PinConfig.GPIO_PinNumber;

	// configure GPIO mode
	if (GPIO_Handle_ptr->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// non-interrupt mode

		// put the correct bits at the correct bit positions in the temp var
		temp = (GPIO_Handle_ptr->GPIO_PinConfig.GPIO_PinMode << (2 * pinNumber));
		// clear the bits in the relevant pin's bit positions in mode register
		GPIO_Handle_ptr->GPIOx_ptr->MODER &= ~(3 << (2 * pinNumber));
		// set the correct bits
		GPIO_Handle_ptr->GPIOx_ptr->MODER |= temp;
	}
	else
	{
		// interrupt mode

		// clear any existing configuration
		EXTI->FTSR &= ~(1 << pinNumber);
		EXTI->RTSR &= ~(1 << pinNumber);

		if (GPIO_Handle_ptr->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// configure FTSR
			EXTI->FTSR |= (1 << pinNumber);
		}
		else if (GPIO_Handle_ptr->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// configure RTSR
			EXTI->RTSR |= (1 << pinNumber);
		}
		else if (GPIO_Handle_ptr->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pinNumber);
			EXTI->RTSR |= (1 << pinNumber);
		}

		// configure EXTI interrupt mask register
		EXTI->IMR |= (1 << pinNumber);

		SYSCFG_PCLK_EN();

		uint8_t EXTI_index = pinNumber / 4;
		uint8_t shift_amount = pinNumber % 4;

		SYSCFG->EXTICR[EXTI_index] &= ~(15 << shift_amount);

		uint8_t port_code = GPIO_BASEADDR_TO_PORTCODE(GPIO_Handle_ptr->
													  GPIOx_ptr);

		SYSCFG->EXTICR[EXTI_index] |= (port_code << shift_amount);
	}

	if (GPIO_Handle_ptr->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUTPUT)
	{
		// configure the output type
		temp = (GPIO_Handle_ptr->
				GPIO_PinConfig.GPIO_PinOutputType << (1 * pinNumber));
		GPIO_Handle_ptr->GPIOx_ptr->OTYPER &= ~(1 << (1 * pinNumber));
		GPIO_Handle_ptr->GPIOx_ptr->OTYPER |= temp;

		// configure GPIO output speed
		temp = (GPIO_Handle_ptr->
				GPIO_PinConfig.GPIO_PinSpeed << (2 * pinNumber));
		GPIO_Handle_ptr->GPIOx_ptr->OSPEEDR &= ~(3 << (2 * pinNumber));
		GPIO_Handle_ptr->GPIOx_ptr->OSPEEDR |= temp;
	}

	// configure PUPD settings
	temp = (GPIO_Handle_ptr->
			GPIO_PinConfig.GPIO_PinPUPDRControl << (2 * pinNumber));
	GPIO_Handle_ptr->GPIOx_ptr->PUPDR &= ~(3 << (2 * pinNumber));
	GPIO_Handle_ptr->GPIOx_ptr->PUPDR |= temp;

	// configure the alternate functionality if pin is in AF mode
	if (GPIO_Handle_ptr->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUNC)
	{
		// calculate whether to use the AFRL or AFRH registers
		uint8_t alt_function_index = pinNumber / 8;
		// calculate how much to shift the alt function bits
		uint8_t shift_amount = pinNumber % 8;

		temp = (GPIO_Handle_ptr->
				GPIO_PinConfig.
				GPIO_PinAlternateFunctionMode << (4 * shift_amount));

		GPIO_Handle_ptr->
		GPIOx_ptr->
		AFR[alt_function_index] &= ~(15 << (4 * shift_amount));

		GPIO_Handle_ptr->GPIOx_ptr->AFR[alt_function_index] |= temp;
	}
}

/*******************************************************************************
 * @fn 			- GPIO Deinitialization
 *
 * @brief		- Deinitialize the GPIO port through RCC reset register
 *
 * @param		- base address of GPIO port
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *GPIOx_ptr)
{
	if (GPIOx_ptr == GPIOA)
	{
		RCC->AHB1RSTR |= (1 << 0);
		RCC->AHB1RSTR &= ~(1 << 0);
	}
	else if (GPIOx_ptr == GPIOB)
	{
		RCC->AHB1RSTR |= (1 << 1);
		RCC->AHB1RSTR &= ~(1 << 1);
	}
	else if (GPIOx_ptr == GPIOC)
	{
		RCC->AHB1RSTR |= (1 << 2);
		RCC->AHB1RSTR &= ~(1 << 2);
	}
	else if (GPIOx_ptr == GPIOD)
	{
		RCC->AHB1RSTR |= (1 << 3);
		RCC->AHB1RSTR &= ~(1 << 3);
	}
	else if (GPIOx_ptr == GPIOE)
	{
		RCC->AHB1RSTR |= (1 << 4);
		RCC->AHB1RSTR &= ~(1 << 4);
	}
	else if (GPIOx_ptr == GPIOH)
	{
		RCC->AHB1RSTR |= (1 << 7);
		RCC->AHB1RSTR &= ~(1 << 7);
	}
}

// data read and write

/*******************************************************************************
 * @fn 			- GPIO Read From Input Pin
 *
 * @brief		- Reads a single bit from the given GPIO pin
 *
 * @param		- base address of GPIO port
 * @param 		- pin number to read from
 *
 * @return		- 1 bit read value from input register
 *
 * @note		- none
 *
 ******************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *GPIOx_ptr,
							  uint8_t pinNumber)
{
	// shifts the desired pin's bit to the 0th bit position and masks the rest
	// of the bits, returning that result
	return (GPIOx_ptr->IDR >> pinNumber) & 1;

	// example IDR: 0111_0110_0x00_0010
	// if the 6th pin is desired, shift right by 6 bits
	// 0111_0110_0x00_0010 >> 6 =
	// 0000_0001_1101_100x
	// bitwise and operation with a 1 to isolate that bit
	// 0000_0001_1101_100x & 1 = x
}

/*******************************************************************************
 * @fn 			- GPIO Read From Input Port
 *
 * @brief		- Reads from the entire GPIO port
 *
 * @param		- base address of GPIO port
 *
 * @return		- 16 bit read value from input register
 *
 * @note		- none
 *
 ******************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *GPIOx_ptr)
{
	return (GPIOx_ptr->IDR & 0xFFFF);
}

/*******************************************************************************
 * @fn 			- GPIO Write to Output Pin
 *
 * @brief		- Writes a single bit to a given GPIO pin
 *
 * @param		- base address of GPIO port
 * @param 		- pin number to write to
 * @param 		- 1 bit value to write
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *GPIOx_ptr,
						   uint8_t pinNumber,
						   uint8_t value)
{
	// if value is 1, set the bit position of the desired pin to 1
	if (value == 1) {
		GPIOx_ptr->ODR |= (1 << pinNumber);
	}
	// otherwise, clear it to 0
	else {
		GPIOx_ptr->ODR &= ~(1 << pinNumber);
	}
}
/*******************************************************************************
 * @fn 			- GPIO Write to Output Port
 *
 * @brief		- Writes a 16 bit value to the given GPIO port
 *
 * @param		- base address of GPIO port
 * @param 		- 16 bit value to write
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *GPIOx_ptr,
							uint16_t value)
{
	GPIOx_ptr->ODR = value;
}

/*******************************************************************************
 * @fn 			- GPIO Toggle Output Pin
 *
 * @brief		- Toggles the GPIO output data register for the given pin
 *
 * @param		- base address of GPIO port
 * @param 		- pin number to toggle
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *GPIOx_ptr,
	   	   	    		  uint8_t pinNumber)
{
	GPIOx_ptr->ODR ^= (1 << pinNumber);
}

// IRQ config and ISR handling

/*******************************************************************************
 * @fn 			- GPIO interrupt config
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,
							 uint8_t en)
{
	if (en)
	{
		// each ISER register is 32 bits wide
		if (IRQNumber <= 31)
		{
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}
		else if (IRQNumber <= 63)
		{
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}
		else if (IRQNumber <= 95)
		{
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));

		}
	}
	else {
		// each ISER register is 32 bits wide
		if (IRQNumber <= 31)
		{
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber <= 63)
		{
			// program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		}
		else if (IRQNumber <= 95)
		{
			// program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));

		}
	}
}

/*******************************************************************************
 * @fn 			- GPIO interrupt priority config
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,
							uint8_t IRQ_Priority)
{
	// calculate IPR register
	uint8_t IPR_register_number = IRQNumber / 4;

	// calculate shift value for priority bits
	uint8_t shift_value = (8 * (IRQNumber % 4)) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASE_ADDR + (4 * IPR_register_number)) |=
										(IRQ_Priority << shift_value);
}

/*******************************************************************************
 * @fn 			- GPIO interrupt request handling
 *
 * @brief		- Handles the GPIO interrupt request by
 * 				  clearing EXTI PR register
 *
 * @param		- pin number of the interrupt we want to clear
 *
 * @return		- none
 *
 * @note		- none
 *
 ******************************************************************************/
void GPIO_IRQHandling(uint8_t pinNumber)
{
	// clear the EXTI PR register corresponding to the pin number
	if (EXTI->PR & (1 << pinNumber))
	{
		// clear the bit
		EXTI->PR |= (1 << pinNumber);
	}
}


















