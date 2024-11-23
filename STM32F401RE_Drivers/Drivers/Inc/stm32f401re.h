/*
 * stm32f401re.h
 *
 *  Created on: Oct 23, 2024
 *      Author: haykkhulyan
 */

#ifndef INC_STM32F401RE_H_
#define INC_STM32F401RE_H_

#include <stdint.h>

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						      Processor Specific Details
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// ARM Cortex M4 Processor NVIC ISERx Register Addresses
#define NVIC_ISER0		((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1		((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2		((volatile uint32_t*) 0xE000E108)
#define NVIC_ISER3		((volatile uint32_t*) 0xE000E10C)
#define NVIC_ISER4		((volatile uint32_t*) 0xE000E110)
#define NVIC_ISER5		((volatile uint32_t*) 0xE000E114)
#define NVIC_ISER6		((volatile uint32_t*) 0xE000E118)
#define NVIC_ISER7		((volatile uint32_t*) 0xE000E11C)

// ARM Cortex M4 Processor NVIC ICERx Register Addresses
#define NVIC_ICER0		((volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1		((volatile uint32_t*) 0xE000E184)
#define NVIC_ICER2		((volatile uint32_t*) 0xE000E188)
#define NVIC_ICER3		((volatile uint32_t*) 0xE000E18C)
#define NVIC_ICER4		((volatile uint32_t*) 0xE000E190)
#define NVIC_ICER5		((volatile uint32_t*) 0xE000E194)
#define NVIC_ICER6		((volatile uint32_t*) 0xE000E198)
#define NVIC_ICER7		((volatile uint32_t*) 0xE000E19C)

// ARM Cortex M4 Processor NVIC IPRx Register Base Address
#define NVIC_IPR_BASE_ADDR	((volatile uint32_t*) 0xE000E400)

// number of priority bits implemented by the STM32F401RE
#define NO_PR_BITS_IMPLEMENTED 4


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						           Generic Macros
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

// takes a pointer value and compares it to GPIO port addresses to return a
// corresponding port code
#define GPIO_BASEADDR_TO_PORTCODE(x) ((x == GPIOA) ? 0 : \
									  (x == GPIOB) ? 1 : \
									  (x == GPIOC) ? 2 : \
									  (x == GPIOD) ? 3 : \
									  (x == GPIOE) ? 4 : \
									  (x == GPIOH) ? 7 : 0)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						Base Addresses of Flash and SRAM
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define FLASH_BASE_ADDR				(0x08000000UL)
#define SRAM1_BASE_ADDR				(0x20000000UL)
#define ROM_BASE_ADDR				(0x1FFF0000UL)
#define SRAM						(SRAM1_BASE_ADDR)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *					Base Addresses of AHBx and APBx Bus Peripherals
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define PERIPHERAL_BASE_ADDR		(0x40000000UL)
#define APB1_PERIPHERAL_BASE_ADDR	(PERIPHERAL_BASE_ADDR)
#define APB2_PERIPHERAL_BASE_ADDR	(0X40010000UL)
#define AHB1_PERIPHERAL_BASE_ADDR	(0X40020000UL)
#define AHB2_PERIPHERAL_BASE_ADDR	(0X50000000UL)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						Base Addresses of APB1 Bus Peripherals
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define TIM2_BASE_ADDR				(APB1_PERIPHERAL_BASE_ADDR + 0x0000)
#define TIM3_BASE_ADDR				(APB1_PERIPHERAL_BASE_ADDR + 0x0400)
#define TIM4_BASE_ADDR				(APB1_PERIPHERAL_BASE_ADDR + 0x0800)
#define TIM5_BASE_ADDR				(APB1_PERIPHERAL_BASE_ADDR + 0x0C00)

#define RTC_BASE_ADDR				(APB1_PERIPHERAL_BASE_ADDR + 0x2800)

#define WWDG_BASE_ADDR				(APB1_PERIPHERAL_BASE_ADDR + 0x2C00)
#define IWDG_BASE_ADDR				(APB1_PERIPHERAL_BASE_ADDR + 0x3000)

#define I2S2EXT_BASE_ADDR			(APB1_PERIPHERAL_BASE_ADDR + 0x3400)

#define SPI2_BASE_ADDR				(APB1_PERIPHERAL_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR				(APB1_PERIPHERAL_BASE_ADDR + 0x3C00)

#define I2S3EXT_BASE_ADDR			(APB1_PERIPHERAL_BASE_ADDR + 0x4000)

#define USART2_BASE_ADDR			(APB1_PERIPHERAL_BASE_ADDR + 0x4400)

#define I2C1_BASE_ADDR				(APB1_PERIPHERAL_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR				(APB1_PERIPHERAL_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR				(APB1_PERIPHERAL_BASE_ADDR + 0x5C00)

#define PWR_BASE_ADDR				(APB1_PERIPHERAL_BASE_ADDR + 0x7000)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						Base Addresses of APB2 Bus Peripherals
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define TIM1_BASE_ADDR				(APB2_PERIPHERAL_BASE_ADDR + 0x0000)

#define USART1_BASE_ADDR			(APB2_PERIPHERAL_BASE_ADDR + 0x1000)
#define USART6_BASE_ADDR			(APB2_PERIPHERAL_BASE_ADDR + 0x1400)

#define ADC_BASE_ADDR				(APB2_PERIPHERAL_BASE_ADDR + 0x2000)

#define SDIO_BASE_ADDR				(APB2_PERIPHERAL_BASE_ADDR + 0x2C00)

#define SPI1_BASE_ADDR				(APB2_PERIPHERAL_BASE_ADDR + 0x3000)
#define SPI4_BASE_ADDR				(APB2_PERIPHERAL_BASE_ADDR + 0x3400)

#define SYSCFG_BASE_ADDR			(APB2_PERIPHERAL_BASE_ADDR + 0x3800)

#define EXTI_BASE_ADDR				(APB2_PERIPHERAL_BASE_ADDR + 0x3C00)

#define TIM9_BASE_ADDR				(APB2_PERIPHERAL_BASE_ADDR + 0x4000)
#define TIM10_BASE_ADDR				(APB2_PERIPHERAL_BASE_ADDR + 0x4400)
#define TIM11_BASE_ADDR				(APB2_PERIPHERAL_BASE_ADDR + 0x4800)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						Base Addresses of AHB1 Bus Peripherals
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define GPIOA_BASE_ADDR				(AHB1_PERIPHERAL_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR				(AHB1_PERIPHERAL_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR				(AHB1_PERIPHERAL_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR				(AHB1_PERIPHERAL_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR				(AHB1_PERIPHERAL_BASE_ADDR + 0x1000)
#define GPIOH_BASE_ADDR				(AHB1_PERIPHERAL_BASE_ADDR + 0x1C00)

#define CRC_BASE_ADDR				(AHB1_PERIPHERAL_BASE_ADDR + 0x3000)

#define RCC_BASE_ADDR				(AHB1_PERIPHERAL_BASE_ADDR + 0x3800)

#define FLASH_INTERFACE_BASE_ADDR	(AHB1_PERIPHERAL_BASE_ADDR + 0x3C00)

#define DMA1_BASE_ADDR				(AHB1_PERIPHERAL_BASE_ADDR + 0x6000)
#define DMA2_BASE_ADDR				(AHB1_PERIPHERAL_BASE_ADDR + 0x6400)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						Base Addresses of AHB2 Bus Peripherals
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define USB_OTG_FS_BASE_ADDR		(AHB2_PERIPHERAL_BASE_ADDR)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						    Peripheral Register Definitions
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

} GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	uint32_t RES1;
	uint32_t RES2;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RES3;
	uint32_t RES4;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	uint32_t RES5;
	uint32_t RES6;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RES7;
	uint32_t RES8;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	uint32_t RES9;
	uint32_t RES10;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RES11;
	uint32_t RES12;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RES13;
	uint32_t RES14;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	uint32_t RES15;
	volatile uint32_t DCKCFGR;

} RCC_RegDef_t;

typedef struct {
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;

} EXTI_RegDef_t;

typedef struct {
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t CMPCR;

} SYSCFG_RegDef_t;

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2CCFGR;
	volatile uint32_t I2SPR;

} SPI_RegDef_t;

typedef struct {
	volatile uint32_t SR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMPR1;
	volatile uint32_t SMPR2;
	volatile uint32_t JOFR[4];
	volatile uint32_t HTR;
	volatile uint32_t LTR;
	volatile uint32_t SQR1[3];
	volatile uint32_t JSQR;
	volatile uint32_t JDR[4];
	volatile uint32_t DR;
	uint32_t RES[149];
	volatile uint32_t CCR;

} ADC_RegDef_t;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						           Peripheral Definitions
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define GPIOA  ((GPIO_RegDef_t*) GPIOA_BASE_ADDR)
#define GPIOB  ((GPIO_RegDef_t*) GPIOB_BASE_ADDR)
#define GPIOC  ((GPIO_RegDef_t*) GPIOC_BASE_ADDR)
#define GPIOD  ((GPIO_RegDef_t*) GPIOD_BASE_ADDR)
#define GPIOE  ((GPIO_RegDef_t*) GPIOE_BASE_ADDR)
#define GPIOH  ((GPIO_RegDef_t*) GPIOH_BASE_ADDR)

#define RCC    ((RCC_RegDef_t*)  RCC_BASE_ADDR)

#define EXTI   ((EXTI_RegDef_t*) EXTI_BASE_ADDR)

#define SYSCFG ((SYSCFG_RegDef_t*) SYSCFG_BASE_ADDR)

#define SPI1   ((SPI_RegDef_t*) SPI1_BASE_ADDR)
#define SPI2   ((SPI_RegDef_t*) SPI2_BASE_ADDR)
#define SPI3   ((SPI_RegDef_t*) SPI3_BASE_ADDR)
#define SPI4   ((SPI_RegDef_t*) SPI4_BASE_ADDR)

#define ADC    ((ADC_RegDef_t*) ADC_BASE_ADDR))

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						   SPI Register Bit Position Definitions
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// SPI Control Register 1
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL 		1
#define SPI_CR1_MSTR 		2
#define SPI_CR1_BR 			3
#define SPI_CR1_SPE 		6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI 		8
#define SPI_CR1_SSM 		9
#define SPI_CR1_RXONLY 		10
#define SPI_CR1_DFF 		11
#define SPI_CR1_CRCNEXT 	12
#define SPI_CR1_CRCEN 		13
#define SPI_CR1_BIDIOE 		14
#define SPI_CR1_BIDIMODE 	15

// SPI Control Register 2
#define SPI_CR2_RXDMAEN	0
#define SPI_CR2_TXDMAEN	1
#define SPI_CR2_SSOE	2
#define SPI_CR2_FRF		4
#define SPI_CR2_ERRIE	5
#define SPI_CR2_RXNEIE	6
#define SPI_CR2_TXEIE   7

// SPI Status Register
#define SPI_SR_RXNE		0
#define SPI_SR_TXE		1
#define SPI_SR_CHSIDE	2
#define SPI_SR_UDR		3
#define SPI_SR_CRCERR	4
#define SPI_SR_MODF 	5
#define SPI_SR_OVR  	6
#define SPI_SR_BSY  	7
#define SPI_SR_FRE  	8

// SPI Data Register
#define SPI_DR	0

// SPI I2S Configuration Register
#define SPI_I2SCFGR_CHLEN	0
#define SPI_I2SCFGR_DATLEN	1
#define SPI_I2SCFGR_CKPOL	3
#define SPI_I2SCFGR_I2SSTD	4
#define SPI_I2SCFGR_PCMSYNC	7
#define SPI_I2SCFGR_I2SCFG	8
#define SPI_I2SCFGR_I2SE	10
#define SPI_I2SCFGR_I2SMOD	11

// SPI I2S Prescaler Register
#define SPI_I2SSPR_I2SDIV	0
#define SPI_I2SSPR_ODD		8
#define SPI_I2SSPR_MCKOE	9

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						Clock Enable Macros for Peripherals
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// GPIO peripheral clock enables
#define GPIOA_PCLK_EN()  (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()  (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()  (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()  (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()  (RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()  (RCC->AHB1ENR |= (1 << 7))

// I2C peripheral clock enables
#define I2C1_PCLK_EN()   (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()   (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()   (RCC->APB1ENR |= (1 << 23))

// SPI peripheral clock enables
#define SPI1_PCLK_EN()   (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()   (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()   (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()	 (RCC->APB2ENR |= (1 << 13))

// USART peripheral clock enables
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

// SYSCFG peripheral clock enables
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

// ADC peripheral clock enables
#define ADC_PCLK_EN()	 (RCC->APB2ENR |= (1 << 8))

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						Clock Disable Macros for Peripherals
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// GPIO peripheral clock disables
#define GPIOA_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 7))

// I2C peripheral clock disables
#define I2C1_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 23))

// SPI peripheral clock disables
#define SPI1_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()	 (RCC->APB2ENR &= ~(1 << 13))

// USART peripheral clock disables
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))

// SYSCFG peripheral clock disables
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))

// ADC peripheral clock disables
#define ADC_PCLK_DI()	 (RCC->APB2ENR &= ~(1 << 8))

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *						           IRQ Numbers
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#include "stm32f401re_gpio.h"
#include "stm32f401re_spi.h"
#include "stm32f401re_adc.h"

#endif /* INC_STM32F401RE_H_ */








