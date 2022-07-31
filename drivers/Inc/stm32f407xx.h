/*
 * stm32f407xx.h
 *
 *  Created on: Jul 24, 2022
 *      Author: syedsuhailsimnani
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include <stdint.h>

#define __vo		volatile

#define FLASH_BASEADDR		0x08000000U					//Base address of the flash or program memory
#define SRAM1_BASEADDR		0x20000000U					//Base address of the SRAM1 memory
#define SRAM2_BASEADDR		0x2001C000U					//Base address of the SRAM2 memory
#define ROM_BASEADDR		0x1FFF0000					//Base address of the system memory
#define SRAM 				SRAM1_BASEADDR				//SRAM1 is the also referenced as SRAM


/*
 * AHBx and APBx Bus Peripherals base addresses
 */

#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

/*
 * Definition of the peripherals which are hanging on the AHB1 BUS
 * TODO: Complete all other peripherals
 */

#define GPIOA_BASEADDR		(AHB1PERIPH_BASE+0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE+0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE+0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE+0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE+0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE+0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE+0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE+0x1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASE+0x2000)
#define GPIOJ_BASEADDR		(AHB1PERIPH_BASE+0x2400)
#define GPIOK_BASEADDR		(AHB1PERIPH_BASE+0x2800)
#define RCC_BASEADDR		(AHB1PERIPH_BASE+0x3800)


/*
 * Definition of the peripherals which are hanging on the APB1 BUS
 * TODO: Complete all other peripherals
 */

#define I2C1_BASEADDR		(APB1PERIPH_BASE+0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASE+0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASE+0x5C00)
#define SPI2_BASEADDR		(APB1PERIPH_BASE+0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASE+0x3C00)
#define USART2_BASEADDR		(APB1PERIPH_BASE+0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASE+0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASE+0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASE+0x5000)

/*
 * Definition of the peripherals which are hanging on the APB2 BUS
 * TODO: Complete all other peripherals
 */
#define EXTI_BASEADDR		(APB2PERIPH_BASE+0x3C00)
#define SPI1_BASEADDR		(APB2PERIPH_BASE+0x3000)
#define USART1_BASEADDR		(APB2PERIPH_BASE+0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASE+0x1400)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASE+0x3800)




/*******************************************************Peripheral register definition structure*******************************************************/

/*
 * Note:Registers of a peripheral are specific to MCU
 */
typedef struct
{
	__vo uint32_t MODER;								//GPIO port mode register
	__vo uint32_t OTYPER;							//GPIO port output type register
	__vo uint32_t OSPEEDR;							//GPIO port output speed register
	__vo uint32_t PUPDR;								//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;								//GPIO port input data register
	__vo uint32_t ODR;								//GPIO port output data register
	__vo uint32_t BSRR;								//GPIO port bit set/reset register
	__vo uint32_t LCKR;								//GPIO port configuration lock register
	__vo uint32_t AFR[2];							//GPIO alternate function high register AFRL= AFR[0], AFRH= AFR[1]
}GPIO_RegDef_t;


/*
 * Peripheral definitions (Peripheral Base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA     		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB     		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC     		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD     		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE     		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF     		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG     		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH     		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI     		((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ     		((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK     		((GPIO_RegDef_t*)GPIOK_BASEADDR)



typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t Reserved0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t Reserved1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t Reserved2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t Reserved3[2];
	__vo uint32_t AHB1LP_ENR;
	__vo uint32_t AHB2LP_ENR;
	__vo uint32_t AHB3LP_ENR;
	uint32_t Reserved4;
	__vo uint32_t APB1LP_ENR;
	__vo uint32_t APB2LP_ENR;
	uint32_t Reserved5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t Reserved6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SC_FGR;
}RCC_RegDef_t;

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)


/*
 * Clock enable macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1<<8))


/*
 * Clock Enable macros for I2C Periphals
 */
#define I2C1_PCLK_EN()			(RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |=(1<<23))


/*
 * Clock Enable macros for SPI Peripherals
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |=(1<<14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |=(1<<15))

/*
 * Clock Enable macros  for USARTx Peripherals
 */
#define USART1_PCLK_EN()		(RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |=(1<<17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |=(1<<18))
#define USART4_PCLK_EN()		(RCC->APB1ENR |=(1<<19))
#define USART5_PCLK_EN()		(RCC->APB1ENR |=(1<<20))
#define USART6_PCLK_EN()		(RCC->APB2ENR |=(1<<5))

/*
 * Clock Enable macros for SYSCFG Peripherals
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |=(1<<14))

/*
 * Clock Disable macros for GPIOX Peripherals
 */
#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<8))

/*
 * Clock Disable macros for I2Cx Peripherals
 */
#define I2C1_PCLK_DI()			(RCC->APB1ENR &=~(1<<21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR &=~(1<<22))
#define I2C3_PCLK_DI()			(RCC->APB1ENR &=~(1<<23))
/*
 * Clock Disable macros for SPIx Peripherals
 */
#define SPI1_PCLK_DI()			(RCC->APB2ENR &=~(1<<12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &=~(1<<14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &=~(1<<15))

/*
 * Clock Disable macros for USARTx Peripherals
 */
#define USART1_PCLK_DI()		(RCC->APB2ENR &=~(1<<4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &=~(1<<17))
#define USART3_PCLK_DI()		(RCC->APB1ENR &=~(1<<18))
#define USART4_PCLK_DI()		(RCC->APB1ENR &=~(1<<19))
#define USART5_PCLK_DI()		(RCC->APB1ENR &=~(1<<20))
#define USART6_PCLK_DI()		(RCC->APB2ENR &=~(1<<5))
/*
 * Clock Disable macros for SYSCFGx Peripherals
 */
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &=~(1<<14))


#endif /* INC_STM32F407XX_H_ */
