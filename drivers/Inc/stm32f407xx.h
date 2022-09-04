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

/****************************************START:Processor specific details********************************************/
/*
 * 	ARM Cortex-m4 processor specific NVIC ISERx register Address
 */
#define NVIC_ISER0				(__vo uint32_t*)0xE000E100
#define NVIC_ISER1				(__vo uint32_t*)0xE000E104
#define NVIC_ISER2				(__vo uint32_t*)0xE000E108
#define NVIC_ISER3				(__vo uint32_t*)0xE000E10C
#define NVIC_ISER4				(__vo uint32_t*)0xE000E110
#define NVIC_ISER5				(__vo uint32_t*)0xE000E114
#define NVIC_ISER6				(__vo uint32_t*)0xE000E118
#define NVIC_ISER7				(__vo uint32_t*)0xE000E11C

/*
 * 	ARM Cortex-m4 processor specific NVIC ICERx register Address
 */


#define NVIC_ICER0				(__vo uint32_t*)0XE000E180
#define NVIC_ICER1				(__vo uint32_t*)0XE000E184
#define NVIC_ICER2				(__vo uint32_t*)0XE000E188
#define NVIC_ICER3				(__vo uint32_t*)0XE000E18C
#define NVIC_ICER4				(__vo uint32_t*)0XE000E190
#define NVIC_ICER5				(__vo uint32_t*)0XE000E194
#define NVIC_ICER6				(__vo uint32_t*)0XE000E198
#define NVIC_ICER7				(__vo uint32_t*)0XE000E19C

/*
 * ARM Cortex-m4 processor specific NVIC IPRx register Address
 */

#define NVIC_IPR_BASEADDR		((__vo uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED					4





/*********************************************************************************************************************/
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
	__vo uint32_t MODER;							//GPIO port mode register
	__vo uint32_t OTYPER;							//GPIO port output type register
	__vo uint32_t OSPEEDR;							//GPIO port output speed register
	__vo uint32_t PUPDR;							//GPIO port pull-up/pull-down register
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
 * Peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;

#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)


/*
 *Peripheral structure register for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

#define SYSCFG 		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


/*
 * Peripheral structure register for Serial Parallel Interface
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;


#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)
/*******************************************************************************************************************/

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
 * Some generic macros
 */
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		ENABLE
#define GPIO_PIN_RESET		DISABLE
#define FLAG_RESET			RESET
#define FLAG_SET			SET

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



/**********************************************************************************************************************
 * Macros to reset GPIOx peripherals
 **********************************************************************************************************************/

#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<0));	(RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<1));	(RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<2)); 	(RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<3));	(RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<4));	(RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<5));	(RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<6));	(RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<7));	(RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<8));	(RCC->AHB1RSTR &= ~(1<<8));}while(0)

/**********************************************************************************************************************
 * Macros to reset pSPIx peripherals
 **********************************************************************************************************************/
#define SPI1_REG_RESET()		do{(RCC->APB2RSTR  |= (1<<12));	(RCC->APB2RSTR  &= ~(1<<12));}while(0)
#define SPI2_REG_RESET()		do{(RCC->APB1RSTR  |= (1<<14));	(RCC->APB1RSTR  &= ~(1<<14));}while(0)
#define SPI3_REG_RESET()		do{(RCC->APB1RSTR  |= (1<<15));	(RCC->APB1RSTR  &= ~(1<<15));}while(0)


/*
 * return the port code for EXTICR[x] registers and takes the input as GPIO port location or GPIO base address
 */
#define GPIO_BASEADDR_TO_CODE(x)		{(x==GPIOA)?0:\
										(x==GPIOB)?1:\
										(x==GPIOC)?2:\
										(x==GPIOD)?3:\
										(x==GPIOE)?4:\
										(x==GPIOF)?5:\
										(x==GPIOG)?6:\
										(x==GPIOH)?7:\
										(x==GPIOI)?8:0}


/*
 * IRQ(Interrupt request) numbers of the STM32F407x MCU
 * This is a generic interrupt information so better to keep in the device driver file
 */
#define IRQ_NO_EXTI0					6
#define IRQ_NO_EXTI1					7
#define IRQ_NO_EXTI2					8
#define IRQ_NO_EXTI3					9
#define IRQ_NO_EXTI4					10
#define IRQ_NO_EXTI9_5					23
#define IRQ_NO_EXT15_10					40

#define IRQ_NO_SPI1						35
#define IRQ_NO_SPI2						36
#define IRQ_NO_SPI3						51

/*
 * IRQ PROIORITY NUMBERS
 */
#define NVIC_IRQ_PRIO0					0
#define NVIC_IRQ_PRIO1					1
#define NVIC_IRQ_PRIO2					2
#define NVIC_IRQ_PRIO3					3
#define NVIC_IRQ_PRIO4					4
#define NVIC_IRQ_PRIO5					5
#define NVIC_IRQ_PRIO6					6
#define NVIC_IRQ_PRIO7					7
#define NVIC_IRQ_PRIO8					8
#define NVIC_IRQ_PRIO9					9
#define NVIC_IRQ_PRIO10					10
#define NVIC_IRQ_PRIO11					11
#define NVIC_IRQ_PRIO12					12
#define NVIC_IRQ_PRIO13					13
#define NVIC_IRQ_PRIO14					14
#define NVIC_IRQ_PRIO15					15
#define NVIC_IRQ_PRIO16					16
#define NVIC_IRQ_PRIO17					17


/****************************************************************************************************************
 ********************************* Bit definition macro for the SPI Periph***************************************
*************************************************************************************************************** */
//For Control Register 1
#define SPI_CR1_CPHA					0
#define SPI_CR1_CPOL					1
#define SPI_CR1_MSTR					2
#define SPI_CR1_BR						3
#define SPI_CR1_SPE						6
#define SPI_CR1_LSBFIRST				7
#define SPI_CR1_SSI						8
#define SPI_CR1_SSM						9
#define SPI_CR1_RXONLY					10
#define SPI_CR1_DFF						11
#define SPI_CR1_CRCNEXT					12
#define SPI_CR1_CRCEN					13
#define SPI_CR1_BIDIOE					14
#define SPI_CR1_BIDIMODE				15

//FOR SPI_CR2
#define SPI_CR2_RXDMAEN					0
#define SPI_CR2_TXDMAEN					1
#define SPI_CR2_SSOE					2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE					6
#define SPI_CR2_TXEIE					7


//For Status Register
#define SPI_SR_RXNE						0
#define SPI_SR_TXE						1
#define SPI_SR_CHSIDE					2
#define SPI_SR_UDR						3
#define SPI_SR_CRCERR					4
#define SPI_SR_MODF						5
#define SPI_SR_OVR						6
#define SPI_SR_BSY						7
#define SPI_SR_FRE						8

#endif /* INC_STM32F407XX_H_ */
