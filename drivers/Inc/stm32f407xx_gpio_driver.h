/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jul 30, 2022
 *      Author: syedsuhailsimnani
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"


typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

/*
 * this is a handle structure for a GPIO pin
 */

typedef	struct
{
	//pinter to hold the base address of  the GPIO Peripheral
	GPIO_RegDef_t *GPIOx;					//this hold the base address of the GPIO Port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;		//this holds GPIO pin configuration settings
}GPIO_Handle_t;


/******************************************************************************************************************************
 *											API Supported by this driver
 *											for more information please check the funciton definitions
 *****************************************************************************************************************************/
/*
 * Peripheral Clock COnfguration
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDI);


/*
 * GPIO Init and Deinit
 */
void GPIO_Init(void);
void GPIO_DeInit(void);


/*
 * Data read and write
 */
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(void);


/*
 * GPIO IRQ configuration and handling
 */
void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);









#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
