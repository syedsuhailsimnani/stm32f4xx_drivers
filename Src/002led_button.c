/*
 * 002led_button.c
 *
 *  Created on: Aug 2, 2022
 *      Author: syedsuhailsimnani
 */


#include <stm32f407xx.h>
#include <stm32f407xx_gpio_driver.h>

void delay(void)
{
	for(uint32_t i=0; i<500000/2;i++);
}
int main(void)
{
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Handle_t GpioLed,GpioLed2,GpioBtn;


	//This is the configurations of two leds present at PD12 and PD13
	GpioLed.pGPIOx=GPIOD;
	GpioLed2.pGPIOx=GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GpioLed2.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_OUT;
	GpioLed2.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLed2.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GpioLed2.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_HIGH;
	GpioLed2.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_HIGH;
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioLed2);


	//Configuration params for the button
	GpioBtn.pGPIOx=GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_HIGH;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);
	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, 0))
		{
			GPIO_ToggleOutputPin(GPIOD, 12);
			GPIO_ToggleOutputPin(GPIOD, 13);
			delay();
		}

	}
	return 0;
}
