/*
 * 002led_button.c
 *
 *  Created on: Aug 2, 2022
 *      Author: syedsuhailsimnani
 */


#include <stm32f407xx.h>
#include <stm32f407xx_gpio_driver.h>
#include <string.h> //for the memset api

void delay(void)
{
	for(uint32_t i=0; i<500000/2;i++);
}
int main(void)
{
	//Creating handle variable for the leds and buttons
	GPIO_Handle_t GpioLed,GpioLed2,GpioBtn;

	//CLearing the structure values to zero for initial configuration
	memset(&GpioLed, 0,sizeof(GpioLed));
	memset(&GpioLed2,0,sizeof(GpioLed2));
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	//This is the configurations of two leds present at PD12 and PD13
	//Selecting the port number D for both leds
	GpioLed.pGPIOx=GPIOD;
	GpioLed2.pGPIOx=GPIOD;

	//Selecting the pin number 12 and 13
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GpioLed2.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;

	//Selcting the mode of LEDs to output type
	GpioLed.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_OUT;
	GpioLed2.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;

	//Selecting the push pull configuratoin for the pins
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLed2.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;

	//Disabling the internal pull ups and pull downs
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GpioLed2.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	//Selecting the very high speed for the response of the pins
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_HIGH;
	GpioLed2.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_HIGH;

	//Enabling the Clock for the GPIOD for registering the catched settings
	GPIO_PeriClockControl(GPIOD, ENABLE);

	//Initializing the portD LEDs using respective handler variables
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioLed2);



	//Configuration params for the button
	//Selecting port A for the button input
	GpioBtn.pGPIOx=GPIOD;

	//Using the pin number 0 for button input
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;

	//Selecting the input type as mode for the button pin config
	GpioBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;

	//Selecting No Pull up internally
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;

	//Selecting the button pin response rate configuration
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_HIGH;

	//Initializing the button pin port clock
	GPIO_PeriClockControl(GPIOD, ENABLE);

	//Catching the button pin configuration settings in the initialization function
	GPIO_Init(&GpioBtn);

	//IRQ configuration
	//Setting IRQ Priority
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);

	//Setting the Interrupt configruation to EXTI pin 9 to 5 as we are using Port D pin number 5;
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);

	return 0;
}

//overriding the interrupt request handler as defined in the  header file for the application
void EXTI9_5_IRQHandler(void)
{
	//delay function for the button debouncing issue
	delay();
	//GPIO irq handling api clears the interrupt that has been triggered.
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, 12);
	GPIO_ToggleOutputPin(GPIOD, 13);

}
