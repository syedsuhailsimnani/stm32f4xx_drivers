#include <stm32f407xx.h>
#include <stm32f407xx_gpio_driver.h>

void delay(void)
{
	for(uint32_t i=0; i<500000/2;i++);
}
int main(void)
{
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Handle_t GpioLed,GpioLed2;
	GpioLed.pGPIOx=GPIOD;
	GpioLed2.pGPIOx=GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GpioLed2.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_OUT;
	GpioLed2.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	GpioLed2.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GpioLed2.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_HIGH;
	GpioLed2.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_HIGH;
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioLed2);
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, 12);
		GPIO_ToggleOutputPin(GPIOD, 13);
		delay();
	}
	return 0;
}

