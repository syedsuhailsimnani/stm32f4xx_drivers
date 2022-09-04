/*
 * 007spi__txonly_arduino.c
 *
 *  Created on: Aug 15, 2022
 *      Author: syedsuhailsimnani
 */

#include <stm32f407xx.h>
#include <stm32f407xx_gpio_driver.h>
#include <stm32f4xx_spi_driver.h>
#include <string.h>

//Testing the SPI2 peripheral
/*
 * we will be using
 * PB15-->MOSI
 * PB14-->MISO
 * PB13-->SCLK
 * PB12-->NSS
 * All this is avaialable for the pin in the alternate functionality mode 5
 */

void SPI2_GPIOInits(void)
{

	GPIO_Handle_t	SPIPins;
	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode =5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	//For Pin number 12 NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

	//For pin number 13 SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//For pin number 14 MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//For Pin number 15 MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
}


void SPI2_Inits(void)
{
	SPI_Handle_t	SPI2Handle;
	SPI2Handle.pSPIx=SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV8; //Generate the SCLK of 2MHz
	SPI2Handle.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM=SPI_SSM_DI;  //Hardware Slavemgmt available for NSS PIN
	SPI_Init(&SPI2Handle);


}


void GPIO_ButtonInit()
{
		GPIO_Handle_t GpioBtn;
	//Configuration params for the button
		GpioBtn.pGPIOx=GPIOA;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_HIGH;
		GPIO_Init(&GpioBtn);
}

void delay(void)
{
	for(uint32_t i=0; i<500000/2;i++);
}
int main(void)
{
	char userDefine[]="Hello World";

	GPIO_ButtonInit();

	//The function which is able to initialise the pins as the SPI pins
	SPI2_GPIOInits();

	//Peripheral configuration which could be done by configuring the SPI handle structure
	SPI2_Inits();

	//Configuration of the SSOE bit in case where the SSI
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();
		//SPI peripheral needs to be enabled before calling the send data function
		SPI_PeripheralControl(SPI2, ENABLE);

		//First send the number of byte information we are going to send
		uint8_t datalength=strlen(userDefine);

		//Sending the SPI datalength to inform the number of byter for transmission.
		SPI_SendData(SPI2, &datalength,1);

		//The API helps us to send data
		SPI_SendData(SPI2, (uint8_t*)userDefine, strlen(userDefine));

		//Confirming SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//Diabling the Spi forcefully
		SPI_PeripheralControl(SPI2, DISABLE);
	}
	return 0;
}



