/*
 * spiExampleBlocking.c
 *
 *  Created on: Aug 14, 2022
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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);

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
	SPI2Handle.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM=SPI_SSM_EN;  //Software slave management available for NSS PIN
	SPI_Init(&SPI2Handle);


}
int main(void)
{
	char userDefine[]="Hello World";
	//The function which is able to initialise the pins as the SPI pins
	SPI2_GPIOInits();

	//Peripheral configuration which could be done by configuring the SPI handle structure
	SPI2_Inits();

	//Slecting the configuration for ssi to retain the NSS software slave management configurations
	SPI_SSIConfig(SPI2, ENABLE);

	//SPI peripheral needs to be enabled before calling the send data function
	SPI_PeripheralControl(SPI2, ENABLE);

	//The API helps us to send data
	SPI_SendData(SPI2, (uint8_t*)userDefine, strlen(userDefine));

	//Confirming SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	//Closing the SPI communication after checking for SPI busy flag
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);
}
