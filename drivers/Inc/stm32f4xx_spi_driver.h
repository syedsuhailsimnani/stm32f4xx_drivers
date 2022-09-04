/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: Aug 11, 2022
 *      Author: syedsuhailsimnani
 */
#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_
#include"stm32f407xx.h"

/*
 * SPI Application States
 */
#define SPI_Ready				0
#define SPI_BUSY_IN_RX			1
#define SPI_BUSY_IN_TX			2


/*
 * /Possible SPI Application  events
 */

#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4

/*
 * SPI peripheral configuration structure
 */

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


/*
 * SPI Handle Structure for configuration of the registers of the SPI peripheral
 */

typedef struct
{
	SPI_RegDef_t 	*pSPIx;			//This holds the base address of the  SPI(0,1,2 )peripherals
	SPI_Config_t 	SPIConfig;
	uint8_t		 	*pTxBuffer;
	uint8_t		 	*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxState;
	uint8_t			RxState;
}SPI_Handle_t;


/*
 * @SPI Device Macros for device macros
 */
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

/*
 * @SPI Device Macros for Bus Config
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_S_RXONLY			3

/*
 * @SPI Device Macros for Speed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * @SPI Device macros for Data frame format
 */
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/*
 * @SPI Device macros for Clock Polarity (CPOL)
 */
#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1


/*
 * @SPI Device macros for Clock Phase (CPHA)
 */
#define SPI_CPHA_LOW					0
#define SPI_CPHA_HIGH					1

/*
 * @SPI device macros for software slave management
 */

#define SPI_SSM_DI						0
#define SPI_SSM_EN						1

/*
 * @SPI STATUS FLAGS
 */
#define SPI_TXE_FLAG					(1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG					(1<<SPI_SR_RXNE)
#define SPI_BUSY_FLAG					(1<<SPI_SR_BSY)
/******************************************************************************************************************************
 *											API Supported by this driver
 *											for more information please check the function definitions
 *****************************************************************************************************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDI);
/*
 * Initialization  and De-Initialisation of the SPI Peripheral
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveDate(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);
/*
 * Data send and receive via SPI using Interrupts
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDateIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ Configuration and ISR Handling
 */
void 		SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void 		SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void 		SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral control apis
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);




#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
