/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Aug 11, 2022
 *      Author: syedsuhailsimnani
 */

#include "stm32f4xx_spi_driver.h"



static void	spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void	spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void	spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
/***********************************************************************************************************************
 * @fn				--Explaining the funciton name
 *
 * @brief			--Eplaining the  funtionality of the API
 *
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 *
 * @return type		--return type of the function and how it should be handled
 *
 * @Note 			--any special requirements or instructions that the user needs to follow while using this API
 ***********************************************************************************************************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
		{
			if(pSPIx==SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if (pSPIx==SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if(pSPIx==SPI3)
			{
				SPI3_PCLK_EN();
			}

		}
		else if (EnorDi==DISABLE)
		{
			if(pSPIx==SPI1)
			{
				SPI1_PCLK_DI();
			}
			else if (pSPIx==SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if(pSPIx==SPI3)
			{
				SPI3_PCLK_DI();
			}

		}

}
/***********************************************************************************************************************
 * @fn				--Explaining the funciton name
 *
 * @brief			--Eplaining the  funtionality of the API
 *
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 *
 * @return type		--return type of the function and how it should be handled
 *
 * @Note 			--any special requirements or instructions that the user needs to follow while using this API
 ***********************************************************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SSI);
	}
	else if(EnorDi==DISABLE)
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
	}
}
/***********************************************************************************************************************
 * @fn				--Explaining the funciton name
 *
 * @brief			--Eplaining the  funtionality of the API
 *
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 *
 * @return type		--return type of the function and how it should be handled
 *
 * @Note 			--any special requirements or instructions that the user needs to follow while using this API
 ***********************************************************************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
	}
	else if(EnorDi==DISABLE)
	{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
	}
}
/***********************************************************************************************************************
 * @fn				--Explaining the funciton name
 *
 * @brief			--Eplaining the  funtionality of the API
 *
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 *
 * @return type		--return type of the function and how it should be handled
 *
 * @Note 			--any special requirements or instructions that the user needs to follow while using this API
 ***********************************************************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	//Configuration of the CR1 register
	uint32_t tempreg=0;

	//1. Configure the device mode
	tempreg |=pSPIHandle->SPIConfig.SPI_DeviceMode<<SPI_CR1_MSTR;

	//2. Configure the bus configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_FD)
	{
		//BIDI Mode should be cleared
		tempreg &= ~(1<<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_HD)
	{
		//BIDI mode should be equal to 1 and BIDI OE would decide the direction of transfer of the data
		tempreg |= (1<<15);

	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_S_RXONLY)
	{
		//BIDI Mode should be Cleared and the RXonly bit needs to be set
		tempreg &= ~(1<<15);
		//RXonly bit configuration
		tempreg |=(1<<10);
	}

	//Configuration of the speed by configuring the baudrate frequency value
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed<<SPI_CR1_BR;

	//Configuration of the Data frame format for the SPI peripheral
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF<<SPI_CR1_DFF;

	//Configuration of the CPOL of the SPI
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL<<SPI_CR1_CPOL;

	//Configuration of the CPHA of the SPI
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA<<SPI_CR1_CPHA;

	//Configuration of the SSM  bit for software select management.
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM<<SPI_CR1_SSM;


	//Using all the configurations catched above in the CR1 Register to configure the SPI
	pSPIHandle->pSPIx->CR1 =tempreg;
}

/***********************************************************************************************************************
 * @fn				--Explaining the funciton name
 *
 * @brief			--Eplaining the  funtionality of the API
 *
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 *
 * @return type		--return type of the function and how it should be handled
 *
 * @Note 			--any special requirements or instructions that the user needs to follow while using this API
 ***********************************************************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx==SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx==SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx==SPI3)
	{
		SPI3_REG_RESET();
	}
}

/***********************************************************************************************************************
 * @fn				--Explaining the funciton name
 *
 * @brief			--Eplaining the  funtionality of the API
 *
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 *
 * @return type		--return type of the function and how it should be handled
 *
 * @Note 			--any special requirements or instructions that the user needs to follow while using this API
 ***********************************************************************************************************************/

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;

}

/***********************************************************************************************************************
 * @fn				--SendData
 *
 * @brief			--BLOCKING CALL API FOR SENDING THE DATE OVER THE SPI
 *
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 *
 * @return type		--return type of the function and how it should be handled
 *
 * @Note 			--any special requirements or instructions that the user needs to follow while using this API
 ***********************************************************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while(len>0)
	{
		//Wait until TXE bit is set below statement hangs until txe ==1
		//one way ->while(!(pSPIx->SR &(1<<1)));
		//2nd way -> Implementing a function which gives us the status of the flag
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)==FLAG_RESET);

		//Check the DFF bit
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			// Load 16 bit data into the DATA Register
			pSPIx->DR= *((uint16_t*)pTxBuffer);
			//Decrementing the data by two length
			len--;
			len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//Load the data register with 8 bits of data
			pSPIx->DR= *pTxBuffer;
			//Decrementing the data by one.
			len--;
			pTxBuffer++;
		}

	}

}
/***********************************************************************************************************************
 * @fn				--Explaining the funciton name
 *
 * @brief			--Eplaining the  funtionality of the API
 *
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 *
 * @return type		--return type of the function and how it should be handled
 *
 * @Note 			--any special requirements or instructions that the user needs to follow while using this API
 ***********************************************************************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}
	else if(EnorDi==DISABLE)
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}
/***********************************************************************************************************************
 * @fn				--Explaining the funciton name
 *
 * @brief			--Eplaining the  funtionality of the API
 *
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 *
 * @return type		--return type of the function and how it should be handled
 *
 * @Note 			--any special requirements or instructions that the user needs to follow while using this API
 ***********************************************************************************************************************/
void SPI_ReceiveDate(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
		while(len>0)
		{
			//Wait until TXE bit is set below statement hangs until txe ==1
			//one way ->while(!(pSPIx->SR &(1<<1)));
			//2nd way -> Implementing a function which gives us the status of the flag
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)==FLAG_RESET);

			//Check the DFF bit
			if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
			{
				// Load 16 bit data into the D RXBufferAddress
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				len--;
				len--;
				(uint16_t*)pRxBuffer++;
			}
			else
			{
				//Load the data register with 8 bits of data
				*pRxBuffer = pSPIx->DR;
				len--;
				pRxBuffer++;
			}

		}
}

/***********************************************************************************************************************
 * @fn				--Explaining the funciton name
 *
 * @brief			--Eplaining the  funtionality of the API
 *
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 *
 * @return type		--return type of the function and how it should be handled
 *
 * @Note 			--any special requirements or instructions that the user needs to follow while using this API
 ***********************************************************************************************************************/
void 		SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi==ENABLE)
	{
		if(IRQNumber<=31)
		{
			//Program the ISER0 register
			*NVIC_ISER0 |=(1<<IRQNumber);

		}
		else if (IRQNumber>31 && IRQNumber<64)
		{
			//Program the ISER1 register
			*NVIC_ISER1 |=(1<<( IRQNumber%32));

		}
		else if (IRQNumber >=64 && IRQNumber<96)
		{
			//Program the ISER2 register
			*NVIC_ISER2 |=(1<<(IRQNumber%32));

		}
	}
	else
	{
		if(IRQNumber<=31)
		{
			//Program the ICER0 register
			*NVIC_ICER0 |=(1<<IRQNumber);

		}
		else if (IRQNumber>31 && IRQNumber<64)
		{
			//Program the ICER1 Register
			*NVIC_ICER1 |=(1<<IRQNumber%32);

		}
		else if (IRQNumber >=64 && IRQNumber<96)
		{
			//Program the ICER2 Register
			*NVIC_ICER2 |=(1<<IRQNumber%32);


		}
	}

}
/***********************************************************************************************************************
 * @fn				--Explaining the funciton name
 *
 * @brief			--Eplaining the  funtionality of the API
 *
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 *
 * @return type		--return type of the function and how it should be handled
 *
 * @Note 			--any special requirements or instructions that the user needs to follow while using this API
 ***********************************************************************************************************************/
void 		SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

	//First we find out the IRP register we are looking for
	uint8_t iprx= IRQNumber/4;								//50/4=12 so the IPRregister IPR12
	uint8_t iprx_section= IRQNumber%4;						//50%4=2  so the 2*8 we have to start from bit 16-23

	uint8_t shift_amount= (8 * iprx_section)+ (8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR + iprx)	|=	(IRQPriority<< shift_amount);

}
/***********************************************************************************************************************
 * @fn				--Explaining the funciton name
 *
 * @brief			--Eplaining the  funtionality of the API
 *
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 *
 * @return type		--return type of the function and how it should be handled
 *
 * @Note 			--any special requirements or instructions that the user needs to follow while using this API
 ***********************************************************************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state =pSPIHandle->TxState;

	if(state!=SPI_BUSY_IN_TX)
	{
		//1.Save the tx buffer address and Len Information in som global variable;
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen=len;

		//2.Mark the SPI state as busy in transmission so that
		//no other code can take over same SPI Peripheral until transmission is over;
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE Control bit to get the interrupt whenever TXE flag is set in the ISR
		pSPIHandle->pSPIx->CR2 |=(1 << SPI_CR2_TXEIE);

		//4. Data transmission will be handled by the  ISR code written later
	}

	return state;

}
/***********************************************************************************************************************
 * @fn				--Explaining the funciton name
 *
 * @brief			--Eplaining the  funtionality of the API
 *
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 *
 * @return type		--return type of the function and how it should be handled
 *
 * @Note 			--any special requirements or instructions that the user needs to follow while using this API
 ***********************************************************************************************************************/
uint8_t SPI_ReceiveDateIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{

	uint8_t state =pSPIHandle->RxState;

	if(state!=SPI_BUSY_IN_RX)
	{
		//1.Save the Rx buffer address and Len Information in som global variable;
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen=len;

		//2.Mark the SPI state as busy in Reception so that
		//no other code can take over same SPI Peripheral until transmission is over;
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE Control bit to get the interrupt whenever RXNEIE flag is set in the ISR
		pSPIHandle->pSPIx->CR2 |=(1 << SPI_CR2_RXNEIE);

		//4. Data transmission will be handled by the  ISR code written later
	}

	return state;
}
/***********************************************************************************************************************
 * @fn				--Explaining the funciton name
 *
 * @brief			--Eplaining the  funtionality of the API
 *
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 * @Param[in]		--parameters or the argument values
 *
 * @return type		--return type of the function and how it should be handled
 *
 * @Note 			--any special requirements or instructions that the user needs to follow while using this API
 ***********************************************************************************************************************/
void 		SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp1,temp2;

	//First let us check for TXE
	temp1= pSPIHandle->pSPIx->SR & (1<<SPI_SR_TXE);
	temp2= pSPIHandle->pSPIx->CR2 &(1<<SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle for TXE
		spi_txe_interrupt_handle();
	}

	//Second let us check for RXNE Flag
	temp1= pSPIHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
	temp2= pSPIHandle->pSPIx->CR2 &(1<<SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle for RXNE
		spi_rxne_interrupt_handle();
	}

	//to implement the overrun error handling scheme

	temp1= pSPIHandle->pSPIx->SR & (1<<SPI_SR_OVR);
	temp2= pSPIHandle->pSPIx->CR2 &(1<<SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		spi_ovr_err_interrupt_handle();
	}

}

//Helper function implmentations
static void	spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Check the DFF bit
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		// Load 16 bit data into the DATA Register
		pSPIHandle->pSPIx->DR= *((uint16_t*)pSPIHandle->pTxBuffer);
		//Decrementing the data by two length
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//Load the data register with 8 bits of data
		pSPIHandle->pSPIx->DR= *pSPIHandle->pTxBuffer;
		//Decrementing the data by one.
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen)
	{
		//Txlength is zero so we have to close the SPI Peripheral Communication and inform the application that Tranmission
		// is over
		//Disable the TXE interrupts from further occurring
		pSPIHandle->pSPIx->CR2	&= ~(1<<SPI_CR2_TXEIE);
		pSPIHandle->pTxBuffer=NULL;
		pSPIHandle->TxLen=0;
		pSPIHandle->TxState=SPI_Ready;
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}


}
static void	spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

}
static void	spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

}
