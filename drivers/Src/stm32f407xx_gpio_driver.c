/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jul 30, 2022
 *      Author: syedsuhailsimnani
 */
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

#include "stm32f407xx_gpio_driver.h"

/***********************************************************************************************************************
 * @fn				--GPIO_PeriClockControl      //GPIO peripheral clock enable function
 *
 * @brief			--This function enables or disbles the clock for the particular GPIO port
 *
 * @Param[1]		--GPIO_RegDef_t *pGPIOx		//Defines the base address of the port
 * @Param[2]		--uint8_t EnorDI			//Enables or Disables the clock for thar particular port
 *
 * @return type		--return type void
 *
 * @Note 			--Please donot enable the power for the GPIO ports which are not in use as this may increase the utilisation
 ***********************************************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{

	if(EnorDi==ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx==GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx==GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else if (EnorDi==DISABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx==GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx==GPIOI)
		{
			GPIOI_PCLK_DI();
		}

	}
}


/***********************************************************************************************************************
 * @fn				--GPIO initilisation
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
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	uint32_t temp=0;

	//Configure mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//Configuration of the non interrupt mode of the GPIO pins
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER	&=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER 	|= temp;
	}
	else
	{
		//Configuration of the interrupt mode of the GPIO pins
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1.Configure the falling edge selection register(FTSR)
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clearing the RTSR bits so that if previous configured could be erased
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1.Configure the Rise edge selection register (RTSR)
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clearing the FTSR bits so that if configured previously could be cleared here
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1.Configure both falling and rising interrupt trigger resgister.
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//2. Configure the GPIO Port selection  in the SYSCFG_EXTICR register
		uint8_t temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portCode= GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portCode<< (temp2*4);

		//3. Enable the EXTI interrupt delivery using the IMR
		EXTI->IMR	|=	1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;


	}

	temp=0;
	//Configure the speed of GPIO pin
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR		&= 	~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR		|=	temp;


	temp=0;
	//Configure the pull up and pull down functionality of the GPIO pin
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR			&=	~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //CLearing the bit position
	pGPIOHandle->pGPIOx->PUPDR			|=	temp;

	temp=0;
	//COnfigure the output type of the gpio pin
    temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<(1*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OTYPER			&=	~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER			|=	temp;

    temp=0;
	//Configure the alternate functionality of the GPIO pin
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN)
    {
    	//configure the alternate function registers
    	uint8_t temp1, temp2;
    	temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
    	temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
    	pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF<<(4*temp2));
    	pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*temp2));
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
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	    if(pGPIOx==GPIOA)
		{
			GPIOA_REG_RESET();
		}
		else if (pGPIOx==GPIOB)
		{
			GPIOB_REG_RESET();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_REG_RESET();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_REG_RESET();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_REG_RESET();
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_REG_RESET();
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_REG_RESET();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_REG_RESET();
		}
		else if (pGPIOx==GPIOI)
		{
			GPIOI_REG_RESET();
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
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR  >> PinNumber) & 0x00000001);
	return  value;
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
uint16_t 	GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
		uint16_t value;
		value = (uint16_t)(pGPIOx->IDR);
		return  value;
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
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
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
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR=Value;
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
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);

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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn				--GPIO_IRQPriorityConfig
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the extiPR register corresponding to the Pin Number
	if(EXTI->PR & (1<<PinNumber))
	{
		//Clear that pending register bit
		EXTI->PR|=(1<<PinNumber);
	}
}
