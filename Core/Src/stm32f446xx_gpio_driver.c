#include <stm32f446xx_gpio_driver.h>

/* API SUPPORTED BY THIS DRIVER */



// PERIPHERAL CLK FUNCTION

/*
 @fn			GPIO_PeriPheralClk

 @brief			THIS FUNCTION ENABLE OR DISABLE CLK

 @param[in]		BASE ADDRESS OF GPIO PERIPHERAL
 @param[in]		ENABLE OR DISABLE MACROS
 @param[in]

 @return		NONE

 @note 			NONE


 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{

	    if (EnorDi == ENABLE) {															/**/
	        if (pGPIOx == GPIOA) {
	            GPIOA_PCLK_EN();
	        } else if (pGPIOx == GPIOB) {
	            GPIOB_PCLK_EN();
	        } else if (pGPIOx == GPIOC) {
	            GPIOC_PCLK_EN();
	        } else if (pGPIOx == GPIOD) {
	            GPIOD_PCLK_EN();
	        } else if (pGPIOx == GPIOE) {
	            GPIOE_PCLK_EN();
	        } else if (pGPIOx == GPIOF) {
	            GPIOF_PCLK_EN();
	        } else if (pGPIOx == GPIOG) {
	            GPIOG_PCLK_EN();
	        } else if (pGPIOx == GPIOH) {
	            GPIOH_PCLK_EN();
	        }
	    } else {
	        if (pGPIOx == GPIOA) {
	            GPIOA_PCLK_DI();
	        } else if (pGPIOx == GPIOB) {
	            GPIOB_PCLK_DI();
	        } else if (pGPIOx == GPIOC) {
	            GPIOC_PCLK_DI();
	        } else if (pGPIOx == GPIOD) {
	            GPIOD_PCLK_DI();
	        } else if (pGPIOx == GPIOE) {
	            GPIOE_PCLK_DI();
	        } else if (pGPIOx == GPIOF) {
	            GPIOF_PCLK_DI();
	        } else if (pGPIOx == GPIOG) {
	            GPIOG_PCLK_DI();
	        } else if (pGPIOx == GPIOH) {
	            GPIOH_PCLK_DI();
	        }
	    }

}

//IN IT DEINIT FUNCTION


/* INIT
 @fn			GPIO_Init

 @brief			THIS FUNCTION USED TO CONFIGURE GPIO PIN

 @param[in]		BASE ADDRESS OF GPIO PERIPHERAL
 @param[in]
 @param[in]

 @return		NONE

 @note 			NONE


 */


void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
	//1. CONFIGURE THE MODE OF GPIO PIN
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<< (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;


	}else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// CONFIGURE THE FTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding bit RTSR
		    EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// CONFIGURE THE RTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding bit RTSR
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// CONFIGURE THE BOTH FTSR & RTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding bit RTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		// 2. CONFIGURE THE GPIO PORT SELECTION FOR SYSCFG_EXTICR.
		uint8_t temp1 = 1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = 1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);


		// 3. ENABLE THE EXTI INTERRUPT DELIVERY USING IMR.
		EXTI->IMR |= 1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	}

	temp = 0;

	//2. CONFIGURE THE SPEED
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;


	temp = 0;
	//3. CONFIGURE THE PUPD SETTING
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	//4. CONFIGURE THE OP TYPE

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. CONFIGURE THE ALT FUNCTIONALITY
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
	uint8_t temp1, temp2;

	temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
	temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
	pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
	pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}

}

/* DEINIT

 @fn			GPIO_DeInit

 @brief			THIS FUNCTION USED TO RESET/DISABLE THE CONFIGURE GPIO PIN

 @param[in]		BASE ADDRESS OF GPIO PERIPHERAL
 @param[in]
 @param[in]

 @return		NONE

 @note 			NONE

 */

void GPIT_DeInit(GPIO_RegDef_t *pGPIOx)
{
	            if (pGPIOx == GPIOA) {
		            GPIOA_REG_RESET();
		        }
	            else if (pGPIOx == GPIOB) {
		            GPIOB_REG_RESET();
		        }
	            else if (pGPIOx == GPIOC) {
		        	GPIOC_REG_RESET();
		        }
	            else if (pGPIOx == GPIOD) {
		        	GPIOD_REG_RESET();
		        }
	            else if (pGPIOx == GPIOE) {
		        	GPIOE_REG_RESET();
		        }
	            else if (pGPIOx == GPIOF) {
		        	GPIOF_REG_RESET();
		        }
	            else if (pGPIOx == GPIOG) {
		        	GPIOG_REG_RESET();
		        }
	            else if (pGPIOx == GPIOH) {
		        	GPIOH_REG_RESET();
		        }

}


// READ/WRITE FUNCTION

 /*

 @fn            		GPIO_ReadFromInputPin

 @brief         		THIS FUNCTION READS THE VALUE FROM A SPECIFIED GPIO INPUT PIN

 @param[in] pGPIOx : 	BASE ADDRESS OF THE GPIO PERIPHERAL
 @param[in] PinNumber : PIN NUMBER TO BE READ (0 TO 15)
 @param[in]

 @return        		0 | 1
 @note     				NONE

*/


uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber)& 0x00000001);

	return value;
}

/*

@fn            		    GPIO_ReadFromInputPort

@brief         		    THIS FUNCTION READS THE VALUE FROM A SPECIFIED GPIO INPUT PORT

@param[in] pGPIOx : 	BASE ADDRESS OF THE GPIO PERIPHERAL
@param[in]
@param[in]

@return        		NONE
@note     			NONE

*/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/*
 @fn            		GPIO_WriteToOutputPin

 @brief                 THIS FUNCTION SETS OR CLEARS A SPECIFIED GPIO OUTPUT PIN

 @param[in] pGPIOx :    BASE ADDRESS OF THE GPIO PERIPHERAL
 @param[in] PinNumber : PIN NUMBER TO BE WRITTEN (0 TO 15)
 @param[in] Value :     VALUE TO WRITE (0 FOR LOW, 1 FOR HIGH)

 @return        NONE

 @note      	NONE
*/


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		// write 1 to odr at the bit field corresponding to the PinNumber
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		// write 0 to odr at the bit field corresponding to the PinNumber
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*
 @fn                 GPIO_WriteToOutputPort

 @brief              THIS FUNCTION WRITES A VALUE TO THE ENTIRE GPIO OUTPUT PORT

 @param[in] pGPIOx : BASE ADDRESS OF THE GPIO PERIPHERAL
 @param[in] Value :  16-BIT VALUE TO BE WRITTEN TO THE PORT
 @param[in]

 @return             NONE

 @note      		 NONE
*/


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*
 @fn            	    GPIO_ToggleOutputPin

 @brief        		    THIS FUNCTION TOGGLES THE STATE OF A SPECIFIED GPIO OUTPUT PIN

 @param[in] pGPIOx :    BASE ADDRESS OF THE GPIO PERIPHERAL
 @param[in] PinNumber : PIN NUMBER TO BE TOGGLED (0 TO 15)
 @param[in]

 @return        		NONE

 @note        			NONE
*/


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}

// IRQ FUNCTION

/*

 @fn            		GPIO_IRQHandling

 @brief         		THIS FUNCTION HANDLES THE INTERRUPT REQUEST FOR A SPECIFIED GPIO PIN

 @param[in] PinNumber : PIN NUMBER THAT TRIGGERED THE INTERRUPT (0 TO 15)

 @return        		NONE

 @note      			NONE

*/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)

{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// Program ISER0.
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) // 32 to 63
		{
			// Program ISER1
			*NVIC_ISER1 |= (1<<IRQNumber % 32);
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ISER2 // 64 to 95
			*NVIC_ISER2 |= (1<<IRQNumber % 64);
		}
		else
		{
			if(IRQNumber <= 31)
			{
				// Program ICER0
				*NVIC_ICER0 |= (1<<IRQNumber);
			}
			else if(IRQNumber > 31 && IRQNumber < 64)
			{
				// Program ICER1
				*NVIC_ICER1 |= (1<<IRQNumber % 32);
			}
			else if(IRQNumber >= 64 && IRQNumber < 96)
			{
				// Program ICER2
				*NVIC_ICER2 |= (1<<IRQNumber % 64);
			}
		}
	}
}


/*

 @fn            		GPIO_IRQHandling

 @brief         		THIS FUNCTION HANDLES THE INTERRUPT REQUEST FOR A SPECIFIED GPIO PIN

 @param[in] PinNumber : PIN NUMBER THAT TRIGGERED THE INTERRUPT (0 TO 15)

 @return        		NONE

 @note      			NONE

*/


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. First lets find out the ipr register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;


	uint8_t shift_amount = (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);

}

/*
 @fn            		GPIO_IRQConfig

 @brief     			THIS FUNCTION CONFIGURES THE INTERRUPT FOR A SPECIFIED GPIO PIN

 @param[in] IRQNumber : IRQ NUMBER AS DEFINED IN THE MCU'S VECTOR TABLE
 @param[in] EnorDi : 	ENABLE OR DISABLE MACRO (ENABLE = 1, DISABLE = 0)
 @param[in]

 @return       			NONE

 @note      			NONE
*/


void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1<<PinNumber))
	{
		//Clear
		EXTI->PR |= (1<<PinNumber);
	}
}
