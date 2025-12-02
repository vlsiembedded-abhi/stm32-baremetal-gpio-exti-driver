
#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include <stm32f446xx.h>

typedef struct /* THIS IS THE LIST CONFIGURABLE ITEM FOR USER APPLICATION */
{
	uint8_t GPIO_PinNumber;		 /* POSSIBLE VALUE FROM @GPIO_PIN_NUMBER */
	uint8_t GPIO_PinMode;			/* POSSIBLE VALUE FROM @GPIO_PIN_MODE */
	uint8_t GPIO_PinSpeed;			/* POSSIBLE VALUE FROM @GPIO_SPEED */
	uint8_t GPIO_PinPuPdControl; /* POSSIBLE VALUE FROM @GPIO_PULLUP_PULLDOWN */
	uint8_t GPIO_PinOPType;		 /* POSSIBLE VALUE FROM @GPIO_OUTPUT_TYPE */
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

typedef struct
{
	// declare a pointer to hold the base address of gpio peripheral
	GPIO_RegDef_t *pGPIOx; /* this is hold the base address of gpio port at which pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


/***
 GPIO_PIN_NUMBER
 ***/

#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/***
 GPIO PIN POSSIBLE MODE
 ***/

// GPIO_PIN_MODE
#define GPIO_MODE_IN	 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4		// FALL TRIGGER
#define GPIO_MODE_IT_RT  5      // RISING TRIGGER
#define GPIO_MODE_IT_RFT 6		// RISING FALL TRIGGER

// GPIO_OUTPUT_TYPE
#define GPIO_OP_TYPE_PP     0   // PUSH PULL
#define GPIO_OP_TYPE_OD	 1   // OPEN DRAIN

// GPIO_SPEED
#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_FAST     2
#define GPIO_SPEED_HIGH     3

// GPIO_PULLUP_PULLDOWN
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU		    1
#define GPIO_PIN_PD 		2

/* API SUPPORTED BY THIS DRIVER */

//IN IT DEINIT FUNCTION

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// PERIPHERAL CLK FUNCTION

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

// READ/WRITE FUNCTION

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// IRQ FUNCTION

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
