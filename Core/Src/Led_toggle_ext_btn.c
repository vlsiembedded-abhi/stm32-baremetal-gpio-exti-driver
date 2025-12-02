#include<stm32f446xx.h>

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW
#define NO_BTN HIGH

void delay(void)
{
	for(uint32_t i = 0; i<500000/2; i++);
}


int main(void){

	GPIO_Handle_t Gpio_Led, GPIOBtn;
		Gpio_Led.pGPIOx = GPIOA;
		Gpio_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
		Gpio_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
		Gpio_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		Gpio_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		Gpio_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


		GPIO_PeriClockControl(GPIOA, ENABLE);

		GPIO_Init(&Gpio_Led);

	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GPIOBtn);
	while(1){
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_3) == BTN_PRESSED)
		{
		delay();
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_9);
		}
	}

	return 0;

}

