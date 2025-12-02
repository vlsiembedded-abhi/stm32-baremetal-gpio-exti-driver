

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

#define __vo volatile
/*
 * Processor Specific Detail
 */

/*
 * ARM Cortex M4 Processor NVIC ISERx Register Address
 */

#define NVIC_ISER0						 (__vo uint32_t*)0xE000E100
#define NVIC_ISER1						 (__vo uint32_t*)0xE000E104
#define NVIC_ISER2						 (__vo uint32_t*)0xE000E108
#define NVIC_ISER3						 (__vo uint32_t*)0xE000E10C

/*
 * ARM Cortex M4 Processor NVIC ICERx Register Address
 */

#define NVIC_ICER0						 (__vo uint32_t*)0XE000E180
#define NVIC_ICER1						 (__vo uint32_t*)0XE000E184
#define NVIC_ICER2						 (__vo uint32_t*)0XE000E188
#define NVIC_ICER3						 (__vo uint32_t*)0XE000E18C

/*
 * ARM Cortex M priority Register
 */

#define NVIC_PR_BASE_ADDR				(__vo uint32_t*)0xE000E400

#define NO_PR_BITS_IMPLEMENTED			4


#define FLASH_BASEADDR					0x08000000U		/* IT IS THE BASE ADDRESS OF FLASH MEMORY */

#define SRAM1_BASEADDR					0x20000000U 	/* THIS IS THE BASE ADDRESS OF SRAM1, SIZE OF SRAM1 = 112KB */
#define SRAM2_BASEADDR					0x2001C000U		/* THIS IS THE BASE ADDRESS OF SRAM2 WHICH IS JUST AFTER 112KB, SIZE OF SRAM2 = 16KB */
#define ROM_BASEADDR					0x1FFF0000U     /* THIS ADDRESS IF OF SYSTEM MEMORY, THE 2ND NAME OF ROM */
#define SRAM 							SRAM1_BASEADDR

#define PERIPH_BASE						0x40000000U     /*OCCUPIED BY TIMMER 2 CR REGISTER */

// APB BUS IS USED FOR THOSE PERIPHERAL WHICH WORK ON LOW SPEED COMMUNICATION.

#define APB1PERIPH_BASE 				PERIPH_BASE
#define APB2PERIPH_BASE					0x40010000U

// AHB BUS IS USED FOR THOSE PERIPHERAL WHICH NEED HIGH SPEED DATA COMMUNICATION.

#define AHB1PERIPH_BASE					0x40020000U
#define AHB2PERIPH_BASE					0x50000000U
#define AHB3PERIPH_BASE					0xA0000000U								/**1**/

// BASE ADDRESS OF AHB1 PERIPHERAL WHICH ARE HANGING TO AHB1 BUS.

#define GPIOA_BASEADDR					(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASE + 0x1C00)
#define RCC_BASEADDR					(AHB1PERIPH_BASE + 0x3800)

// BASE ADDRESS OF APB1 PERIPHERAL WHICH ARE HANGING TO APB 1 BUS.

#define I2C1_BASEADDR					(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR					(APB1PERIPH_BASE + 0X3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASE + 0X3C00)

#define USART2_BASEADDR					(APB1PERIPH_BASE + 0X4400)
#define USART3_BASEADDR					(APB1PERIPH_BASE + 0X4800)
#define UART4_BASEADDR					(APB1PERIPH_BASE + 0X4C00)
#define UART5_BASEADDR					(APB1PERIPH_BASE + 0X5000)

// BASE ADDRESS OF APB2 PERIPHERAL WHICH ARE HANGING TO APB 2 BUS.

#define SPI1_BASEADDR					(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR					(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR					(APB2PERIPH_BASE + 0x1400)
#define EXTI_BASEADDR					(APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR					(APB2PERIPH_BASE + 0x3800)

// USING STRUCT FOR PERIPHERAL RESISTER

typedef struct
{
	__vo uint32_t MODER;      /*GPIO port mode register, OffSet: 0x00 */
	__vo uint32_t OTYPER;	  /*GPIO port output type register, OffSet: 0x04 */
	__vo uint32_t OSPEEDER;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];	  /* AFR[0]: GPIO alternate function low register, AFR[1]:  GPIO alternate function high register, OffSet: 0x20 */

}GPIO_RegDef_t;

// RCC DEFINITION

typedef struct
{
	__vo uint32_t CR; 	             /*RCC clock control register, OffSet: 0x00*/
	__vo uint32_t PLLCFGR; 	         /*RCC PLL configuration register, OffSet: 0x04*/
	__vo uint32_t CFGR; 	         /*RCC clock configuration register, OffSet: 0x08*/
	__vo uint32_t CIR; 	             /*RCC clock interrupt register, OffSet: 0x0C*/
	__vo uint32_t AHB1RSTR; 	     /*RCC AHB1 peripheral reset register, OffSet: 0x10*/
	__vo uint32_t AHB2RSTR; 	     /*RCC AHB2 peripheral reset register, OffSet: 0x14*/
	__vo uint32_t AHB3RSTR; 	     /*RCC AHB3 peripheral reset register, OffSet: 0x18*/

	__vo uint32_t RESERVED0;		 /* RESERVED, OffSet: 0x1C */

	__vo uint32_t APB1RSTR; 	     /*RCC APB1 peripheral reset register, OffSet: 0x20*/
	__vo uint32_t APB2RSTR; 	     /* RCC APB2 peripheral reset register, OffSet: 0x24*/

	__vo uint32_t RESERVED1[2];		 /* RESERVED, OffSet[0]: 0x28, OffSet[1]: 0x2C */

	__vo uint32_t AHB1ENR; 	         /* RCC AHB1 peripheral clock enable register, OffSet: 0x30*/
	__vo uint32_t AHB2ENR; 	     	 /* RCC AHB2 peripheral clock enable register, OffSet: 0x34*/
	__vo uint32_t AHB3ENR; 	     	 /* RCC AHB3 peripheral clock enable register, OffSet: 0x38*/

	__vo uint32_t RESERVED2;		 /* RESERVED, OffSet: 0x3C */

	__vo uint32_t APB1ENR; 	     	 /* RCC APB1 peripheral clock enable register, OffSet: 0x40*/
	__vo uint32_t APB2ENR; 	     	 /* RCC APB2 peripheral clock enable register, OffSet: 0x44*/

	__vo uint32_t RESERVED3[2];		 /* RESERVED, OffSet[0]: 0x48, OffSet[1]: 0x4C */

	__vo uint32_t AHB1LPENR; 	     /* RCC AHB1 peripheral clock enable in low power mode register, OffSet: 0x50*/
	__vo uint32_t AHB2LPENR; 	     /* RCC AHB2 peripheral clock enable in low power mode register, OffSet:  0x54*/
	__vo uint32_t AHB3LPENR; 	     /* RCC AHB3 peripheral clock enable in low power mode register, OffSet: 0x58*/

	__vo uint32_t RESERVED4;		 /* RESERVED, OffSet: 0x5C */

	__vo uint32_t APB1LPENR; 	     /* RCC APB1 peripheral clock enable in low power mode register, OffSet: 0x60*/
	__vo uint32_t APB2LPENR; 	     /* RCC APB2 peripheral clock enable in low power mode register, OffSet: 0x64*/

	__vo uint32_t RESERVED5[2];		 /* RESERVED, OffSet[0]: 0x68, OffSet[1]: 0x6C */

	__vo uint32_t BDCR;	             /* RCC Backup domain control register, OffSet: 0x70*/
	__vo uint32_t CSR;	             /* RCC clock control and status register, OffSet: 0x74*/

	__vo uint32_t RESERVED6[2];		 /* RESERVED, OffSet[0]: 0x78, OffSet[1]: 0x7C */

	__vo uint32_t SSCGR;	         /* RCC spread spectrum clock generation register, OffSet: 0x80*/
	__vo uint32_t PLLI2SCFGR;	     /* RCC PLLI2S configuration register, OffSet: 0x84*/
	__vo uint32_t PLLSAICFGR;	     /* RCC PLL configuration register, OffSet: 0x88*/
	__vo uint32_t DCKCFGR;	         /* RCC dedicated clock configuration register, OffSet: 0x8C*/
	__vo uint32_t CKGATENR;	      	 /* RCC clocks gated enable register, OffSet: 0x90*/
	__vo uint32_t DCKCFGR2;	     	 /* RCC dedicated clocks configuration register 2, OffSet: 0x94*/

}RCC_RegDef_t;

// USING STRUCT FOR PERIPHERAL EXTI

typedef struct
{
	__vo uint32_t IMR;        /*Interrupt Mask Register, OffSet: 0x00 */
	__vo uint32_t EMR;	      /*Event Mask register, OffSet: 0x04 */
	__vo uint32_t RTSR;       /* Rising Trigger Section Register 0x08*/
	__vo uint32_t FTSR;	      /* Falling Trigger Section Register 0x0C*/
	__vo uint32_t SWIER;      /* Software Interrupt Event Register 0x10*/
	__vo uint32_t PR;		  /* Pending Register 0x14*/


}EXTI_RegDef_t;

/* Definition of SYSCFG register */

typedef struct
{
	__vo uint32_t MEMRMP;        /*Memory Remap Register, OffSet: 0x00 */
	__vo uint32_t PMC;	      /*Peripheral Mode register, OffSet: 0x04 */
	__vo uint32_t EXTICR[4];       /* External Interrupt Configuration, OffSet CR1: 0x08, OffSet CR2: 0x0C, OffSet CR3: 0x10, OffSet CR4: 0x14  */
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;	      /* Compensation Cell Control Register 0x20*/
	uint32_t RESERVED2[2];
	__vo uint32_t CFGR;      /* Configure Register 0x2C*/

}SYSCFG_RegDef_t;


/* Peripheral definition( type casted GPIO base address to struct */

#define GPIOA					( (GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB					( (GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC					( (GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD					( (GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE					( (GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF					( (GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG					( (GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH					( (GPIO_RegDef_t*) GPIOH_BASEADDR)

/* Peripheral definition FOR RCC */

#define RCC						( (RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI 					( (EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG					( (SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

/*GPIO PORT CLK ENABLE*/

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1<<7))

/* CLK ENABLE MACRO FOR I2Cx */

#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1<<23))

/* CLK ENABLE MACRO FOR SPIx */

#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1<<13))

/* CLK ENABLE MACRO FOR USART */

#define USART1_PCLK_EN()				(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()				(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()				(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()					(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()					(RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()				(RCC->APB2ENR |= (1<<5))

/* CLK ENABLE MACRO FOR SYSCFG */

#define SYSCFG_PCLK_EN()				(RCC->APB2ENR |= (1<<14))

/*CLK DISABLE MACRO FOR GPIOX */

#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<7))

/*CLK DISABLE MACRO FOR SPIX */

#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()				(RCC->APB2ENR &= ~(1<<13))

/*CLK DISABLE MACRO FOR USARTX */

#define USART1_PCLK_DI()				(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()				(RCC->APB2ENR &= ~(1<<5))

/*CLK DISABLE MACRO FOR SYSCFG */

#define SYSCFG_PCLK_DI()				(RCC->APB2ENR &= ~(1<<14))

/* GPIO RESET REG */
#define GPIOA_REG_RESET()						do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()						do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()						do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()						do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()						do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOF_REG_RESET()  					    do{ (RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5)); }while(0)
#define GPIOG_REG_RESET()						do{ (RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6)); }while(0)
#define GPIOH_REG_RESET()						do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7)); }while(0)

#define GPIO_BASEADDR_TO_CODE(x)			(	(x == GPIOA)?0:\
												(x == GPIOB)?1:\
												(x == GPIOC)?2:\
												(x == GPIOD)?3:\
												(x == GPIOE)?4:\
												(x == GPIOF)?5:\
												(x == GPIOG)?6:\
												(x == GPIOH)?7:0	)

/*
 * IRQ(Interrupt Request) Number Of STM32F446Rxx MCU
 * Note: Update these macro with valid value according MCU
 * TODO: You May Complete this list for other peripheral
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/*
 * IRQ Priority Configuration
 */

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15


/* SOME GENERIC MACROS */

#define ENABLE 				1
#define DISABLE             0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#include <stm32f446xx_gpio_driver.h>


#endif /* INC_STM32F446XX_H_ */
