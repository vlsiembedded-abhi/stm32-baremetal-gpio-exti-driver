# stm32-baremetal-gpio-exti-driver
This repository contains a custom GPIO driver for the STM32F446RE microcontroller using bare-metal register-level programming, along with example projects demonstrating LED control and external interrupt handling.

🚀 Included Projects
🔹 001Led_toggle.c

Simple LED blinking using the custom GPIO driver.

Configures PA5 as output

Toggles LED using GPIO_ToggleOutputPin()

Tests clock enable, MODER, and ODR operations

🔹 External Button Interrupt (EXTI) – 003LED_BUTTON_INTERRUPT.c

LED toggles on button press using external interrupt, not polling.

Configures GPIO pin as EXTI input

Selects rising/falling/both-edge trigger

Routes GPIO → EXTI line using SYSCFG

Enables interrupt in NVIC

Handles interrupt in GPIO_IRQHandling()

Demonstrates EXTI pending flag clearing and ISR execution

📁 Driver Files
stm32f446xx.h

MCU memory map, RCC, GPIO, EXTI, SYSCFG register definitions, base addresses, and NVIC configuration.

stm32f446xx_gpio_driver.h

Driver API declarations and pin configuration structures.

stm32f446xx_gpio_driver.c

Full GPIO & EXTI driver implementation:

GPIO_Init, DeInit

Pin read/write/toggle

Clock control

EXTI configuration

NVIC interrupt enable/priority

Interrupt handler

🛠 Tools Used

STM32CubeIDE

STM32F446RE (Nucleo)

Bare-Metal Embedded C

ARM Cortex-M4 register programming

📌 Project Summary

This repo demonstrates how to build your own GPIO and EXTI driver from scratch, understand low-level peripheral registers, and implement real interrupt-driven applications without HAL or LL library.
