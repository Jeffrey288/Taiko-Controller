#ifndef BUTTON_H
#define BUTTON_H

#include "stm32f1xx_hal.h"

#define BTN_PAD_R1_PORT	GPIOE
#define BTN_PAD_R1_PIN	GPIO_PIN_0
#define BTN_PAD_R2_PORT GPIOE
#define BTN_PAD_R2_PIN 	GPIO_PIN_1
#define BTN_PAD_R3_PORT	GPIOE
#define BTN_PAD_R3_PIN 	GPIO_PIN_2
#define BTN_PAD_R4_PORT GPIOE
#define BTN_PAD_R4_PIN	GPIO_PIN_3
#define BTN_PAD_C1_PORT GPIOE
#define BTN_PAD_C1_PIN	GPIO_PIN_4
#define BTN_PAD_C2_PORT	GPIOE
#define BTN_PAD_C2_PIN	GPIO_PIN_5
#define BTN_PAD_C3_PORT	GPIOE
#define BTN_PAD_C3_PIN	GPIO_PIN_6
#define BTN_PAD_C4_PORT	GPIOC
#define BTN_PAD_C4_PIN	GPIO_PIN_13

void ButtonPadInit();
int ButtonPadCallback();

#endif // BUTTON_H
