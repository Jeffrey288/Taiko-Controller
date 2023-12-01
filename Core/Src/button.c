#include "button.h"

GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
void ButtonPadInit() {
	HAL_GPIO_WritePin(BTN_PAD_C1_PORT, BTN_PAD_C1_PIN, 1);
	HAL_GPIO_WritePin(BTN_PAD_C2_PORT, BTN_PAD_C2_PIN, 1);
	HAL_GPIO_WritePin(BTN_PAD_C3_PORT, BTN_PAD_C3_PIN, 1);
	HAL_GPIO_WritePin(BTN_PAD_C4_PORT, BTN_PAD_C4_PIN, 1);
}

uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint16_t keyPressed = 0;
int ButtonPadCallback(int GPIO_Pin) {
	// See if it's falling or rising edge, may fail at times
	int callback_pin_value = HAL_GPIO_ReadPin(BTN_PAD_R1_PORT, GPIO_Pin);
	if (callback_pin_value == GPIO_PIN_RESET) {
		previousMillis = HAL_GetTick();
		return 0;
	}

	currentMillis = HAL_GetTick();
	keyPressed = 0;
	if (currentMillis - previousMillis > 20) {

		// Change this if the R pins are not the same
		GPIO_InitStructPrivate.Pin = BTN_PAD_R1_PIN|BTN_PAD_R2_PIN|BTN_PAD_R3_PIN|BTN_PAD_R4_PIN;
		GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
		GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(BTN_PAD_R1_PORT, &GPIO_InitStructPrivate);

		HAL_GPIO_WritePin(BTN_PAD_C1_PORT, BTN_PAD_C1_PIN, 1);
		HAL_GPIO_WritePin(BTN_PAD_C2_PORT, BTN_PAD_C2_PIN, 0);
		HAL_GPIO_WritePin(BTN_PAD_C3_PORT, BTN_PAD_C3_PIN, 0);
		HAL_GPIO_WritePin(BTN_PAD_C4_PORT, BTN_PAD_C4_PIN, 0);
		if (HAL_GPIO_ReadPin(BTN_PAD_R1_PORT, BTN_PAD_R1_PIN) && GPIO_Pin == BTN_PAD_R1_PIN) keyPressed = 16;
		if (HAL_GPIO_ReadPin(BTN_PAD_R2_PORT, BTN_PAD_R2_PIN) && GPIO_Pin == BTN_PAD_R2_PIN) keyPressed = 15;
		if (HAL_GPIO_ReadPin(BTN_PAD_R3_PORT, BTN_PAD_R3_PIN) && GPIO_Pin == BTN_PAD_R3_PIN) keyPressed = 14;
		if (HAL_GPIO_ReadPin(BTN_PAD_R4_PORT, BTN_PAD_R4_PIN) && GPIO_Pin == BTN_PAD_R4_PIN) keyPressed = 13;

		HAL_GPIO_WritePin(BTN_PAD_C1_PORT, BTN_PAD_C1_PIN, 0);
		HAL_GPIO_WritePin(BTN_PAD_C2_PORT, BTN_PAD_C2_PIN, 1);
		if (HAL_GPIO_ReadPin(BTN_PAD_R1_PORT, BTN_PAD_R1_PIN) && GPIO_Pin == BTN_PAD_R1_PIN) keyPressed = 12;
		if (HAL_GPIO_ReadPin(BTN_PAD_R2_PORT, BTN_PAD_R2_PIN) && GPIO_Pin == BTN_PAD_R2_PIN) keyPressed = 11;
		if (HAL_GPIO_ReadPin(BTN_PAD_R3_PORT, BTN_PAD_R3_PIN) && GPIO_Pin == BTN_PAD_R3_PIN) keyPressed = 10;
		if (HAL_GPIO_ReadPin(BTN_PAD_R4_PORT, BTN_PAD_R4_PIN) && GPIO_Pin == BTN_PAD_R4_PIN) keyPressed = 9;

		HAL_GPIO_WritePin(BTN_PAD_C2_PORT, BTN_PAD_C2_PIN, 0);
		HAL_GPIO_WritePin(BTN_PAD_C3_PORT, BTN_PAD_C3_PIN, 1);
		if (HAL_GPIO_ReadPin(BTN_PAD_R1_PORT, BTN_PAD_R1_PIN) && GPIO_Pin == BTN_PAD_R1_PIN) keyPressed = 8;
		if (HAL_GPIO_ReadPin(BTN_PAD_R2_PORT, BTN_PAD_R2_PIN) && GPIO_Pin == BTN_PAD_R2_PIN) keyPressed = 7;
		if (HAL_GPIO_ReadPin(BTN_PAD_R3_PORT, BTN_PAD_R3_PIN) && GPIO_Pin == BTN_PAD_R3_PIN) keyPressed = 6;
		if (HAL_GPIO_ReadPin(BTN_PAD_R4_PORT, BTN_PAD_R4_PIN) && GPIO_Pin == BTN_PAD_R4_PIN) keyPressed = 5;

		HAL_GPIO_WritePin(BTN_PAD_C3_PORT, BTN_PAD_C3_PIN, 0);
		HAL_GPIO_WritePin(BTN_PAD_C4_PORT, BTN_PAD_C4_PIN, 1);
		if (HAL_GPIO_ReadPin(BTN_PAD_R1_PORT, BTN_PAD_R1_PIN) && GPIO_Pin == BTN_PAD_R1_PIN) keyPressed = 4;
		if (HAL_GPIO_ReadPin(BTN_PAD_R2_PORT, BTN_PAD_R2_PIN) && GPIO_Pin == BTN_PAD_R2_PIN) keyPressed = 3;
		if (HAL_GPIO_ReadPin(BTN_PAD_R3_PORT, BTN_PAD_R3_PIN) && GPIO_Pin == BTN_PAD_R3_PIN) keyPressed = 2;
		if (HAL_GPIO_ReadPin(BTN_PAD_R4_PORT, BTN_PAD_R4_PIN) && GPIO_Pin == BTN_PAD_R4_PIN) keyPressed = 1;

		HAL_GPIO_WritePin(BTN_PAD_C1_PORT, BTN_PAD_C1_PIN, 1);
		HAL_GPIO_WritePin(BTN_PAD_C2_PORT, BTN_PAD_C2_PIN, 1);
		HAL_GPIO_WritePin(BTN_PAD_C3_PORT, BTN_PAD_C3_PIN, 1);
		HAL_GPIO_WritePin(BTN_PAD_C4_PORT, BTN_PAD_C4_PIN, 1);

		GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStructPrivate.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(BTN_PAD_R1_PORT, &GPIO_InitStructPrivate);
	}

	previousMillis = currentMillis;
	return keyPressed;

}
