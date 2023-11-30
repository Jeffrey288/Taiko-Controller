/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "drum.h"
#include "kadon.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;
DMA_HandleTypeDef hdma_dac_ch2;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/****************************************************************************************************
 *
 * LCD SCREEN
 //=========================================电�?接线================================================//
 //     LCD模�?�                STM32�?�片机
 //      VCC          接        DC5V/3.3V      //电�?
 //      GND          接          GND          //电�?地
 //=======================================液晶�?数�?�线接线==========================================//
 //本模�?�默认数�?�总线类型为SPI总线
 //     LCD模�?�                STM32�?�片机
 //    SDI(MOSI)      接          PB15         //液晶�?SPI总线数�?�写信�?�
 //    SDO(MISO)      接          PB14         //液晶�?SPI总线数�?�读信�?�，如果�?需�?读，�?�以�?接线
 //=======================================液晶�?控制线接线==========================================//
 //     LCD模�?� 					      STM32�?�片机
 //       LED         接          -          //液晶�?背光控制信�?�，如果�?需�?控制，接5V或3.3V
 //       SCK         接          PB13         //液晶�?SPI总线时钟信�?�
 //      DC/RS        接          PB10         //液晶�?数�?�/命令控制信�?�
 //       RST         接          PB12         //液晶�?�?�?控制信�?�
 //       CS          接          PB11         //液晶�?片选控制信�?�
 //=========================================触摸�?触接线=========================================//
 //如果模�?��?带触摸功能或者带有触摸功能，但是�?需�?触摸功能，则�?需�?进行触摸�?接线
 //	   LCD模�?�                STM32�?�片机
 //      T_IRQ        接          PE15         //触摸�?触摸中断信�?�
 //      T_DO         接          PE14         //触摸�?SPI总线读信�?�
 //      T_DIN        接          PE13         //触摸�?SPI总线写信�?�
 //      T_CS         接          PE12         //触摸�?片选控制信�?�
 //      T_CLK        接          PE11         //触摸�?SPI总线时钟信�?�
 *
 **************************************************************************************************/

int drum_max_val[4] = {0, 0, 0, 0};
int drum_interrupt_start_tick = 0;
int drum_interrupt_counts = 0;

uint32_t audio_interrupt_counts = 0;
uint32_t audio_interrupt_start_tick = 0;
uint32_t mix_interrupt_counts = 0;
uint32_t mix_interrupt_start_tick = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {
		drum_interrupt_counts++;
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		DrumUpdate();
//
//		if (HAL_GetTick() % 3000 < 10) {
//			for (int i = 0; i < 4; i++) {
//				drum_max_val[i] = 0;
//			}
//		}
//		for (int i = 0; i < 4; i++) {
//			if (drum_max_val[i] < drum_sensor_values[i]) {
//				drum_max_val[i] = drum_sensor_values[i];
//			}
//		}
	}

	else if (htim == &htim4) {
		mix_interrupt_counts++;
		PrecomputeMix();
	}


}



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

GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint16_t keyPressed = 0;
//uint16_t previousKeyPressed = 0;
uint16_t btn_callbacks = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// See if it's falling or rising edge, may fail at times
	int callback_pin_value = HAL_GPIO_ReadPin(BTN_PAD_R1_PORT, GPIO_Pin);
	if (callback_pin_value == GPIO_PIN_RESET) {
		previousMillis = HAL_GetTick();
		return;
	}

//	btn_callbacks += 1;
//	return;
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

		switch (keyPressed) {
			case 0: return;
			case 1: btn_callbacks += 1; break;
			case 2: btn_callbacks -= 1; break;
		}

	}

	previousMillis = currentMillis;
}


typedef struct
{
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
} keyboardHID;

extern USBD_HandleTypeDef hUsbDeviceFS;
keyboardHID keyboardhid = {0,0,0,0,0,0,0,0};

typedef struct USB_JoystickReport_Input_t {
  uint16_t Button; // 16 buttons; see JoystickButtons for bit mapping
  uint8_t  HAT;    // HAT switch; one nibble w/ unused nibble
  uint8_t  LX;     // Left  Stick X
  uint8_t  LY;     // Left  Stick Y
  uint8_t  RX;     // Right Stick X
  uint8_t  RY;     // Right Stick Y
  uint8_t  VendorSpec;
} USB_JoystickReport_Input;
USB_JoystickReport_Input switchhid = {0,0,0,0,0,0,0};

typedef enum {
    SWITCH_Y       = 0x01,
    SWITCH_B       = 0x02,
    SWITCH_A       = 0x04,
    SWITCH_X       = 0x08,
    SWITCH_L       = 0x10,
    SWITCH_R       = 0x20,
    SWITCH_ZL      = 0x40,
    SWITCH_ZR      = 0x80,
    SWITCH_MINUS   = 0x100,
    SWITCH_PLUS    = 0x200,
    SWITCH_LCLICK  = 0x400,
    SWITCH_RCLICK  = 0x800,
    SWITCH_HOME    = 0x1000,
    SWITCH_CAPTURE = 0x2000,
} JoystickButtons;

typedef enum {
	MUSIC_UNINITED,
	MUSIC_PAUSED,
	MUSIC_PLAYING,
} MusicState;


// ----------- AUDIO PLAYBACK -------------
#define AUDIO_FREQ		48000		// TIMER SETTING
#define SYSCLK_FREQ		72000000	// SYSCLOCK FREQUENCY
#define AUDIO_BUFF_LENGTH 500
#define MAX_TRACKS 	10
#define	AUDIO_MASTER_FREQ	AUDIO_FREQ
#define AUDIO_SLAVE_FREQ	96 	// (AUDIO_FREQ / AUDIO_PRECOMP)
//#define AUDIO_PRECOMP_PERIOD
typedef enum {
	DRUM_DON,
	DRUM_KA,
} DrumSound;

union {
	uint16_t u;
	int16_t i;
} audio_buff[AUDIO_BUFF_LENGTH];
typedef struct {
	int16_t* buff;
	uint16_t length;
	uint16_t pos;
} AudioTrack;
AudioTrack audio_tracks[MAX_TRACKS];
int num_tracks = 0;
int audio_dma_on = 0;

void AddDrum(DrumSound sound) {
	AddTrack((AudioTrack) {
		.buff = drum_sounds[sound],
		.length = drum_sound_lengths[sound],
		.pos = 0
	});
}

DAC_HandleTypeDef hdac;
int pos = 0;
void PrecomputeMix() {

//	if (num_tracks <= 0) {
//		if (audio_dma_on) HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
//		memset(audio_buff, 0, AUDIO_BUFF_LENGTH * 2);
//	} else {
//		if (!audio_dma_on) HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)audio_buff, AUDIO_BUFF_LENGTH, DAC_ALIGN_12B_L);

		// trying to just play don

		for (int i = 0; i < AUDIO_BUFF_LENGTH; i++) {
//			audio_buff[i].u = (int16_t) (don[pos]) / 4 + 32768;
			audio_buff[i].u = ka[pos];
//			audio_buff[2 * i + 1].u = don_signed[pos] / 4 + 32768;
			pos = (pos + 1) % ka_length;
		}

//		int j = 0;
//		while (j < num_tracks) {
//			int16_t* buff = audio_tracks[j].buff;
//			uint16_t pos = audio_tracks[j].pos;
//			uint16_t len = audio_tracks[j].length;
//			// min(remaining length of song, audio_buff length)
//			uint16_t min = (len - pos > AUDIO_BUFF_LENGTH) ? AUDIO_BUFF_LENGTH : len - pos;
//			for (int i = 0; i < min; i++) {
//				audio_buff[i].i += buff[pos + i] / 3;
//			}
//			pos += min;
//			audio_tracks[j].pos = pos;
//			if (pos >= len) {
//				RemoveTrack(j);
//			} else {
//				j++; // if you understand how RemoveTrack works
//			}
//		}
//
//		for (int i = 0; i < AUDIO_BUFF_LENGTH; i++) {
//			audio_buff[i].u = -audio_buff[i].i + 32768;
//		}
//	}

}

void AddTrack(AudioTrack track) {
	if (num_tracks >= MAX_TRACKS) return;
	audio_tracks[num_tracks++] = track;
}

void RemoveTrack(uint16_t index) {
	if (num_tracks <= 0) return;
	audio_tracks[index] = audio_tracks[--num_tracks];
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

//  audio_channel_left.curr = audio_channel_left.first = audio_channel_left.out;
//  audio_channel_left.toWrite = 0;
//  audio_channel_right.curr = audio_channel_right.first = audio_channel_right.out;
//  audio_channel_right.toWrite = 0;
//
//  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)audio_channel_left.out, 128, DAC_ALIGN_12B_R);
//  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)audio_channel_right.out, 128, DAC_ALIGN_12B_R);
//  HAL_TIM_Base_Start(&htim4);
//  HAL_TIM_Base_Start(&htim2);

	HAL_GPIO_WritePin(BTN_PAD_C1_PORT, BTN_PAD_C1_PIN, 1);
	HAL_GPIO_WritePin(BTN_PAD_C2_PORT, BTN_PAD_C2_PIN, 1);
	HAL_GPIO_WritePin(BTN_PAD_C3_PORT, BTN_PAD_C3_PIN, 1);
	HAL_GPIO_WritePin(BTN_PAD_C4_PORT, BTN_PAD_C4_PIN, 1);

	ILI9341_Init();
	ILI9341_Set_Rotation(2);
	LCD_FillScreen(PINK);
	HAL_ADC_Start_DMA(&hadc1, drum_sensor_values, 4);
	DrumInit();

  	// Setting the clock divider somehow helps :D
  	FRESULT fresult = f_mount(&fs, "/", 1);
  	if (fresult != FR_OK) {
  		LCD_Print(0, 10, "Error: f_mount (%d)", fresult); while (1);
  	}

  	FIL file;
  	uint16_t temp;
  	fresult = f_open(&file, "drum.cfg", FA_READ | FA_WRITE);
  	if (fresult == FR_OK) {
  		uint32_t buff[5];
  		fresult = f_read(&file, buff, 5 * 4, &temp);
  		if (buff[0] + buff[1] + buff[2] + buff[3] == buff[4]) {
  			for (int i = 0; i < 4; i++) drums[i].sensor_thresh = buff[i];
  		} else {
  			DrumCalibrate();
  			uint32_t buff[5] = {drums[0].sensor_thresh, drums[1].sensor_thresh, drums[2].sensor_thresh, drums[3].sensor_thresh,
					drums[0].sensor_thresh + drums[1].sensor_thresh + drums[2].sensor_thresh + drums[3].sensor_thresh};
  			fresult = f_write(&file, buff, 5 * 4, &temp);
  		}
  		LCD_Print(0, 10, "Have file, reading... %d", fresult);
  	} else if (fresult == FR_NO_FILE) {
  		fresult = f_open(&file, "drum.cfg", FA_WRITE | FA_CREATE_NEW);
  		DrumCalibrate();
  		uint32_t buff[5] = {drums[0].sensor_thresh, drums[1].sensor_thresh, drums[2].sensor_thresh, drums[3].sensor_thresh,
  				drums[0].sensor_thresh + drums[1].sensor_thresh + drums[2].sensor_thresh + drums[3].sensor_thresh};
  		fresult = f_write(&file, buff, 5 * 4, &temp);
  		LCD_Print(0, 10, "No file, calibrating... %d", fresult);
  	} else {
  		LCD_Print(0, 10, "Error: f_open (%d)", fresult); while (1);
  	}
  	f_close(&file);

//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  	for (int i = 0; i < ka_length; i++) {
  		ka[i] = (int32_t) (((int16_t*) ka)[i]) / 3 + 32768;
  	}
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
//	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)don, 23239, DAC_ALIGN_12B_L);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)audio_buff, AUDIO_BUFF_LENGTH, DAC_ALIGN_12B_L);

	HAL_TIM_Base_Start_IT(&htim3);
  	drum_interrupt_start_tick = HAL_GetTick();

	HAL_TIM_Base_Start(&htim2);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
  	audio_interrupt_start_tick = HAL_GetTick();

  	HAL_TIM_Base_Start_IT(&htim4);
	mix_interrupt_start_tick = HAL_GetTick();
//	HAL_TIM_Base_Start(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//
//	uint16_t Wave_LUT[128] = {
//	    2048, 2149, 2250, 2350, 2450, 2549, 2646, 2742, 2837, 2929, 3020, 3108, 3193, 3275, 3355,
//	    3431, 3504, 3574, 3639, 3701, 3759, 3812, 3861, 3906, 3946, 3982, 4013, 4039, 4060, 4076,
//	    4087, 4094, 4095, 4091, 4082, 4069, 4050, 4026, 3998, 3965, 3927, 3884, 3837, 3786, 3730,
//	    3671, 3607, 3539, 3468, 3394, 3316, 3235, 3151, 3064, 2975, 2883, 2790, 2695, 2598, 2500,
//	    2400, 2300, 2199, 2098, 1997, 1896, 1795, 1695, 1595, 1497, 1400, 1305, 1212, 1120, 1031,
//	    944, 860, 779, 701, 627, 556, 488, 424, 365, 309, 258, 211, 168, 130, 97,
//	    69, 45, 26, 13, 4, 0, 1, 8, 19, 35, 56, 82, 113, 149, 189,
//	    234, 283, 336, 394, 456, 521, 591, 664, 740, 820, 902, 987, 1075, 1166, 1258,
//	    1353, 1449, 1546, 1645, 1745, 1845, 1946, 2047
//	};



//  LCD_DrawFilledRectangle(0, 0, 240, 320, RED);

	long last_ticks = 0;
	long tft_last_ticks = 0;
	long ticks = 0;
	int num_hits = 0;
	int hit_state = 0;
	while (1) {


//		keyboardhid.MODIFIER = 0x02;  // left Shift
//		keyboardhid.KEYCODE1 = 0x04;  // press 'a'
//		keyboardhid.KEYCODE2 = 0x05;  // press 'b'
//		USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof (keyboardhid));
//		HAL_Delay (50);
//
//		keyboardhid.MODIFIER = 0x00;  // shift release
//		keyboardhid.KEYCODE1 = 0x00;  // release key
//		keyboardhid.KEYCODE2 = 0x00;  // release key
//		USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof (keyboardhid));

//		switchhid.Button = SWITCH_A | SWITCH_CAPTURE;  // left Shift
//		USBD_HID_SendReport(&hUsbDeviceFS,  (uint8_t*) &switchhid, sizeof (switchhid));
//		HAL_Delay (50);
//
//		switchhid.Button = 0x00;  // shift release
//		USBD_HID_SendReport(&hUsbDeviceFS,  (uint8_t*) &switchhid, sizeof (switchhid));
//		HAL_Delay (200);

		LCD_Print(0, 1, "%4ld, %6d", drum_sensor_values[0], pos);

//	  if (drum_sensor_values[0] > 300) {
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
//		  num_hits += 1;
//	  } else {
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
//	  }
//
		if (HAL_GetTick() - tft_last_ticks > 200) {

			AddDrum(HAL_GetTick() % 2);

//			__disable_irq();
			int r = 0;
			LCD_Print(0, r++, "%02ld:%02ld:%02ld.%03ld, %6.1f Hz, %2d",
					HAL_GetTick() / (1000 * 60 * 60),
					HAL_GetTick() / (1000 * 60) % 60,
					(HAL_GetTick() / 1000) % 60, HAL_GetTick() % 1000,
					(float) drum_interrupt_counts / (HAL_GetTick() - drum_interrupt_start_tick + 1) * 1000,
					num_tracks);

//			LCD_Print(0, r++, "AUD: %10d, %6.1f", audio_interrupt_counts,
//					(float) audio_interrupt_counts / (HAL_GetTick() - audio_interrupt_start_tick + 1) * 1000);
//			LCD_Print(0, r++, "MIX: %10d, %6.1f", mix_interrupt_counts,
//					(float) mix_interrupt_counts / (HAL_GetTick() - mix_interrupt_start_tick + 1) * 1000);

//			LCD_Print(0, r++, "         adc | hits");
//			for (int i = 0; i < 1; i++) {
//				LCD_Print(0, r++, "Drum %d: %4ld | %4d | %4d", i,
//						drum_sensor_values[i], drums[i].hit_count, drum_max_val[i]);
//			}

//			for (int i = 0; i < 4; i++) {
//				LCD_Print(0, r++, "%lf, %lf, %d",
//						drums[i].sensor_avg, drums[i].sensor_sd, drums[i].sensor_thresh);
//			}

//			__enable_irq();

			tft_last_ticks = HAL_GetTick();
		}

		LCD_Print(0, 6, "BTN: %3d", btn_callbacks);

//
//	  if (HAL_GetTick() - last_ticks > 400) {
//
//		  if ((HAL_GetTick() / 400) % 2 == 0) {
//			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//		  } else {
//			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//		  }
//
//		  last_ticks = HAL_GetTick();
//	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 100;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|T_CLK_Pin
                          |T_CS_Pin|T_DIN_Pin|T_DO_Pin|T_IRQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_DC_Pin|LCD_CS_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 PE6 T_CLK_Pin
                           T_CS_Pin T_DIN_Pin T_DO_Pin T_IRQ_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|T_CLK_Pin
                          |T_CS_Pin|T_DIN_Pin|T_DO_Pin|T_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DC_Pin LCD_CS_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin|LCD_CS_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
