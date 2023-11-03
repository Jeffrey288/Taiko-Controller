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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {
		drum_interrupt_counts++;
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		DrumUpdate();

		if (HAL_GetTick() % 3000 < 10) {
			for (int i = 0; i < 4; i++) {
				drum_max_val[i] = 0;
			}
		}
		for (int i = 0; i < 4; i++) {
			if (drum_max_val[i] < drum_sensor_values[i]) {
				drum_max_val[i] = drum_sensor_values[i];
			}
		}
	}

	if (htim == &htim4) {

	}

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
#define AUDIO_PRECOMP 	500		// precompute how many samples per interrupt
#define AUDIO_BLOCKS	5		// how many audio blocks are there for DAC DMA
#define AUDIO_INSTANCES_MAX  10	 // at most how many audio instance can be played at once
#define	AUDIO_MASTER_FREQ	AUDIO_FREQ
#define AUDIO_SLAVE_FREQ	96 	// (AUDIO_FREQ / AUDIO_PRECOMP)
//#define AUDIO_PRECOMP_PERIOD
typedef enum {
	DRUM_DON,
	DRUM_KA,
} DrumSound;

typedef struct {
	DrumSound drum_sound;
	uint32_t audio_index;
	uint32_t audio_length;
} AudioInstance;

AudioInstance audio_instances[AUDIO_INSTANCES_MAX];
uint16_t audio_instances_num = 0;

typedef struct {
	// the precomputed mix buffer that the DMA is going to send
	uint16_t out[AUDIO_PRECOMP * AUDIO_BLOCKS];
	uint16_t *curr;		// current position
	uint16_t *first;	// first position in the out array

	uint16_t toWrite;	// which block to write
	uint32_t channel;	// which DMA channel to use
	uint8_t onFlag;		// whether this audio channel is being used
} AudioChannel;

AudioChannel audio_channel_left;
AudioChannel audio_channel_right;

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
  	drum_interrupt_start_tick = HAL_GetTick();
	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


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

		switchhid.Button = SWITCH_A | SWITCH_CAPTURE;  // left Shift
		USBD_HID_SendReport(&hUsbDeviceFS,  (uint8_t*) &switchhid, sizeof (switchhid));
		HAL_Delay (50);

		switchhid.Button = 0x00;  // shift release
		USBD_HID_SendReport(&hUsbDeviceFS,  (uint8_t*) &switchhid, sizeof (switchhid));
		HAL_Delay (200);


//	  if (drum_sensor_values[0] > 300) {
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
//		  num_hits += 1;
//	  } else {
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
//	  }
//
		if (HAL_GetTick() - tft_last_ticks > 30) {

//			__disable_irq();
			int r = 0;
			LCD_Print(0, r++, "%02ld:%02ld:%02ld.%03ld, %6.1f Hz",
					HAL_GetTick() / (1000 * 60 * 60),
					HAL_GetTick() / (1000 * 60) % 60,
					(HAL_GetTick() / 1000) % 60, HAL_GetTick() % 1000,
					(float) drum_interrupt_counts / (HAL_GetTick() - drum_interrupt_start_tick + 1) * 1000);

			LCD_Print(0, r++, "         adc | hits");
			for (int i = 0; i < 4; i++) {
				LCD_Print(0, r++, "Drum %d: %4ld | %4d | %4d", i,
						drum_sensor_values[i], drums[i].hit_count, drum_max_val[i]);
			}

			for (int i = 0; i < 4; i++) {
				LCD_Print(0, r++, "%lf, %lf, %d",
						drums[i].sensor_avg, drums[i].sensor_sd, drums[i].sensor_thresh);
			}

//			__enable_irq();

			tft_last_ticks = HAL_GetTick();
		}
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
  htim2.Init.Prescaler = 1632;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
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
  htim4.Init.Prescaler = 88;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, T_CLK_Pin|T_CS_Pin|T_DIN_Pin|T_DO_Pin
                          |T_IRQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_DC_Pin|LCD_CS_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : T_CLK_Pin T_CS_Pin T_DIN_Pin T_DO_Pin
                           T_IRQ_Pin */
  GPIO_InitStruct.Pin = T_CLK_Pin|T_CS_Pin|T_DIN_Pin|T_DO_Pin
                          |T_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
