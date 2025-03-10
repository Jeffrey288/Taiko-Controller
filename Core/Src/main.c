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
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "drum.h"
#include "kadon.h"
#include "audio.h"
#include "button.h"
#include "usb_device.h"
#include "adc.h"
#include "task.h"
#include "descriptor.h"
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

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
osThreadId adcTaskHandle;
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
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);
void StartADCTask(void const * argument);

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

int drum_interrupt_start_tick = 0;
int drum_interrupt_counts = 0;


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



uint32_t audio_interrupt_counts = 0;
uint32_t audio_interrupt_start_tick = 0;
uint32_t mix_interrupt_counts = 0;
uint32_t mix_interrupt_start_tick = 0;

int16_t reading;
int16_t voltage[4];
int16_t max_reading[4];


uint32_t adc_buffer_length = 0;
uint32_t itr_tick = 0;
int16_t errors[4];
void processADC();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {
		processADC();

//		itr_tick++;
//		for (int i = 0; i < 4; i++) {
//			adc_buffer[adc_buffer_length][i] = values_adc[i];
//		}
//		adc_buffer[adc_buffer_length][4] = itr_tick;
//		adc_buffer_length++;;



//		drum_interrupt_counts++;
//		DrumUpdate(0);
//
//
//		uint8_t ADSConfig[3] = {0x01,
//							     ADS1115_OS | ADS1115_MODE_CONTINUOUS | ADS1115_PGA_ONE,
//								 ADS1115_DATA_RATE_250 | ADS1115_COMP_MODE | ADS1115_COMP_POL | ADS1115_COMP_LAT | ADS1115_COMP_QUE };
//		uint8_t ADSWrite[1] = {0x00};
//		uint8_t ADSReceive[2];
////		__disable_irq();
//		for (int i = 0; i < 4; i++){
//			ADSConfig[1] = ADS1115_OS | ADS1115_PGA_ONE | ADS1115_MODE_CONTINUOUS | ((0b100 | i) << 4); // choose AIN
//
//			int temp;
//			errors[1] = HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSConfig, 3, 100);
////			if (!temp) LCD_Print(0, r++, "ERROR 1! %d", temp);
//			errors[2] = HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSWrite, 1, 100);
////			if (!temp) LCD_Print(0, r++, "ERROR 2! %d", temp);
////			HAL_Delay(20);
//
//			errors[3] = HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS << 1, ADSReceive, 2, 100);
////			if (!temp) LCD_Print(0, r++, "ERROR 3! %d", temp);
//			voltage[i] = (ADSReceive[0] << 8 | ADSReceive[1]);

//			ADS1115_config[0] = ADS1115_OS | ain_pin_addr[i] | ADS1115_pga | ADS1115_MODE;
//			ADS1115_config[1] = ADS1115_dataRate | ADS1115_COMP_MODE | ADS1115_COMP_POL | ADS1115_COMP_LAT| ADS1115_COMP_QUE;
//
//			if(HAL_I2C_Mem_Write(&ADS1115_I2C_Handler, (uint16_t) (ADS1115_devAddress << 1), ADS1115_CONFIG_REG, 1, ADS1115_config, 2, ADS1115_TIMEOUT) == HAL_OK){
//
//				if(HAL_I2C_Mem_Read(&ADS1115_I2C_Handler, (uint16_t) ((ADS1115_devAddress << 1) | 0x1), ADS1115_CONVER_REG, 1, ADS1115_rawValue, 2, ADS1115_TIMEOUT) == HAL_OK){
//
//					voltage[i] = (float) (((int16_t) (ADS1115_rawValue[0] << 8) | ADS1115_rawValue[1]) * ADS1115_voltCoef);
//
//				}
//
//			}


//		}
//		__enable_irq();

//		if (drum_interrupt_counts % 2 == 0) {
//
//		keyboardhid.KEYCODE1 = drums[0].state >= DRUM_HIT ? 0x07 : 0x00;  // press 'd'
//		keyboardhid.KEYCODE2 = drums[1].state >= DRUM_HIT ? 0x09 : 0x00;  // press 'f'
//		keyboardhid.KEYCODE3 = drums[2].state >= DRUM_HIT ? 0x0d : 0x00;  // press 'j'
//		keyboardhid.KEYCODE4 = drums[3].state >= DRUM_HIT ? 0x0e : 0x00;  // press 'k'
//		USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
//		}
	}

	else if (htim == &htim4) {
		mix_interrupt_counts++;
		PrecomputeMix();
	}
}

uint16_t btn_callbacks = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int keyPressed = ButtonPadCallback(GPIO_Pin);
	ButtonPad_DrumCalibration(keyPressed);
	btn_callbacks++;
}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	if (hi2c == &hi2c1) {
		for (int i = 0; i < 4; i++) {
			drum_sensor_values[i] = drum_i2c_buff[i];
		}
	}
}

uint8_t Rx_data[1] = {0};
uint16_t Rx_buff[6] = {0};
int Rx_length = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	Rx_length += 1;
	if (Rx_length < 2) {
		if (*Rx_data != 0xFF) {
			Rx_length = 0;
		} else {
			((uint8_t*) Rx_buff)[Rx_length++] = *Rx_data;
		}
	} else {
		((uint8_t*) Rx_buff)[Rx_length++] = *Rx_data;
		if (Rx_length == 12) {
			if (Rx_buff[1] + Rx_buff[2] + Rx_buff[3] + Rx_buff[4] == Rx_buff[5]) {
				for (int i = 0; i < 4; i++) {
					drum_sensor_values[i] = Rx_buff[i+1];
				}
			}
			Rx_length = 0;
		}
	}
	HAL_UART_Receive_IT(&huart1, Rx_data, 1);
}

int isSent = 1;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	isSent = 1;
//	countinterrupt++;
}

uint32_t values_adc[1000];
uint32_t adc_processed_until = 0; // in number of readings
uint32_t total_processed = 0; // groups
int cur_active = -1;
int max_value = 0;
uint32_t first_high_time = 0;
uint32_t last_hit_time = 0;
int32_t call_tick = -1;
int call_array[] = {7, 9, 13, 14};
int uart_debug = 0;

#define AVG_WINDOW 3
uint16_t past_values[4][AVG_WINDOW]; // average over past 4 values
int16_t sum_past[4];
int16_t avg_past[4];
uint8_t cur_past_idx = 0;


// hit -> peak
// find the highest peak
// when no update of max after some duration (say 3), activate drum (cuz should be peak, won't need to care)
// then the values will slowly decrease, this is the timeout
// smaller peaks beyond the max should be ignored, we don't want to care about them -> double reaction + cross reaction
//
void processADC() {

	if (uart_debug) return;

//		__disable_irq();
	int adc_transferred_until = 1000 - hadc1.DMA_Handle->Instance->CNDTR;
//		__enable_irq();

	while (adc_processed_until / 4 != adc_transferred_until / 4) {

//		for (int i = 0; i < 4; i++) {
////			sum_past[i] += - past_values[cur_past_idx][i] + new_value;
//			uint16_t new_value = values_adc[adc_processed_until + i];
//			past_values[cur_past_idx][i] = new_value;
//			sum_past[i] = 0;
//			for (int j = 0; j < AVG_WINDOW; j++) {
//				sum_past[i] += past_values[i][j];
//			}
//			avg_past[i] = sum_past[i] / AVG_WINDOW;
//		}
//		cur_past_idx = (cur_past_idx + 1) % AVG_WINDOW;

		int lk = values_adc[adc_processed_until];
		int ld = values_adc[adc_processed_until + 1];
		int rd = values_adc[adc_processed_until + 2];
		int rk = values_adc[adc_processed_until + 3];
//		int lk = avg_past[0];
//		int ld = avg_past[1];
//		int rd = avg_past[2];
//		int rk = avg_past[3];

		if (call_tick != -1 && HAL_GetTick() - call_tick > 10) {
			keyboardhid.KEYCODE1 = 0;
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof (keyboardhid));
			call_tick = -1;
		}

		if (total_processed - last_hit_time > 60) {
			if (cur_active == -1 && (lk > 100 || ld > 100 || rd > 100 || rk > 100)) {
				first_high_time = total_processed;
				max_value = lk;
				cur_active = 0;
			}

			if ((lk > 60 || ld > 60 || rd > 60 || rk > 60)) {
				first_high_time = total_processed;
			}

			if (cur_active != -1) {
				if (lk > max_value) {
					max_value = lk;
					cur_active = 0;
				}
				if (ld > max_value) {
					max_value = ld;
					cur_active = 1;
				}
				if (rd > max_value) {
					max_value = rd;
					cur_active = 2;
				}
				if (rk > max_value) {
					max_value = rk;
					cur_active = 3;
				}

				if (total_processed - first_high_time > 1) {
					// activate cur_active;
					last_hit_time = total_processed;
					call_tick = HAL_GetTick();
					keyboardhid.KEYCODE1 = call_array[cur_active];
					USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof (keyboardhid));

//					char send = 'd' + call_array[cur_active] - 7;
//					HAL_UART_Transmit(&huart1, &send, 1, 1000);

					cur_active = -1;
					max_value = 0;
				}
			}
		}

		adc_processed_until = (adc_processed_until + 4) % 1000;
		total_processed++;
	}

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
  UpdateHIDClassConfig(&Keyboard_ClassConfig);
  UpdateHIDItfConfig(&Keyboard_ItfConfig);
//  UpdateHIDClassConfig(&Switch_ClassConfig);
//  UpdateHIDItfConfig(&Switch_ItfConfig);
  MX_USB_DEVICE_Init();
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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)values_adc,1000);
  // 1.125MHz * 239.5 cycles * 4 = 0.85 ms
//  HAL_UART_Receive_IT (&huart1, Rx_data, 1);

	ButtonPadInit();

	ILI9341_Init();
	ILI9341_Set_Rotation(2);
	LCD_FillScreen(PINK);

  	// Setting the clock divider somehow helps :D
//  	FRESULT fresult = f_mount(&fs, "/", 1);
//  	if (fresult != FR_OK) {
//  		LCD_Print(0, 19, "Error: f_mount (%d)", fresult); while (1);
//  	}

//	DrumInit();

//	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
//
	TIM3->PSC = 720 - 1;
//	TIM3->ARR = 50 * 50 - 1; // 400Hz
	TIM3->ARR = 50 - 1; // 2000Hz
	HAL_TIM_Base_Start_IT(&htim3);
//  	drum_interrupt_start_tick = HAL_GetTick();
//
//  	TIM4->PSC = 0;
//	TIM4->ARR = 1499; // 48000Hz
//	HAL_TIM_Base_Start(&htim2);
//	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
//  	audio_interrupt_start_tick = HAL_GetTick();
//
//  	HAL_TIM_Base_Start_IT(&htim4);
//  	TIM4->PSC = 0;
//  	TIM4->ARR = 499; // 96Hz
//  	HAL_TIM_Base_Start_IT(&htim4); // runs off of TIM2
//	mix_interrupt_start_tick = HAL_GetTick();


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

//  /* Create the thread(s) */
//  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
//
//  /* definition and creation of adcTask */
//  osThreadDef(adcTask, StartADCTask, osPriorityRealtime, 0, 128);
//  adcTaskHandle = osThreadCreate(osThread(adcTask), NULL);
//
//  /* USER CODE BEGIN RTOS_THREADS */
////  /* add threads, ... */
//  /* USER CODE END RTOS_THREADS */
//
//  /* Start scheduler */
//  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



	long last_ticks = 0;
	long tft_last_ticks = 0;
	long ticks = 0;
	int num_hits = 0;
	int hit_state = 0;
//
//	if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) (ADS1115_ADDRESS << 1), 5, 1) == HAL_OK) {
////		return HAL_OK;
//	} else {
////		return HAL_ERROR;
//		while (1) {}
//	}


	uint32_t reset_ticks;
	uint16_t temp = 0;
	int uart_debug_offset = 0;


//	int cur_active = -1;
//	int max_value = 0;
//	uint32_t first_high_time = 0;
//	uint32_t last_hit_time = 0;
//	uint32_t total_processed = 0;
//	uint32_t adc_processed_until = 0;
//	int32_t call_tick = -1;
//	int call_array[] = {7, 9, 13, 14};
	while (1) {
//
////		 let's just assume this loop runs very very very fast
////		__disable_irq();
//		int adc_transferred_until = 1000 - hadc1.DMA_Handle->Instance->CNDTR;
////		__enable_irq();
//
//		while (adc_processed_until / 4 != adc_transferred_until / 4) {
//			int lk = values_adc[adc_processed_until];
//			int ld = values_adc[adc_processed_until + 1];
//			int rd = values_adc[adc_processed_until + 2];
//			int rk = values_adc[adc_processed_until + 3];
//			rk = (int) (rk * 1.2);
//			// fsm: no active (cur_active = -1) -> yes active for a period (cur_active != -1) -> cooldown (cur_active = -1, cooldown) -> no active
//
//			if (call_tick != -1 && HAL_GetTick() - call_tick > 10) {
//				keyboardhid.KEYCODE1 = 0;
//				USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof (keyboardhid));
//				call_tick = -1;
//			}
//
//			if (total_processed - last_hit_time > 60) {
//				if (cur_active == -1 && (lk > 100 || ld > 100 || rd > 100 || rk > 100)) {
//					first_high_time = total_processed;
//					max_value = lk;
//					cur_active = 0;
//				}
//
//				if ((lk > 60 || ld > 60 || rd > 60 || rk > 60)) {
//					first_high_time = total_processed;
//				}
//
//				if (cur_active != -1) {
//					if (lk > max_value) {
//						max_value = lk;
//						cur_active = 0;
//					}
//					if (ld > max_value) {
//						max_value = ld;
//						cur_active = 1;
//					}
//					if (rd > max_value) {
//						max_value = rd;
//						cur_active = 2;
//					}
//					if (rk > max_value) {
//						max_value = rk;
//						cur_active = 3;
//					}
//
//					if (total_processed - first_high_time > 1) {
//						// activate cur_active;
//						last_hit_time = total_processed;
//						call_tick = HAL_GetTick();
//						keyboardhid.KEYCODE1 = call_array[cur_active];
//						USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof (keyboardhid));
//
//						char send = 'd' + call_array[cur_active] - 7;
//						HAL_UART_Transmit(&huart1, &send, 1, 1000);
//
//						cur_active = -1;
//						max_value = 0;
//					}
//				}
//			}
//
//			adc_processed_until = (adc_processed_until + 4) % 1000;
//			total_processed++;
//		}

//		char send2 = ' ';
//		HAL_UART_Transmit(&huart1, &send2, 1, 1000);

//		keyboardhid.KEYCODE1 = 9;
//		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof (keyboardhid));

//		char tx_buff[20] = "apple";
//		hadc1->DMA_Handle->Instance->CNDTR;
//		hadc1.DMA_Handle;

//		sprintf(tx_buff, "adc0: %5d\n  ", (int) hadc1.DMA_Handle->Instance->CNDTR);
//		sprintf(tx_buff, "adc0: togaw\n  ",);
//		sprintf(tx_buff, "adc0: %5d\n  ", (int) values_adc[0]);

//		HAL_UART_Transmit(&huart1, tx_buff, ((char*) tx_buff_u16) - tx_buff + 4, 1000);

		char rx_buff;
		if (HAL_UART_Receive(&huart1, &rx_buff, 1, 0)==HAL_OK) //if transfer is successful
		{
			if (0 <= rx_buff & rx_buff <= 3) {
				uart_debug_offset = rx_buff;
			}
			else if (rx_buff == 90) {
				uart_debug = uart_debug ? 0 : 1;
			}
		}


		if (uart_debug && isSent == 1)
		{
			// 9 MHz = 9 * 10^6 data points per second
			// 9M / (239.5 + 12.5) / 40 = 8.8
			// 9 *10^6 / (239.5 + 12.5) / 5 = 7142
			// 9 *10^6 / (239.5 + 12.5) / 2 = 11904
			char tx_buff[2000];
			tx_buff[0] = 's';
			tx_buff[1] = 't';
			tx_buff[2] = 'a';
			tx_buff[3] = 't';
			int skip_count = 1;
			int adc_transferred_until = 1000 - hadc1.DMA_Handle->Instance->CNDTR;
			char* tx_buff_u16 = tx_buff + 4;
			while (adc_processed_until / 4 != adc_transferred_until / 4) {
				*(tx_buff_u16++) = (uint8_t) (values_adc[adc_processed_until] >> 2);
				adc_processed_until = (adc_processed_until + 1) % 1000;
			}
			*(((char *) tx_buff_u16) + 0) = 'e';
			*(((char *) tx_buff_u16) + 1) = 'n';
			*(((char *) tx_buff_u16) + 2) = 'd';
			*(((char *) tx_buff_u16) + 3) = 'e';

			  HAL_UART_Transmit_DMA(&huart1, tx_buff, ((char*) tx_buff_u16) - tx_buff + 4);
			  isSent = 0;
		}






//		 HAL_Delay(20);
//
//		for (int i = 0; i < 4; i++) {
//			if (voltage[i] > max_reading[i]) {
//				max_reading[i] = voltage[i];
//			}
//		}
//
//		if (HAL_GetTick() - reset_ticks > 1000) {
//			reset_ticks = HAL_GetTick();
//			for (int i = 0; i < 4; i++) {
//				max_reading[i] = 0;
//			}
//		}
//
//		int r = 0;
//		if (HAL_GetTick() - tft_last_ticks > 100) {

//			uint8_t data;
//			HAL_UART_Receive(&huart1, &data, 1, 10);

//			AddDrum((HAL_GetTick() / 1000) % 2);
//			LCD_Print(0, r++, "%02ld:%02ld:%02ld.%03ld, %6.1fHz,%2d,%2d",
//					HAL_GetTick() / (1000 * 60 * 60),
//					HAL_GetTick() / (1000 * 60) % 60,
//					(HAL_GetTick() / 1000) % 60, HAL_GetTick() % 1000,
//					(float) drum_interrupt_counts / (HAL_GetTick() - drum_interrupt_start_tick + 1) * 1000,
//					Rx_length, btn_callbacks);
//			LCD_Print(0, r++, "err: %4d", temp);
//			LCD_Print(0, r++, "acd%6d %6d %6d %6d     ", max_reading[0], max_reading[1], max_reading[2], max_reading[3]);
//////			LCD_DrumCalibration(&r);
//			LCD_Print(0, r++, "acd%6d %6d %6d %6d     ", voltage[0], voltage[1], voltage[2], voltage[3]);
//			LCD_Print(0, r++, "acd%6d %6d %6d %6d     ", errors[0], errors[1], errors[2], errors[3]);
//			LCD_Print(0, 0, "%05d %05d", max_reading[0], voltage[0]);
			//			LCD_DrumCalibration(&r);


//			for (int i = 0; i < 20; i++) {
//				LCD_Print(0, r++, "%3d %3d %3d %3d %3d",
//									requests[i].bmRequest,
//									requests[i].bRequest,
//									requests[i].wValue,
//									requests[i].wIndex,
//									requests[i].wLength);
//			}

			/**
			 * USBD_LL_SetupStage
			 * USBD_StdEPReq
			 * USBD_CUSTOM_HID_Setup
			 *
			 * https://gist.github.com/abarisani/4595a7c535435038e0571237893c81c4
			 * https://github.com/keyboardio/FingerprintUSBHost/blob/master/src/FingerprintUSBHost.cpp
			 * https://www.circuitbread.com/tutorials/how-usb-works-enumeration-and-configuration-part-3
			 *
			// 127/63: keyboard
			// 150/86: switch

			33 = 0x21	& USB_REQ_TYPE_MASK (0x60) = 0x20
			129 = 0x81	& USB_REQ_TYPE_MASK (0x60) = 0x00

			// windows 10:
			// 33 10 0 0 0									USB_REQ_TYPE_CLASS		CUSTOM_HID_REQ_SET_IDLE
			// 129 6 8704 0 127/150							USB_REQ_TYPE_STANDARD	USB_REQ_GET_DESCRIPTOR
			// 33 9 512 0 0(1 in windows 7)	keyboard		USB_REQ_TYPE_CLASS
			 *
			 * 	get device descriptor (64 bytes)
				set address
				get device descriptor (bLength)
				get configuration descriptor (255 bytes)
				get string descriptors, including language/zero (255 bytes)
				get device descriptor (bLength)
				get configuration descriptor (1st: bLength, 2nd: wTotalLength)
				get string descriptors, including language/zero (1st: 2 bytes, 2nd: bLength)

			// switch:
			// 129 6 8704 0 63/86
			// 33 11 0 0 0 keyboard
			// 33 10 0 0 0
			// 33 9 512 0 1

			// mac
			// 129 6 8704 0 63/86
			// 33 10 1563 0 0
			// 33 11 1 0 0
			 *
			 * 	set address
				get device descriptor (bLength)
				get string descriptors, without language/zero (1st: 2 bytes, 2nd: bLength)
				get configuration descriptor (1st: bLength, 2nd: wTotalLength)

			// android
			// 33 10 0 0 0
			// 129 6 8704 0 63/86
			// 33 9 512 0 1
			 *
			 */

//			tft_last_ticks = HAL_GetTick();
//		}

//
////		keyboardhid.MODIFIER = 0x02;  // left Shift
//		for (int i = 0; i < 26; i++) {
//			keyboardhid.KEYCODE1 = 0x04 + i;  // press 'a'
//			temp = USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof (keyboardhid));
//			HAL_Delay (10);
//		}

//		switchhid.Button = 0x04;
//		temp = USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &switchhid, sizeof (switchhid));
//		HAL_Delay (20);
//
//		switchhid.Button = 0x00;
//		temp = USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &switchhid, sizeof (switchhid));
//		HAL_Delay (20);

//		keyboardhid.MODIFIER = 0x00;  // shift release
//		keyboardhid.KEYCODE1 = 0x05;  // release key
//		keyboardhid.KEYCODE2 = 0x00;  // release key
//		temp = USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof (keyboardhid));
//		HAL_Delay (4);

//
//		uint8_t ADSConfig[3] = {0x01,
//							     ADS1115_OS | ADS1115_MODE_CONTINUOUS | ADS1115_PGA_ONE,
//								 ADS1115_DATA_RATE_250 | ADS1115_COMP_MODE | ADS1115_COMP_POL | ADS1115_COMP_LAT | ADS1115_COMP_QUE };
//		uint8_t ADSWrite[1] = {0x00};
//		uint8_t ADSReceive[2];
//		for (int i = 0; i < 4; i++){
//			ADSConfig[1] = ADS1115_OS | ADS1115_PGA_ONE | ADS1115_MODE_CONTINUOUS | ((0b100 | i) << 4); // choose AIN
//
//			int temp;
//			temp = HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSConfig, 3, 100);
////			if (!temp) LCD_Print(0, r++, "ERROR 1! %d", temp);
//			temp = HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSWrite, 1, 100);
////			if (!temp) LCD_Print(0, r++, "ERROR 2! %d", temp);
//			HAL_Delay(20);
//
//			temp = HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS << 1, ADSReceive, 2, 100);
////			if (!temp) LCD_Print(0, r++, "ERROR 3! %d", temp);
//			voltage[i] = (ADSReceive[0] << 8 | ADSReceive[1]);
//
////			ADS1115_config[0] = ADS1115_OS | ain_pin_addr[i] | ADS1115_pga | ADS1115_MODE;
////			ADS1115_config[1] = ADS1115_dataRate | ADS1115_COMP_MODE | ADS1115_COMP_POL | ADS1115_COMP_LAT| ADS1115_COMP_QUE;
////
////			if(HAL_I2C_Mem_Write(&ADS1115_I2C_Handler, (uint16_t) (ADS1115_devAddress << 1), ADS1115_CONFIG_REG, 1, ADS1115_config, 2, ADS1115_TIMEOUT) == HAL_OK){
////
////				if(HAL_I2C_Mem_Read(&ADS1115_I2C_Handler, (uint16_t) ((ADS1115_devAddress << 1) | 0x1), ADS1115_CONVER_REG, 1, ADS1115_rawValue, 2, ADS1115_TIMEOUT) == HAL_OK){
////
////					voltage[i] = (float) (((int16_t) (ADS1115_rawValue[0] << 8) | ADS1115_rawValue[1]) * ADS1115_voltCoef);
////
////				}
////
////			}
//
//
//		}


//		switchhid.Button = SWITCH_A | SWITCH_CAPTURE;  // left Shift
//		USBD_HID_SendReport(&hUsbDeviceFS,  (uint8_t*) &switchhid, sizeof (switchhid));
//		HAL_Delay (50);
//
//		switchhid.Button = 0x00;  // shift release
//		USBD_HID_SendReport(&hUsbDeviceFS,  (uint8_t*) &switchhid, sizeof (switchhid));
//		HAL_Delay (200);

//		uint8_t ay = 20;
//		HAL_UART_Transmit(&huart1, &ay, 1, 10);



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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5; // ADC_SAMPLETIME_71CYCLES_5; // ADC_SAMPLETIME_239CYCLES_5;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 5, 0);
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
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */

	long last_ticks = 0;
	long tft_last_ticks = 0;
	long ticks = 0;
	int num_hits = 0;
	int hit_state = 0;
	int reset_ticks = 0;
  /* Infinite loop */
  for(;;)
  {
//	  osDelay(1);


		for (int i = 0; i < 4; i++) {
			if (voltage[i] > max_reading[i]) {
				max_reading[i] = voltage[i];
			}
		}

		if (HAL_GetTick() - reset_ticks > 200) {
			reset_ticks = HAL_GetTick();
//			for (int i = 0; i < 4; i++) {
//				max_reading[i] = 0;
//			}
			 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
		}

		int r = 0;
////		if (HAL_GetTick() - tft_last_ticks > 10) {
//
////			uint8_t data;
////			HAL_UART_Receive(&huart1, &data, 1, 10);
//
////			AddDrum((HAL_GetTick() / 1000) % 2);
//		__disable_irq();
//		UBaseType_t uxSavedInterruptStatus;
//		 uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
			LCD_Print(0, r++, "%02ld:%02ld:%02ld.%03ld, %6.1fHz,%2d,%2d",
					HAL_GetTick() / (1000 * 60 * 60),
					HAL_GetTick() / (1000 * 60) % 60,
					(HAL_GetTick() / 1000) % 60, HAL_GetTick() % 1000,
					(float) drum_interrupt_counts / (HAL_GetTick() - drum_interrupt_start_tick + 1) * 1000,
					Rx_length, btn_callbacks);
//		LCD_Print(0, r++, "%02ld:%02ld:%02ld.%03ld",
//							HAL_GetTick() / (1000 * 60 * 60),
//							HAL_GetTick() / (1000 * 60) % 60,
//							(HAL_GetTick() / 1000) % 60, HAL_GetTick() % 1000);
//		LCD_Print(0, r++, "penis");
			LCD_Print(0, r++, "acd%6d %6d %6d %6d     ", max_reading[0], max_reading[1], max_reading[2], max_reading[3]);
//			  taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );
//			__enable_irq();
//////			LCD_DrumCalibration(&r);
			LCD_Print(0, r++, "acd%6d %6d %6d %6d     ", voltage[0], voltage[1], voltage[2], voltage[3]);
//			LCD_Print(0, r++, "acd%6d %6d %6d %6d     ", errors[0], errors[1], errors[2], errors[3]);
////			LCD_Print(0, 0, "%05d %05d", max_reading[0], voltage[0]);
//			//			LCD_DrumCalibration(&r);
//			tft_last_ticks = HAL_GetTick();
//		}
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartADCTask */
/**
* @brief Function implementing the adcTask thread.
* @param argument: Not used
* @retval None
*/


typedef union
{
    struct
    {
        volatile unsigned char    RESV  	  :1;   //it does not matter 0 or 1
        volatile unsigned char    NOP         :2;
        volatile unsigned char    PULLUP      :1;
        volatile unsigned char    TS_MODE     :1;
        volatile unsigned char    DR          :3;
        volatile unsigned char    MODE        :1;
        volatile unsigned char    PGA         :3;
        volatile unsigned char    MUX         :3;
        volatile unsigned char    OS          :1;   //high
    } stru;
    volatile unsigned int  word;
    volatile unsigned char byte[2];
} ADS_InitTypeDef;


//ADS1118 Configuration Registers
ADS_InitTypeDef adsConfigReg;

/* USER CODE END Header_StartADCTask */
void StartADCTask(void const * argument)
{
  /* USER CODE BEGIN StartADCTask */
	//ADS Structure variable
//		ADS_InitTypeDef ConfigReg;

		//We will use it as single-shot mode
		adsConfigReg.stru.OS				=	0x1;   //high
		// 0x4 enables AIN0 , 0x5 enables AIN1,0x6 enables AIN2 and 0x7 enables AIN3 .
		adsConfigReg.stru.MUX			=	0x4;
		//Programmable Gain amplifier
		adsConfigReg.stru.PGA      = 0x1;		//  FSR 4.096V
		//Continuous mode or single-shot mode
		adsConfigReg.stru.MODE			=	0x0;
		//Data Rate register
		adsConfigReg.stru.DR       = 0x4;
		//If you want to use this chip as a temperature sensor set this as 1.
		adsConfigReg.stru.TS_MODE	=	0x0;
		//Enable built-in pull-up resistors.
		adsConfigReg.stru.PULLUP		= 0x1;
		//Command mode. Set this always as 0x01.
		adsConfigReg.stru.NOP			=	0x1;
		//Reserved register. It does not matter this register is 1 or 0.
		adsConfigReg.stru.RESV			= 0x1;

//		adsConfigReg.word=ConfigReg->word;

//		HAL_Delay(100);


  /* Infinite loop */
  for(;;)
  {
	  drum_interrupt_counts++;
	  osDelay(3);

	  for (int i = 0; i < 4; i++){
		  voltage[i];
	  }





//	  		uint8_t ADSConfig[3] = {0x01,
//	  							     ADS1115_OS | ADS1115_MODE_CONTINUOUS | ADS1115_PGA_ONE,
//	  								 ADS1115_DATA_RATE_250 | ADS1115_COMP_MODE | ADS1115_COMP_POL | ADS1115_COMP_LAT | ADS1115_COMP_QUE };
//	  		uint8_t ADSWrite[1] = {0x00};
//	  		uint8_t ADSReceive[2];
//	  //		__disable_irq();
//	  		for (int i = 0; i < 4; i++){
//	  			ADSConfig[1] = ADS1115_OS | ADS1115_PGA_ONE | ADS1115_MODE_CONTINUOUS | ((0b100 | i) << 4); // choose AIN
//
//	  			int temp;
//	  			errors[1] = HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSConfig, 3, 100);
//	  //			if (!temp) LCD_Print(0, r++, "ERROR 1! %d", temp);
//	  			errors[2] = HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSWrite, 1, 100);
//	  //			if (!temp) LCD_Print(0, r++, "ERROR 2! %d", temp);
//	  //			HAL_Delay(20);
//
//	  			errors[3] = HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS << 1, ADSReceive, 2, 100);
//	  //			if (!temp) LCD_Print(0, r++, "ERROR 3! %d", temp);
//	  			voltage[i] = (ADSReceive[0] << 8 | ADSReceive[1]);
//	  			osDelay(10);

	  //			ADS1115_config[0] = ADS1115_OS | ain_pin_addr[i] | ADS1115_pga | ADS1115_MODE;
	  //			ADS1115_config[1] = ADS1115_dataRate | ADS1115_COMP_MODE | ADS1115_COMP_POL | ADS1115_COMP_LAT| ADS1115_COMP_QUE;
	  //
	  //			if(HAL_I2C_Mem_Write(&ADS1115_I2C_Handler, (uint16_t) (ADS1115_devAddress << 1), ADS1115_CONFIG_REG, 1, ADS1115_config, 2, ADS1115_TIMEOUT) == HAL_OK){
	  //
	  //				if(HAL_I2C_Mem_Read(&ADS1115_I2C_Handler, (uint16_t) ((ADS1115_devAddress << 1) | 0x1), ADS1115_CONVER_REG, 1, ADS1115_rawValue, 2, ADS1115_TIMEOUT) == HAL_OK){
	  //
	  //					voltage[i] = (float) (((int16_t) (ADS1115_rawValue[0] << 8) | ADS1115_rawValue[1]) * ADS1115_voltCoef);
	  //
	  //				}
	  //
	  //			}


//	  		}
  }
  /* USER CODE END StartADCTask */
}

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
