#include "drum.h"
#include "math.h"
#include "main.h"
#include "stdlib.h"
#include "fatfs.h"
#include "lcd.h"

FATFS fs;

uint32_t drum_sensor_values[NUM_DRUMS];
uint8_t drum_i2c_buff[20];
DrumStruct drums[NUM_DRUMS];
DrumOutputDevice drum_output_device;
uint32_t drum_max_val[NUM_DRUMS] = {0};

void DrumInit() {

	// init ADC
//	HAL_ADC_Start_DMA(&hadc1, drum_sensor_values, 4);

	// init drums struct
	drum_output_device = DRUM_OUTPUT_NONE;
//	drum_output_callback = &DrumOutputDefault;
	for (int i = 0; i < NUM_DRUMS; i++) {
		drums[i].type = i;
		if (drums[i].type == DRUM_LEFT_KA || drums[i].type == DRUM_RIGHT_KA)
			drums[i].sound = DRUM_KA;
		else
			drums[i].sound = DRUM_DON;
		drums[i].state = DRUM_IDLE;
		drums[i].sensor_value_pt = &(drum_sensor_values[i]);
		drums[i].hit_count = 0;
		drums[i].last_tick = HAL_GetTick();
	}

	// deal with sd card stuff
	// Format: {drum[0], drum[1], drum[2], drum[3], checksum}
	FIL file;
	uint16_t temp;
	FRESULT fresult = f_open(&file, "drum.cfg", FA_READ | FA_WRITE);
	if (fresult == FR_OK) {
		uint32_t buff[5];
//		fresult = f_read(&file, buff, 5 * 4, &temp);
		if (buff[0] + buff[1] + buff[2] + buff[3] == buff[4]) {
			for (int i = 0; i < 4; i++) drums[i].sensor_thresh = buff[i];
		} else {
			DrumCalibrate();
			uint32_t buff[5] = {drums[0].sensor_thresh, drums[1].sensor_thresh, drums[2].sensor_thresh, drums[3].sensor_thresh,
					drums[0].sensor_thresh + drums[1].sensor_thresh + drums[2].sensor_thresh + drums[3].sensor_thresh};
			fresult = f_write(&file, buff, 5 * 4, &temp);
		}
		LCD_Print(0, 19, "Success: Have file, reading... %d", fresult);
	} else if (fresult == FR_NO_FILE) {
		fresult = f_open(&file, "drum.cfg", FA_WRITE | FA_CREATE_NEW);
		DrumCalibrate();
		uint32_t buff[5] = {drums[0].sensor_thresh, drums[1].sensor_thresh, drums[2].sensor_thresh, drums[3].sensor_thresh,
				drums[0].sensor_thresh + drums[1].sensor_thresh + drums[2].sensor_thresh + drums[3].sensor_thresh};
		fresult = f_write(&file, buff, 5 * 4, &temp);
		LCD_Print(0, 19, "Error: No file, calibrating... %d", fresult);
	} else {
		LCD_Print(0, 19, "Error: f_open (%d)", fresult); while (1);
	}
	f_close(&file);

}

void DrumThreshWrite() {
	FIL file;
	int temp;
	FRESULT fresult = f_open(&file, "drum.cfg", FA_READ | FA_WRITE);
	if (fresult == FR_OK) {
		uint32_t buff[5] = {drums[0].sensor_thresh, drums[1].sensor_thresh, drums[2].sensor_thresh, drums[3].sensor_thresh,
				drums[0].sensor_thresh + drums[1].sensor_thresh + drums[2].sensor_thresh + drums[3].sensor_thresh};
		fresult = f_write(&file, buff, 5 * 4, &temp);
	} else if (fresult == FR_NO_FILE) {
		fresult = f_open(&file, "drum.cfg", FA_WRITE | FA_CREATE_NEW);
		uint32_t buff[5] = {drums[0].sensor_thresh, drums[1].sensor_thresh, drums[2].sensor_thresh, drums[3].sensor_thresh,
				drums[0].sensor_thresh + drums[1].sensor_thresh + drums[2].sensor_thresh + drums[3].sensor_thresh};
		fresult = f_write(&file, buff, 5 * 4, &temp);
	} else {
		LCD_Print(0, 19, "Error: f_open (%d)", fresult); while (1);
	}
	f_close(&file);
}

// Let's not touch this :D
void DrumCalibrate() {

	uint32_t last_tick = HAL_GetTick();
	//	uint16_t values[NUM_DRUMS][DRUM_CALIBRATION_SAMPLE_NUM];
	// Dynamically allocate memory for the 2D array
	uint16_t **values = (uint16_t**) malloc(NUM_DRUMS * sizeof(uint16_t*));
	for (int i = 0; i < NUM_DRUMS; ++i) {
		values[i] = (uint16_t*) malloc(DRUM_CALIBRATION_SAMPLE_NUM * sizeof(uint16_t));
	}

	// sampling
	for (int i = 0; i < DRUM_CALIBRATION_SAMPLE_NUM; i++) {
		while (last_tick == HAL_GetTick()) {
		} // waits for 1 tick
		last_tick = HAL_GetTick();
		for (int j = 0; j < NUM_DRUMS; j++) {
			values[j][i] = drum_sensor_values[j];
		}
	}

	// calculates avg and sd for each drum, tunes threshold
	for (int i = 0; i < NUM_DRUMS; i++) {
		uint32_t sum = 0;
		uint32_t max_val = 0;
		for (int j = 0; j < DRUM_CALIBRATION_SAMPLE_NUM; j++) {
			sum += values[i][j];
			if (values[i][j] > max_val)
				max_val = values[i][j];
		}
		double avg = (double) sum / DRUM_CALIBRATION_SAMPLE_NUM;
		double sqerr = 0;
		for (int j = 0; j < DRUM_CALIBRATION_SAMPLE_NUM; j++) {
			sqerr += (values[i][j] - avg) * (values[i][j] - avg);
		}
		double sd = sqrt(sqerr / (DRUM_CALIBRATION_SAMPLE_NUM - 1));

		drums[i].sensor_avg = avg;
		drums[i].sensor_max = max_val;
		drums[i].sensor_sd = sd;
		drums[i].sensor_thresh = fmin(avg + 7 * sd, max_val + 5 * sd);

	}

	for (int i = 0; i < NUM_DRUMS; ++i) {
		free(values[i]);
	}
	free(values);

}

void DrumUpdate(uint16_t activations) { // actiavtions: bitwise representation

	__disable_irq();

	int i = 0;
	DrumStruct* drum = drums;
	for (; i < 4; i++, drum++) {
		if (*(drum->sensor_value_pt) > drum->sensor_thresh || ((activations >> i) & 1)) {
			if (drum->state == DRUM_IDLE) {
				AddDrum(drum->sound); // play the sound
			}
			drum->state = DRUM_HIT;
			drum->last_tick = HAL_GetTick();
		} else {
			if (drum->state != DRUM_IDLE) {
				drum->state = DRUM_COOLDOWN;
				if (/* drum->state == DRUM_COOLDOWN
					&& */HAL_GetTick() - drum->last_tick > DRUM_COOLDOWN_LENGTH) {
					drum->state = DRUM_IDLE;
				}
			}
		}

		if (*(drum->sensor_value_pt) > drum_max_val[i]) {
			drum_max_val[i] = *(drum->sensor_value_pt);
		}
	}

	__enable_irq();
}

uint16_t drum_calibrate_number = 0;
uint32_t drum_max_val_reset_ticks = 0;
void LCD_DrumCalibration(int* r) {
	if (HAL_GetTick() - drum_max_val_reset_ticks > 1500) {
		for (int i = 0; i < NUM_DRUMS; i++) drum_max_val[i] = 0;
		drum_max_val_reset_ticks = HAL_GetTick();
	}

	LCD_Print(0, (*r)++, "cal: %2d | adc | hits | max", drum_calibrate_number);
	for (int i = 0; i < 4; i++) {
		LCD_Print(0, (*r)++, "Drum %d: %4ld | %4d | %4d", i,
				drum_sensor_values[i], drums[i].hit_count, drum_max_val[i]);
	}

	LCD_Print(0, (*r)++, "trsh | st. | avg | sd ");
	for (int i = 0; i < 4; i++) {
		LCD_Print(0, (*r)++, "%4d | %3d | %4.0f | %2.2f",
				drums[i].sensor_thresh, drums[i].state, drums[i].sensor_avg, drums[i].sensor_sd);
	}

//	LCD_Print(0, (*r)++, "%3d", drums[0].state);

}

void ButtonPad_DrumCalibration(int keyPressed) {
	switch (keyPressed) {
		case 1: AddDrum(DRUM_DON); break;
		case 2: AddDrum(DRUM_KA); break;
		case 3: DrumUpdate(1); break;
		case 4: DrumUpdate(2); break;
		case 5: drums[drum_calibrate_number].sensor_thresh += 10; break;
		case 6: drums[drum_calibrate_number].sensor_thresh -= 10; break;
		case 7: drum_calibrate_number = (drum_calibrate_number + 1) % NUM_DRUMS; break;
		case 8: DrumThreshWrite(); break;
	}
}
