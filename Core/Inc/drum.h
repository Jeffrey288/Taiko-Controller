#ifndef __DRUM_H
#define __DRUM_H

#include <stdint.h>
#include "fatfs.h"

extern FATFS fs;

#define DRUM_CALIBRATION_SAMPLE_NUM 300
#define DRUM_COOLDOWN_LENGTH 20

typedef enum {
	DRUM_IDLE = 0,
	DRUM_OFF = 0,
	DRUM_HIT = 1,
	DRUM_ON = 1,
	DRUM_COOLDOWN = 2,
	NUM_DRUM_STATES
} DrumState;

typedef enum {
	DRUM_LEFT_KA = 0,
	DRUM_LEFT_DON = 1,
	DRUM_RIGHT_DON = 2,
	DRUM_RIGHT_KA = 3,
	NUM_DRUMS = 4
} DrumType;

typedef enum {
	DRUM_OUTPUT_NONE,
	DRUM_OUTPUT_PC,
	DRUM_OUTPUT_SWITCH,
	DRUM_OUTPUT_PS,
	DRUM_OUTPUT_WII
} DrumOutputDevice;

typedef enum {
	DRUM_DON,
	DRUM_KA,
} DrumSound;

typedef struct {
	DrumType type;
	DrumSound sound;

	uint32_t hit_count;

	DrumState state;
	uint32_t last_tick;

	uint32_t* sensor_value_pt;
	uint32_t sensor_max;
	double sensor_avg;
	double sensor_sd;
	uint32_t sensor_thresh;
} DrumStruct;

extern uint32_t drum_sensor_values[NUM_DRUMS];
extern DrumStruct drums[NUM_DRUMS];
extern DrumOutputDevice drum_output_device;
extern uint32_t drum_max_val[NUM_DRUMS]; // used for debug

void DrumInit();
void DrumCalibrate();
void DrumUpdate(uint16_t);
void DrumThreshWrite();
void LCD_DrumCalibration(int* r);
void ButtonPad_DrumCalibration(int keyPressed);

#endif // __DRUM_H
