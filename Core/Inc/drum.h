#ifndef __DRUM_H
#define __DRUM_H

#include <stdint.h>
#include "fatfs.h"

extern FATFS fs;

#define DRUM_CALIBRATION_SAMPLE_NUM 300
#define DRUM_COOLDOWN_LENGTH 2

typedef enum {
	DRUM_IDLE,
	DRUM_HIT,
	DRUM_COOLDOWN,
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

typedef struct {
	DrumType type;

	uint32_t hit_count;

	DrumState state;
	uint32_t last_tick;

	uint32_t* sensor_value_pt;
	uint32_t sensor_max;
	double sensor_avg;
	double sensor_sd;
	uint32_t sensor_thresh;
} DrumStruct;

//typedef void (*DrumOutputCallback) (DrumType);
extern uint32_t drum_sensor_values[4];
extern DrumStruct drums[4];
extern DrumOutputDevice drum_output_device;
//extern DrumOutputCallback drum_output_callback;

void DrumInit();
void DrumCalibrate();

// This runs every millisecond
static inline void DrumUpdate() {

	int i = 0;
	DrumStruct* drum = drums;
	for (; i < 4; i++, drum++) {
		if (*(drum->sensor_value_pt) > drum->sensor_thresh) {
			if (drum->state != DRUM_HIT) {
				drum->last_tick = HAL_GetTick();
//				if (drum->state == DRUM_IDLE) {
//					drum_output_callback(i);
//				}
				drum->state = DRUM_HIT;
			}
		} else {
			if (drum->state == DRUM_IDLE) {
			} else if (drum->state == DRUM_HIT) {
				drum->last_tick = HAL_GetTick();
				drum->state = DRUM_COOLDOWN;
			} else if (HAL_GetTick() - drum->last_tick > DRUM_COOLDOWN_LENGTH) {
//				drum->last_tick = HAL_GetTick();
				drum->state = DRUM_IDLE;
			}
		}
	}

}


#endif // __DRUM_H
