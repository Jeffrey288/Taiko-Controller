#include "drum.h"
#include "math.h"

uint32_t drum_sensor_values[4];
DrumStruct drums[4];
DrumOutputDevice drum_output_device;
DrumOutputCallback drum_output_callback;

void DrumOutputDefault(DrumType type) {
	drums[type].hit_count++;
}

void DrumInit() {

	drum_output_device = DRUM_OUTPUT_NONE;
	drum_output_callback = &DrumOutputDefault;
	for (int i = 0; i < NUM_DRUMS; i++) {
		drums[i].type = i;
		drums[i].state = DRUM_IDLE;
		drums[i].sensor_value_pt = &(drum_sensor_values[i]);
		drums[i].hit_count = 0;
		drums[i].last_tick = HAL_GetTick();
	}

}

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

