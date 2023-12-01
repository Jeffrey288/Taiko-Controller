#include "audio.h"
#include "kadon.h"
#include "stm32f1xx_hal.h"
#include "main.h"

uint16_t audio_buff[AUDIO_BUFF_LENGTH];

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

void AddTrack(AudioTrack track) {
	if (num_tracks >= MAX_TRACKS) return;
	audio_tracks[num_tracks++] = track;
}

void RemoveTrack(uint16_t index) {
	if (num_tracks <= 0) return;
	audio_tracks[index] = audio_tracks[--num_tracks];
}

void PrecomputeMix() {

	if (num_tracks <= 0) {
		if (audio_dma_on) {
			HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
			audio_dma_on = 0;
		}
		memset(audio_buff, 0, AUDIO_BUFF_LENGTH * 2);
	} else {

		if (!audio_dma_on) {
			// Resynchronize the timer
			HAL_TIM_Base_Stop(&htim2);
			HAL_TIM_Base_Stop(&htim4);

			HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)audio_buff, AUDIO_BUFF_LENGTH, DAC_ALIGN_12B_L);
			TIM2->CNT = 0;
			TIM4->CNT = 0;

			HAL_TIM_Base_Start_IT(&htim4);
			HAL_TIM_Base_Start(&htim2);
			__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

			audio_dma_on = 1;
		}

		// Completely unoptimized code, cuz no need :D
		for (int i = 0; i < AUDIO_BUFF_LENGTH; i++) {

			int32_t sum = 0;
			int j = 0;
			while (j < num_tracks) {
				sum += audio_tracks[j].buff[audio_tracks[j].pos++] / 4;
				if (audio_tracks[j].pos > audio_tracks[j].length) {
					RemoveTrack(j);
				} else {
					j++;
				}
			}

			if (sum < -32768) {
				audio_buff[i] = 0;
			} else if (sum > 32767) {
				audio_buff[i] = 65535;
			} else {
				audio_buff[i] = sum + 32768;
			}
		}

	}

}

