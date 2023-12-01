#ifndef __AUDIO_H
#define __AUDIO_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include "drum.h"

// ----------- AUDIO PLAYBACK -------------
#define AUDIO_FREQ		48000		// TIMER SETTING
#define SYSCLK_FREQ		72000000	// SYSCLOCK FREQUENCY
#define AUDIO_BUFF_LENGTH 500
#define MAX_TRACKS 	10
#define	AUDIO_MASTER_FREQ	AUDIO_FREQ
#define AUDIO_SLAVE_FREQ	96 	// (AUDIO_FREQ / AUDIO_PRECOMP)

extern uint16_t audio_buff[AUDIO_BUFF_LENGTH];

typedef struct {
	int16_t* buff;
	uint16_t length;
	uint16_t pos;
} AudioTrack;

extern AudioTrack audio_tracks[MAX_TRACKS];
extern int num_tracks;
extern int audio_dma_on;
extern DAC_HandleTypeDef hdac;

void AddDrum(DrumSound sound);
void AddTrack(AudioTrack track);
void RemoveTrack(uint16_t index);
void PrecomputeMix();

#endif // __AUDIO_H
