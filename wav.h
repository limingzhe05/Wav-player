/*
 * wav.h
 *
 *  Created on: 14/04/2014
 *      Author: Christian
 */

#ifndef WAV_H_
#define WAV_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
uint8_t AudioFormat;
uint8_t NumChannels;
uint16_t SampleRate;
uint8_t BitsPerSample;
} wav_fmt;

bool wav_parse(uint8_t *data, wav_fmt *fmt);

#endif /* WAV_H_ */
