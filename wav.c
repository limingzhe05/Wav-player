/*
 * wav.c
 *
 *  Created on: 14/04/2014
 *      Author: Christian
 */

#include "wav.h"
#include "stdbool.h"
#include "stdint.h"

bool wav_parse(uint8_t *data, wav_fmt *fmt) {
	if (*((uint32_t *) &data[0]) != 0x52494646 ) {
		return false;
	}
	if (*((uint32_t *) &data[8]) != 0x57415645 ) {
		return false;
	}
	fmt->AudioFormat = (uint8_t) *((uint16_t *) &data[20]);
	if (fmt->AudioFormat != 1) {
		return false;
	}
	fmt->NumChannels = (uint8_t) *((uint16_t *) &data[22]);
	fmt->SampleRate = (uint16_t) *((uint32_t *) &data[24]);
	fmt->BitsPerSample = (uint8_t) *((uint16_t *) &data[34]);

	return true;
}

