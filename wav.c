/*
 * wav.c
 *
 *  Created on: 14/04/2014
 *      Author: Christian
 */

#include "wav.h"
#include "stdbool.h"
#include "stdint.h"

#include "intrinsics.h"

#define SWAP_UINT16(x) _swap_bytes(x)
#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))
//#define SWAP_UINT32(x) ( (uint32_t) _swap_bytes(x & 0xFFFF) << 16) || (_swap_bytes(x >> 16))

bool wav_parse(uint8_t *data, wav_fmt *fmt) {
	if (*((uint32_t *) &data[0]) != SWAP_UINT32(0x52494646) ) {
		return false;
	}
	if (*((uint32_t *) &data[8]) != SWAP_UINT32(0x57415645) ) {
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

