#include "hamming.h"
#include "BitMatrixMath.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

const matrix_type G[11][15] = {
	 {1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
	 {1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
	 {1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
	 {0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
	 {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
	 {0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
	 {1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
	 {1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
	 {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}
};

const matrix_type Ht[15][4] = {
	 {1, 0, 0, 0},
	 {0, 1, 0, 0},
	 {0, 0, 1, 0},
	 {0, 0, 0, 1},
	 {1, 1, 0, 0},
	 {0, 1, 1, 0},
	 {0, 0, 1, 1},
	 {1, 1, 0, 1},
	 {1, 0, 1, 0},
	 {0, 1, 0, 1},
	 {1, 1, 1, 0},
	 {0, 1, 1, 1},
	 {1, 1, 1, 1},
	 {1, 0, 1, 1},
	 {1, 0, 0, 1}
};

const matrix_type st[16][4] = {
	 {1, 0, 0, 0},
	 {0, 1, 0, 0},
	 {0, 0, 1, 0},
	 {0, 0, 0, 1},
	 {1, 1, 0, 0},
	 {0, 1, 1, 0},
	 {0, 0, 1, 1},
	 {1, 1, 0, 1},
	 {1, 0, 1, 0},
	 {0, 1, 0, 1},
	 {1, 1, 1, 0},
	 {0, 1, 1, 1},
	 {1, 1, 1, 1},
	 {1, 0, 1, 1},
	 {1, 0, 0, 1},
	 {0, 0, 0, 0}
};

const matrix_type eps[16][15] = {
	 {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
	 {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
	 {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
	 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
	 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
	 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
	 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
	 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

static uint16_t encodeChar(uint8_t c)
{
	int8_t cBits[11] = { 0 };  // MSB 3 bits should be 0
	int8_t encodedBits[15] = { 0 };
	uint16_t encodedChar = 0; // MSB should be 0

	for (uint32_t i = 0; i < 8; i++)
	{
		cBits[10 - i] = (c >> i) & 1;
	}

	multiply((matrix_type *)cBits, (matrix_type *)G, 1, 11, 15, (matrix_type *)encodedBits);

	for (uint32_t i = 0; i < 15; i++)
	{
		encodedChar |= encodedBits[14 - i] << i;
	}

	return encodedChar;
}

static uint8_t decodeChar(uint16_t c)
{
	int8_t encodedBits[15] = { 0 };
	int8_t s[4];
	uint8_t Char = 0;
	uint32_t row;


	for (uint32_t i = 0; i < 15; i++)
	{
		encodedBits[14 - i] = (c >> i) & 1;
	}

	multiply((matrix_type *)encodedBits, (matrix_type *)Ht, 1, 15, 4, (matrix_type *)s);

	row = ismember((matrix_type *)st, (matrix_type *)s, 16, 4);

	XOR((matrix_type *)encodedBits, (matrix_type *)eps[row], 1, 15, (matrix_type *)encodedBits);

	// first 4 MSB are the parity bits
	// following 3 MSB are useless fillers
	for (uint32_t i = 7; i < 15; i++)
	{
		Char |= encodedBits[i] << (14 - i);
	}

	return Char;
}

void hamming_encode(uint8_t const * bytes, uint32_t numBytes, uint8_t * buf, uint32_t bufSize)
{
	uint16_t encodedChar;
	uint32_t index;
	assert(bufSize > ((2 * numBytes) + 5));

	for (index = 0; index < numBytes; index++)
	{
		encodedChar = encodeChar(bytes[index]);
		buf[index * 2] = encodedChar >> 8;
		buf[(index * 2) + 1] = encodedChar & 0xFF;
	}
}

void hamming_decode(uint8_t const * bytes, uint32_t numBytes, uint8_t *buf, uint32_t bufSize)
{
	uint16_t encodedByte;

	for (uint32_t i = 0; (i < numBytes); i += 2)
	{
		encodedByte = (uint16_t)(bytes[i] & 0xFF) << 8;
		encodedByte |= (uint16_t)(bytes[i + 1] & 0xFF);

		buf[i >> 1] = decodeChar(encodedByte);
	}
}
