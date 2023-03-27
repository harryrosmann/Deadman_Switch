#ifndef __HAMMING__
#define __HAMMING__

#include <stdint.h>
#include <stdbool.h>

void hamming_encode(uint8_t const *bytes, uint32_t numBytes, uint8_t * buf, uint32_t bufSize);
void hamming_decode(uint8_t const *bytes, uint32_t numBytes, uint8_t *buf, uint32_t bufSize);

#endif
