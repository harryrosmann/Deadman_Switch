#ifndef __CRC16__
#define __CRC16__

#include <stdint.h>

uint16_t crc16_compute(uint8_t const * p_data, uint32_t size, uint16_t const * p_crc);

#endif
