#ifndef __SIMPLE_PACKET__
#define __SIMPLE_PACKET__

#include <stdint.h>
#include <stdbool.h>

/*
header: 1 byte
length: 1 byte
crc16: 2 bytes
*/

#define SP_HEADER_OVERHEAD 1
#define SP_LENGTH_OVERHEAD 1
#define SP_CRC_OVERHEAD 2

#define SP_OVERHEAD (SP_HEADER_OVERHEAD + SP_LENGTH_OVERHEAD + SP_CRC_OVERHEAD)

#define SP_HEADER '^'

#define SP_HEADER_OFFSET 0
#define SP_LENGTH_OFFSET 1
#define SP_PAYLOAD_OFFSET 2

#define SP_MAX_PAYLOAD_LENGTH 255

void create_simple_packet(uint8_t const *payload, uint16_t payloadSize, uint8_t *buffer, uint16_t bufferSize);
bool check_simple_packet(uint8_t const *packet);
void simple_packet_extract_payload(uint8_t const *packet, uint8_t *payload, uint16_t payloadSize);
uint8_t simple_packet_get_payload_size(uint8_t const *packet);
uint16_t simple_packet_get_packet_size(uint8_t const *packet);

#endif
