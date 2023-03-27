#ifndef __PACKET_ENGINE__
#define __PACKET_ENGINE__

#include <stdint.h>

#define PACKET_EMPTY 0
#define ERR_PACKET_BUF_SIZE -1
#define ERR_PACKET_PAYLOAD_LARGE -2
#define ERR_PACKETING_FAILED -3
#define ERR_PACKET_CRC_MISMATCH -4
#define ERR_PAYLOAD_BUF_SMALL -5

int32_t constructPacket(uint8_t const *payload, uint32_t payloadLength, uint8_t *packet, uint32_t packetBufSize);
int32_t decodePacket(uint8_t const *packet, uint32_t packetLength, uint8_t *payload, uint32_t payloadBufSize);

#endif
