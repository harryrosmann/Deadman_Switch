#include "simplePacket.h"
#include "crc16.h"
#include <string.h>
#include <assert.h>

void create_simple_packet(uint8_t const * payload, uint16_t payloadSize, uint8_t * buffer, uint16_t bufferSize)
{
  uint8_t crc1, crc2;
  uint16_t crc;
  assert((payloadSize + SP_OVERHEAD) < bufferSize);

  memset(buffer, 0, bufferSize);

  buffer[SP_HEADER_OFFSET] = SP_HEADER;

  buffer[SP_LENGTH_OFFSET] = payloadSize & 0xFF;

  memcpy(buffer + SP_PAYLOAD_OFFSET, payload, payloadSize & 0xFF);

  crc = crc16_compute(buffer, (uint32_t)(SP_HEADER_OVERHEAD + SP_LENGTH_OVERHEAD + payloadSize), NULL);

  crc1 = crc >> 8;
  crc2 = crc & 0xFF;

  memset(buffer + SP_HEADER_OVERHEAD + SP_LENGTH_OVERHEAD + payloadSize, crc1, 1);
  memset(buffer + SP_HEADER_OVERHEAD + SP_LENGTH_OVERHEAD + payloadSize + 1, crc2, 1);
}

static uint16_t get_crc(uint8_t const *packet)
{
  uint16_t crc;
  uint8_t payloadLength = simple_packet_get_payload_size(packet);
  crc = *(packet + SP_HEADER_OVERHEAD + SP_LENGTH_OVERHEAD + payloadLength) << 8;
  crc |= *(packet + SP_HEADER_OVERHEAD + SP_LENGTH_OVERHEAD + payloadLength + 1);

  return crc;
}

bool check_simple_packet(uint8_t const * packet)
{
  uint16_t crc = crc16_compute(packet, SP_HEADER_OVERHEAD + SP_LENGTH_OVERHEAD + simple_packet_get_payload_size(packet), NULL);
  uint16_t payloadCRC = get_crc(packet);

  return payloadCRC == crc;
}

void simple_packet_extract_payload(uint8_t const * packet, uint8_t * payload, uint16_t payloadSize)
{
  assert(payloadSize >= simple_packet_get_payload_size(packet));

  memcpy(payload, packet + SP_PAYLOAD_OFFSET, simple_packet_get_payload_size(packet));
}

uint8_t simple_packet_get_payload_size(uint8_t const * packet)
{
  return packet[SP_LENGTH_OFFSET];
}

uint16_t simple_packet_get_packet_size(uint8_t const *packet)
{
	return (uint16_t)packet[SP_LENGTH_OFFSET] + SP_OVERHEAD;
}
