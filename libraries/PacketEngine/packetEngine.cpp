#include "packetEngine.h"
#include "simplePacket.h"
#include "hamming.h"
#include <stdint.h>

/*
* @description: Simple function to construct packets
*
* @param[in] payload - The payload that is getting packeted
* @param[in] payloadLength - The length of the payload including the NULL character
* @param[out] packet - The resulting packet. Must be allocated outside of this function
* @param[in] packetBufSize - The maximum size of the packet buffer. It is recommended that it is at least twice plus 10 of the size of the payload length
*
* @ret length of resulting packet upon success, <0 on error
*
* Possible error conditions:
* - ERR_PACKET_BUF_SIZE: packet buffer size too small (solution: allocate more space or split payload)
* - ERR_PACKET_PAYLOAD_LARGE: payload cannot fit into single packet (solution: split payload)
* - ERR_PACKETING_FAILED: packeting failed for some reason (solution: try again?)
*/
int32_t constructPacket(uint8_t const *payload, uint32_t payloadLength, uint8_t *packet, uint32_t packetBufSize)
{
	uint8_t packetBuffer[SP_MAX_PAYLOAD_LENGTH + SP_OVERHEAD + 1];

	// Check packet buffer size
	if (((payloadLength * 2) + (SP_OVERHEAD * 2)) > packetBufSize)
	{
		return ERR_PACKET_BUF_SIZE;
	}

	// Check if payload is too large
	if (payloadLength > SP_MAX_PAYLOAD_LENGTH)
	{
		return ERR_PACKET_PAYLOAD_LARGE;
	}

	create_simple_packet(payload, payloadLength, packetBuffer, (uint16_t)sizeof(packetBuffer));

	// double check packet
	if (!check_simple_packet(packetBuffer))
	{
		return ERR_PACKETING_FAILED;
	}

	hamming_encode(packetBuffer, simple_packet_get_packet_size(packetBuffer), packet, packetBufSize);

	return 2 * (payloadLength + SP_OVERHEAD);
}

/*
* @description: Simple function to deconstruct packets
*
* @param[in] packet - The packet that needs deconstruction
* @param[out] payload - The resulting payload. Must be allocated outside of this function
* @param[in] payloadBufSize - The maximum size of the payload buffer. It is recommended that it is at least 256 bytes
*
* @ret length of resulting payload upon success, <0 on error
*
* Possible error conditions:
* - ERR_PACKET_CRC_MISMATCH: packet corrupted (solution: No solution, send NACK)
* - ERR_PAYLOAD_BUF_SMALL: payload cannot fit into payload buffer (solution: allocate more space for payload receive buffer)
*/
int32_t decodePacket(uint8_t const *packet, uint32_t packetLength, uint8_t *payload, uint32_t payloadBufSize)
{
	uint8_t packetBuf[SP_MAX_PAYLOAD_LENGTH + SP_OVERHEAD + 1];
	int32_t payloadLength;

	hamming_decode(packet, packetLength, packetBuf, SP_MAX_PAYLOAD_LENGTH + SP_OVERHEAD + 1);

	if (!check_simple_packet(packetBuf))
	{
		return ERR_PACKET_CRC_MISMATCH;
	}

	payloadLength = simple_packet_get_payload_size(packetBuf);

	if (payloadLength > payloadBufSize)
	{
		return ERR_PAYLOAD_BUF_SMALL;
	}

	simple_packet_extract_payload(packetBuf, payload, payloadBufSize);

	return payloadLength;
}
