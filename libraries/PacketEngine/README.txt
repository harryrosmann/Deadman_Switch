The packet engine should be the only thing you interact with. 
It wraps the packeting and hamming stuff for you so you don't have 
to worry about implementing it correctly. 

Here is an example usage:

#include "packetEngine.h"
#include <stdio.h>

int main(void)
{
        uint8_t payload[] = "test";
        uint8_t packet[256] = "";
        uint8_t decodedPacket[256] = "";
        int32_t packetLength = constructPacket(payload, 5, packet, 256);
	//check error conditions (outlined in packetEngine.h)

        int32_t payloadLength = decodePacket(packet, packetLength, decodedPacket, packetLength);
        //check error conditions (outlined in packetEngine.h)

        return 0;
}