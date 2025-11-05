#define  DEFAULT_CRC_VALUE     0XFFFF

typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;


const uint16_t CRCTALBES[] =
{
	0x0000, 0xCC01, 0xD801, 0x1400, 
	0xF001, 0x3C00, 0x2800, 0xE401, 
	0xA001, 0x6C00, 0x7800, 0xB401, 
	0x5000, 0x9C01, 0x8801, 0x4400
};

uint16_t crc_iv(uint8_t *msg, uint32_t length, uint16_t crc)
{
	unsigned int i = 0;
	uint8_t  chChar = 0;

	for (i = 0; i < length; i++) 
    {
		chChar = *msg++;
		crc = CRCTALBES[(chChar ^ crc) & 15] ^ (crc >> 4);
		crc = CRCTALBES[((chChar >> 4) ^ crc) & 15] ^ (crc >> 4);
	}
    
	return crc;
}

uint16_t com_crc(uint8_t *msg, uint32_t length)
{
    return crc_iv(msg, length, DEFAULT_CRC_VALUE);
}