
#include<stdint.h>
#define payloadSize   24

const uint8_t _sbusHeader = 0x0F;
const uint8_t _sbusFooter = 0x00;
const uint8_t _sbus2Footer = 0x04;
const uint8_t _sbus2Mask = 0x0F;

uint8_t payload[payloadSize ];