
#include<stdint.h>
#define payloadSize   24

const uint8_t _sbusHeader = 0x0F;
const uint8_t _sbusFooter = 0x00;
const uint8_t _sbus2Footer = 0x04;
const uint8_t _sbus2Mask = 0x0F;
#define SBUS_LOST_FRAME 0x04
#define SBUS_FAILSAFE 0x08

uint8_t payload[payloadSize ];


typedef struct SBUS_CHANNEL
{
    uint16_t  channels_out[15];
    bool failsafe;
    bool lost_frame;
} sbus_channel_data_t;