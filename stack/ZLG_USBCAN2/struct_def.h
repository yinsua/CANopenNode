#pragma once

typedef struct
{
    unsigned int device_type;
    unsigned int device_id;
    unsigned int channel;
    unsigned int baud;
    unsigned int tx_type;
    unsigned int tx_sleep;
    unsigned int tx_frames;
    unsigned int tx_count;
    unsigned int rx_timeout;
} CANDeviceType;

typedef enum{
    BAUD_1M = 0x1400
} CANBaud;