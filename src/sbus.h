#ifndef _APP_SBUS_H_
#define _APP_SBUS_H_

#include <stdint.h>
#include <stdbool.h>

#define SBUS_BAUD (100000)

#define SBUS_NUM_CHANNELS (16)
#define SBUS_PACKET_SIZE (25)
#define SBUS_HEADER (0x0f)
#define SBUS_END (0x00)

#define SBUS_OPT_C17 (0b0001)
#define SBUS_OPT_C18 (0b0010)
#define SBUS_OPT_FS  (0b1000)
#define SBUS_OPT_FL  (0b0100)

enum sbus_err
{
    SBUS_OK = 0,
    SBUS_FAIL = -1,
    SBUS_ERR_TCGETS2 = -2,
    SBUS_ERR_TCSETS2 = -3,
    SBUS_ERR_OPEN = -4,
    SBUS_ERR_INVALID_ARG = -5,
    SBUS_ERR_DESYNC = -6,
};

struct sbus_packet
{
    uint16_t channels[SBUS_NUM_CHANNELS];
    bool ch17, ch18;
    bool failsafe;
    bool frame_lost;
};

void sbus_init(void);

void sbus_read(struct sbus_packet *packet);

#endif
