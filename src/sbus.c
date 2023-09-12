#include "sbus.h"

#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define SBUS_RX_PIN (13)
#define SBUS_UART (uart0)

enum parser_state
{
    WAIT_FOR_HEADER,
    PACKET,
    HEADER_SKIP,
} g_parser_state = WAIT_FOR_HEADER;

static uint8_t g_parser_buf[SBUS_PACKET_SIZE] = {0};
static int g_packet_pos = 0;
static struct sbus_packet g_last_packet = {0};

static uint g_pio_sm;

static enum sbus_err sbus_decode(const uint8_t buf[], struct sbus_packet *packet)
{
    if (!packet || !buf)
    {
        return SBUS_ERR_INVALID_ARG;
    }
    if (buf[0] != SBUS_HEADER || buf[24] != SBUS_END)
    {
        return SBUS_FAIL;
    }

    uint16_t *channels = packet->channels;
    const uint8_t *payload = buf + 1;
    channels[0]  = (uint16_t)((payload[0]       | payload[1] << 8)                       & 0x07FF);
    channels[1]  = (uint16_t)((payload[1] >> 3  | payload[2] << 5)                       & 0x07FF);
    channels[2]  = (uint16_t)((payload[2] >> 6  | payload[3] << 2 | payload[4] << 10)    & 0x07FF);
    channels[3]  = (uint16_t)((payload[4] >> 1  | payload[5] << 7)                       & 0x07FF);
    channels[4]  = (uint16_t)((payload[5] >> 4  | payload[6] << 4)                       & 0x07FF);
    channels[5]  = (uint16_t)((payload[6] >> 7  | payload[7] << 1 | payload[8] << 9)     & 0x07FF);
    channels[6]  = (uint16_t)((payload[8] >> 2  | payload[9] << 6)                       & 0x07FF);
    channels[7]  = (uint16_t)((payload[9] >> 5  | payload[10] << 3)                      & 0x07FF);
    channels[8]  = (uint16_t)((payload[11]      | payload[12] << 8)                      & 0x07FF);
    channels[9]  = (uint16_t)((payload[12] >> 3 | payload[13] << 5)                      & 0x07FF);
    channels[10] = (uint16_t)((payload[13] >> 6 | payload[14] << 2 | payload[15] << 10)  & 0x07FF);
    channels[11] = (uint16_t)((payload[15] >> 1 | payload[16] << 7)                      & 0x07FF);
    channels[12] = (uint16_t)((payload[16] >> 4 | payload[17] << 4)                      & 0x07FF);
    channels[13] = (uint16_t)((payload[17] >> 7 | payload[18] << 1 | payload[19] << 9)   & 0x07FF);
    channels[14] = (uint16_t)((payload[19] >> 2 | payload[20] << 6)                      & 0x07FF);
    channels[15] = (uint16_t)((payload[20] >> 5 | payload[21] << 3)                      & 0x07FF);

    uint8_t opt = buf[23] & 0xf;
    packet->ch17       = opt & SBUS_OPT_C17;
    packet->ch18       = opt & SBUS_OPT_C18;
    packet->failsafe   = opt & SBUS_OPT_FS;
    packet->frame_lost = opt & SBUS_OPT_FL;

    return SBUS_OK;
}

static enum sbus_err verify_packet()
{
    if (g_parser_buf[0] == SBUS_HEADER &&
        g_parser_buf[SBUS_PACKET_SIZE - 1] == SBUS_END)
    {
        return SBUS_OK;
    }
    else
    {
        return SBUS_FAIL;
    }
}

static enum sbus_err decode_packet()
{
    return sbus_decode(g_parser_buf, &g_last_packet);
}

static void parser_feed(uint8_t byte)
{
    switch (g_parser_state)
    {
        case WAIT_FOR_HEADER:
        case HEADER_SKIP:
            if (byte == SBUS_HEADER)
            {
                if (g_parser_state == HEADER_SKIP)
                {
                    // skip this header
                    g_parser_state = WAIT_FOR_HEADER;
                    break;
                }

                g_parser_buf[0] = SBUS_HEADER;
                g_packet_pos = 1;
                g_parser_state = PACKET;
            }
            break;

        case PACKET:
            g_parser_buf[g_packet_pos] = byte;
            g_packet_pos++;

            if (g_packet_pos >= SBUS_PACKET_SIZE)
            {
                if (verify_packet() == SBUS_OK &&
                    decode_packet() == SBUS_OK)
                {
                    // receive next packet
                    g_parser_state = WAIT_FOR_HEADER;
                }
                else
                {
                    /*
                        * SBUS header is '15' and packet end is '0'.
                        * In case the packet looks like this:
                        * 15 .. 15 .. 00 15 .. 15 .. 00
                        * |------------| |------------|
                        * We could have locked on like this:
                        * 15 .. 15 .. 00 15 .. 15 .. 00
                        *       |------------| |------------|
                        * In this situation we would loop forever.
                        * So if a desync happens it is safer to skip the next header
                        * which makes sure we are always moving inside each packet
                        * and not stuck a on single match.
                        *
                        *
                        * Actual example (observed when my transmitter was turned off):
                        *
                        * First match:
                        * 15 124 224 3 31 248 192 7 62 240 129 15 124 12 0 15 224 3 31 44 194 199 10 86 128
                        * ^Found header                          Real end^ ^Real header                 ^End mismatch
                        *                                      ^
                        *                                      |
                        *                          (this becomes new header)
                        * Next match:
                        * 15 124 12 0 15 224 3 31 44 194 199 10 86 128 15 124 224 3 31 248 192 7 62 240 129
                        *   Real end^ ^Real header
                        * You can see the decoder grabbed the next '15'.
                        * In the next step it would grab the next(er) '15' which would end up being the actual header
                        * and decoding would succeed.
                        */
                    g_parser_state = HEADER_SKIP;
                }

                g_packet_pos = 0;
            }
            break;
    }
}

void sbus_init(void)
{
    uart_init(SBUS_UART, SBUS_BAUD);
    uart_set_format(SBUS_UART, 8, 2, UART_PARITY_EVEN);
    gpio_set_function(SBUS_RX_PIN, GPIO_FUNC_UART);
    gpio_set_inover(SBUS_RX_PIN, GPIO_OVERRIDE_INVERT);
}

void sbus_read(struct sbus_packet *packet)
{
    uint8_t buf;
    while (uart_is_readable(SBUS_UART))
    {
        uart_read_blocking(SBUS_UART, &buf, 1);
        parser_feed(buf);
    }
    memcpy(packet, &g_last_packet, sizeof(struct sbus_packet));
}
