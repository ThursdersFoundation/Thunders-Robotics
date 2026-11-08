/**
 * @file packet_parser.c
 * @brief State-machine based packet parser implementation.
 *
 * Frame layout:
 *   [STX=0xA5] [TYPE:1] [LEN_H:1] [LEN_L:1] [PAYLOAD:LEN] [CRC_H:1] [CRC_L:1] [ETX=0x5A]
 *
 * The parser is byte-at-a-time so it can be fed from a ring buffer or
 * directly from an interrupt byte hook.
 */
#include "packet_parser.h"

#include <string.h>

void PacketParser_Init(PacketParser_t *p)
{
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
    p->state = PARSER_STATE_STX;
}

void PacketParser_Reset(PacketParser_t *p)
{
    if (p == NULL) {
        return;
    }
    p->state          = PARSER_STATE_STX;
    p->type           = 0;
    p->len            = 0;
    p->payload_index  = 0;
    p->crc_recv       = 0;
    p->frame_errors++;
}

int PacketParser_Feed(PacketParser_t *p, uint8_t byte, Packet_t *out)
{
    if (p == NULL || out == NULL) {
        return 0;
    }

    /* A new STX at any point resets the state machine.  This is what the
     * host test suite calls the "missing ETX" recovery test. */
    if (byte == PACKET_STX && p->state != PARSER_STATE_STX) {
        p->frame_errors++;
        p->state = PARSER_STATE_STX;
    }

    switch (p->state) {
    case PARSER_STATE_STX:
        if (byte == PACKET_STX) {
            p->state = PARSER_STATE_TYPE;
            p->crc_recv = 0;
            p->payload_index = 0;
            p->len = 0;
        }
        return 0;

    case PARSER_STATE_TYPE:
        p->type = byte;
        p->state = PARSER_STATE_LEN_H;
        return 0;

    case PARSER_STATE_LEN_H:
        p->len = (uint16_t)byte << 8;
        p->state = PARSER_STATE_LEN_L;
        return 0;

    case PARSER_STATE_LEN_L:
        p->len |= byte;
        if (p->len > PACKET_MAX_PAYLOAD) {
            p->frame_errors++;
            p->state = PARSER_STATE_STX;
            return 0;
        }
        if (p->len == 0) {
            p->state = PARSER_STATE_CRC_H;
        } else {
            p->state = PARSER_STATE_PAYLOAD;
        }
        return 0;

    case PARSER_STATE_PAYLOAD:
        if (p->payload_index < PACKET_MAX_PAYLOAD) {
            p->payload[p->payload_index++] = byte;
        }
        if (p->payload_index >= p->len) {
            p->state = PARSER_STATE_CRC_H;
        }
        return 0;

    case PARSER_STATE_CRC_H:
        p->crc_recv = (uint16_t)byte << 8;
        p->state = PARSER_STATE_CRC_L;
        return 0;

    case PARSER_STATE_CRC_L:
        p->crc_recv |= byte;
        p->state = PARSER_STATE_ETX;
        return 0;

    case PARSER_STATE_ETX:
        if (byte != PACKET_ETX) {
            p->frame_errors++;
            p->state = PARSER_STATE_STX;
            return 0;
        }
        /* Validate the CRC over TYPE + LEN_H + LEN_L + PAYLOAD. */
        uint16_t calc = CRC16_CCITT_Init();
        calc = CRC16_CCITT_Update(calc, p->type);
        calc = CRC16_CCITT_Update(calc, (uint8_t)(p->len >> 8));
        calc = CRC16_CCITT_Update(calc, (uint8_t)(p->len & 0xFFU));
        for (uint16_t i = 0; i < p->len; i++) {
            calc = CRC16_CCITT_Update(calc, p->payload[i]);
        }
        if (calc != p->crc_recv) {
            p->crc_errors++;
            p->state = PARSER_STATE_STX;
            return 0;
        }
        out->type = p->type;
        out->len  = p->len;
        out->crc  = p->crc_recv;
        if (p->len > 0) {
            memcpy(out->payload, p->payload, p->len);
        }
        p->frames_received++;
        p->state = PARSER_STATE_STX;
        return 1;

    default:
        p->state = PARSER_STATE_STX;
        return 0;
    }
}
