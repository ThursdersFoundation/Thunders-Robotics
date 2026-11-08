/**
 * @file packet_parser.h
 * @brief State-machine based packet parser for the UART3 host-comms link.
 *
 * Frame layout:
 *   [STX=0xA5] [TYPE:1] [LEN_H:1] [LEN_L:1] [PAYLOAD:LEN] [CRC_H:1] [CRC_L:1] [ETX=0x5A]
 *
 * The parser is byte-at-a-time so it can be fed from a ring buffer or
 * directly from an interrupt byte hook.
 */
#ifndef PACKET_PARSER_H
#define PACKET_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "crc.h"

#define PACKET_STX          0xA5U
#define PACKET_ETX          0x5AU
#define PACKET_MAX_PAYLOAD  240U

/** @brief Output of a successful parse. */
typedef struct
{
    uint8_t  type;
    uint8_t  payload[PACKET_MAX_PAYLOAD];
    uint16_t len;
    uint16_t crc;
} Packet_t;

/** @brief Parser state. */
typedef enum
{
    PARSER_STATE_STX = 0,
    PARSER_STATE_TYPE,
    PARSER_STATE_LEN_H,
    PARSER_STATE_LEN_L,
    PARSER_STATE_PAYLOAD,
    PARSER_STATE_CRC_H,
    PARSER_STATE_CRC_L,
    PARSER_STATE_ETX
} PacketParserState_t;

/** @brief Per-instance parser context. */
typedef struct
{
    PacketParserState_t state;
    uint8_t            type;
    uint16_t           len;
    uint16_t           payload_index;
    uint8_t            payload[PACKET_MAX_PAYLOAD];
    uint16_t           crc_recv;
    uint32_t           frames_received;
    uint32_t           crc_errors;
    uint32_t           frame_errors;
} PacketParser_t;

/**
 * @brief Initialise the parser state.
 * @param p  Parser context to initialise.
 */
void PacketParser_Init(PacketParser_t *p);

/**
 * @brief Feed one byte to the parser.  If a complete frame was received,
 *        the out packet is filled in and 1 is returned.  Otherwise 0.
 * @param p     Parser context.
 * @param byte  Incoming byte.
 * @param out   Output packet (valid only when return is 1).
 * @return 1 if a complete, valid packet was emitted; 0 otherwise.
 */
int PacketParser_Feed(PacketParser_t *p, uint8_t byte, Packet_t *out);

/**
 * @brief Reset the parser back to the STX state (e.g. after a timeout).
 * @param p  Parser context.
 */
void PacketParser_Reset(PacketParser_t *p);

#ifdef __cplusplus
}
#endif

#endif /* PACKET_PARSER_H */
