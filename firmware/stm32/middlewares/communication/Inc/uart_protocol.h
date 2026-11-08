/**
 * @file uart_protocol.h
 * @brief UART3 host-comms protocol layer.  Builds framed outbound packets
 *        (STX / TYPE / LEN-H / LEN-L / PAYLOAD / CRC-H / CRC-L / ETX), pushes
 *        inbound bytes into a ring buffer, and exposes a packet dequeue API.
 */
#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "packet_parser.h"
#include "ring_buffer.h"

/* -------------------------------------------------------------------------
 * Frame format constants.
 * ------------------------------------------------------------------------- */
#define UART_PROTO_STX          0xA5U
#define UART_PROTO_ETX          0x5AU
#define UART_PROTO_MAX_PAYLOAD  240U
#define UART_PROTO_HEADER_LEN   6U   /* STX + TYPE + LEN_H + LEN_L + CRC_H + CRC_L */
#define UART_PROTO_MAX_FRAME    (UART_PROTO_HEADER_LEN + UART_PROTO_MAX_PAYLOAD + 1U)

/** @brief RX queue depth (number of completed packets held). */
#define UART_PROTO_RX_QUEUE_LEN 4U

/** @brief Per-instance state of the UART protocol. */
typedef struct
{
    UART_HandleTypeDef *huart;
    RingBuffer_t        rx_ring;
    uint8_t             rx_ring_mem[UART_PROTO_RX_QUEUE_LEN * UART_PROTO_MAX_FRAME];
    PacketParser_t      parser;
    uint8_t             tx_buf[UART_PROTO_MAX_FRAME];
} UARTProto_t;

/**
 * @brief Initialise the UART protocol layer and start the RX ring-buffer
 *        DMA-coupled receive loop.  Returns HAL_OK on success.
 * @param huart  Handle to USART3.
 */
HAL_StatusTypeDef UARTProto_Init(UART_HandleTypeDef *huart);

/**
 * @brief Build a framed packet (STX + TYPE + LEN + PAYLOAD + CRC + ETX) into
 *        the internal TX buffer and transmit it via HAL_UART_Transmit_DMA.
 * @param type     Message type (one of MsgType_t).
 * @param payload  Pointer to the payload bytes (may be NULL for 0-byte types).
 * @param len      Payload length in bytes (0..UART_PROTO_MAX_PAYLOAD).
 * @return HAL_OK on success, HAL_ERROR on transport/buffer failure.
 */
HAL_StatusTypeDef UARTProto_Send(uint8_t type, const uint8_t *payload, uint16_t len);

/**
 * @brief Feed any newly-arrived RX bytes through the packet parser and into
 *        the receive queue.
 * @return HAL_OK on success (even if no complete packet was produced),
 *         HAL_ERROR on parser hard failure.
 */
HAL_StatusTypeDef UARTProto_Poll(void);

/**
 * @brief Dequeue the next complete inbound packet.
 * @param out  Output pointer; the caller receives a copy of the Packet_t.
 * @return HAL_OK if a packet was dequeued, HAL_ERROR if the queue was empty.
 */
HAL_StatusTypeDef UARTProto_Dequeue(Packet_t *out);

#ifdef __cplusplus
}
#endif

#endif /* UART_PROTOCOL_H */
