/**
 * @file uart_protocol.c
 * @brief UART3 host-comms protocol layer implementation.  Builds framed
 *        outbound packets (STX + TYPE + LEN + PAYLOAD + CRC + ETX) and feeds
 *        inbound bytes through the state-machine packet parser.
 */
#include "uart_protocol.h"

#include <string.h>

static UARTProto_t s_proto;
static uint8_t     s_dma_rxbuf[USART3_RX_BUF_SIZE];

/* Small fixed-size packet queue populated by the parser. */
static Packet_t   s_rx_queue[UART_PROTO_RX_QUEUE_LEN];
static volatile uint8_t s_rx_head = 0;
static volatile uint8_t s_rx_tail = 0;

HAL_StatusTypeDef UARTProto_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return HAL_ERROR;
    }
    memset(&s_proto, 0, sizeof(s_proto));
    s_proto.huart = huart;
    RingBuffer_Init(&s_proto.rx_ring, s_proto.rx_ring_mem, sizeof(s_proto.rx_ring_mem));
    PacketParser_Init(&s_proto.parser);

    /* Kick off a circular DMA receive into s_dma_rxbuf.  Each completion
     * callback (HAL_UARTEx_RxEventCallback in stm32f4xx_it.c) pushes the
     * received bytes into the parser's ring buffer. */
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, s_dma_rxbuf, sizeof(s_dma_rxbuf)) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef UARTProto_Send(uint8_t type, const uint8_t *payload, uint16_t len)
{
    if (len > UART_PROTO_MAX_PAYLOAD) {
        return HAL_ERROR;
    }
    uint8_t *tx = s_proto.tx_buf;
    tx[0] = UART_PROTO_STX;
    tx[1] = type;
    tx[2] = (uint8_t)(len >> 8);
    tx[3] = (uint8_t)(len & 0xFFU);
    if (len > 0 && payload != NULL) {
        memcpy(&tx[4], payload, len);
    }
    uint16_t crc = CRC16_CCITT_Init();
    crc = CRC16_CCITT_Update(crc, type);
    crc = CRC16_CCITT_Update(crc, tx[2]);
    crc = CRC16_CCITT_Update(crc, tx[3]);
    for (uint16_t i = 0; i < len; i++) {
        crc = CRC16_CCITT_Update(crc, payload[i]);
    }
    tx[4 + len] = (uint8_t)(crc >> 8);
    tx[4 + len + 1] = (uint8_t)(crc & 0xFFU);
    tx[4 + len + 2] = UART_PROTO_ETX;

    uint16_t total = (uint16_t)(4 + len + 3);
    return HAL_UART_Transmit_DMA(s_proto.huart, tx, total);
}

HAL_StatusTypeDef UARTProto_Poll(void)
{
    /* Drain any completed DMA bytes into the parser. */
    uint16_t dma_pos = sizeof(s_dma_rxbuf) - __HAL_DMA_GET_COUNTER(s_proto.huart->hdmarx);
    static uint16_t s_last_pos = 0;
    while (s_last_pos != dma_pos) {
        uint8_t b = s_dma_rxbuf[s_last_pos];
        Packet_t pkt;
        if (PacketParser_Feed(&s_proto.parser, b, &pkt)) {
            uint8_t next = (uint8_t)((s_rx_head + 1U) % UART_PROTO_RX_QUEUE_LEN);
            if (next != s_rx_tail) {
                s_rx_queue[s_rx_head] = pkt;
                s_rx_head = next;
            }
        }
        s_last_pos = (uint16_t)((s_last_pos + 1U) % sizeof(s_dma_rxbuf));
    }
    return HAL_OK;
}

HAL_StatusTypeDef UARTProto_Dequeue(Packet_t *out)
{
    if (out == NULL) {
        return HAL_ERROR;
    }
    if (s_rx_head == s_rx_tail) {
        return HAL_ERROR;
    }
    *out = s_rx_queue[s_rx_tail];
    s_rx_tail = (uint8_t)((s_rx_tail + 1U) % UART_PROTO_RX_QUEUE_LEN);
    return HAL_OK;
}

/* Hook called from the HAL_UARTEx_RxEventCallback in stm32f4xx_it.c.
 * Provides the byte count consumed by the DMA so we can immediately run
 * UARTProto_Poll() from the ISR context.  Marked weak so the main comm
 * task can also drive polling. */
__attribute__((weak))
void UARTProto_OnRxEvent(uint16_t bytes_received)
{
    (void)bytes_received;
    UARTProto_Poll();
}
