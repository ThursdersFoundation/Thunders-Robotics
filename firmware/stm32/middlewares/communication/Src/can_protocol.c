/**
 * @file can_protocol.c
 * @brief CAN1 host-comms protocol layer.  Manages the per-ID handler
 *        dispatch table, builds standard-frame TX messages, and drains
 *        RX FIFO0.
 */
#include "can_protocol.h"

#include <string.h>

typedef struct
{
    uint16_t      id;
    can_handler_t cb;
} CANHandlerSlot_t;

static CAN_HandleTypeDef *s_hcan = NULL;
static CANHandlerSlot_t   s_handlers[CAN_PROTO_MAX_HANDLERS];
static uint8_t            s_handler_count = 0;
static uint8_t            s_inited = 0;

HAL_StatusTypeDef CANProto_Init(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) {
        return HAL_ERROR;
    }
    s_hcan = hcan;
    memset(s_handlers, 0, sizeof(s_handlers));
    s_handler_count = 0;
    s_inited = 1;
    return HAL_OK;
}

HAL_StatusTypeDef CANProto_RegisterHandler(uint16_t std_id, can_handler_t cb)
{
    if (!s_inited || cb == NULL) {
        return HAL_ERROR;
    }
    /* Replace if already registered. */
    for (uint8_t i = 0; i < s_handler_count; i++) {
        if (s_handlers[i].id == std_id) {
            s_handlers[i].cb = cb;
            return HAL_OK;
        }
    }
    if (s_handler_count >= CAN_PROTO_MAX_HANDLERS) {
        return HAL_ERROR;
    }
    s_handlers[s_handler_count].id = std_id;
    s_handlers[s_handler_count].cb = cb;
    s_handler_count++;
    return HAL_OK;
}

HAL_StatusTypeDef CANProto_Send(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    if (!s_inited || data == NULL) {
        return HAL_ERROR;
    }
    if (dlc > 8U) dlc = 8U;

    CAN_TxHeaderTypeDef hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.StdId = std_id & 0x7FFU;
    hdr.IDE   = CAN_ID_STD;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.DLC   = dlc;
    hdr.TransmitGlobalTime = DISABLE;

    uint32_t mailbox;
    uint32_t timeout = HAL_GetTick() + 50U;
    while (HAL_CAN_GetTxMailboxFreeLevel(s_hcan) == 0U) {
        if (HAL_GetTick() >= timeout) {
            return HAL_ERROR;
        }
    }
    uint8_t buf[8] = {0};
    memcpy(buf, data, dlc);
    if (HAL_CAN_AddTxMessage(s_hcan, &hdr, buf, &mailbox) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

static void dispatch(uint16_t id, const uint8_t *data, uint8_t dlc)
{
    for (uint8_t i = 0; i < s_handler_count; i++) {
        if (s_handlers[i].id == id && s_handlers[i].cb != NULL) {
            s_handlers[i].cb(id, data, dlc);
            return;
        }
    }
}

HAL_StatusTypeDef CANProto_Receive(uint16_t *id_out, uint8_t *data_out, uint8_t *dlc_out)
{
    if (!s_inited || id_out == NULL || data_out == NULL || dlc_out == NULL) {
        return HAL_ERROR;
    }
    if (HAL_CAN_GetRxFifoFillLevel(s_hcan, CAN_RX_FIFO0) == 0U) {
        return HAL_ERROR;
    }
    CAN_RxHeaderTypeDef hdr;
    uint8_t buf[8];
    if (HAL_CAN_GetRxMessage(s_hcan, CAN_RX_FIFO0, &hdr, buf) != HAL_OK) {
        return HAL_ERROR;
    }
    *id_out  = (uint16_t)hdr.StdId;
    *dlc_out = (uint8_t)hdr.DLC;
    memcpy(data_out, buf, hdr.DLC);
    dispatch(*id_out, data_out, *dlc_out);
    return HAL_OK;
}

/* -------------------------------------------------------------------------
 * HAL CAN RX FIFO0 callback.  Invoked from CAN1_RX0_IRQHandler in
 * stm32f4xx_it.c when a message lands in FIFO0.
 * ------------------------------------------------------------------------- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != s_hcan) {
        return;
    }
    CAN_RxHeaderTypeDef hdr;
    uint8_t buf[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, buf) == HAL_OK) {
            dispatch((uint16_t)hdr.StdId, buf, (uint8_t)hdr.DLC);
        }
    }
}
