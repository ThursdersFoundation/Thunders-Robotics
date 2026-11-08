/**
 * @file can_protocol.h
 * @brief CAN1 host-comms protocol layer.  Encodes the project's standard-frame
 *        message IDs, manages a callback dispatch table keyed by 11-bit ID,
 *        and exposes a non-blocking receive primitive that drains CAN1 RX
 *        FIFO0.
 */
#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "can.h"   /* pulls in CAN_ID_* and hcan1 */

/* -------------------------------------------------------------------------
 * CAN message ID layout (11-bit standard frames).
 *   Bits [10..8] = direction (0x01 = cmd, 0x02 = tlm)
 *   Bits [7..0]  = MsgType_t
 * ------------------------------------------------------------------------- */
#define CAN_ID_DIR_MASK    0x700U
#define CAN_ID_TYPE_MASK   0x0FFU
#define CAN_ID_DIR_CMD     0x100U
#define CAN_ID_DIR_TLM     0x200U

/** @brief Maximum number of per-ID handlers in the dispatch table. */
#define CAN_PROTO_MAX_HANDLERS 8U

/** @brief CAN RX FIFO drain depth. */
#define CAN_PROTO_FIFO_DEPTH 3U

/** @brief Pointer to a per-ID handler. */
typedef HAL_StatusTypeDef (*can_handler_t)(uint16_t std_id,
                                            const uint8_t *data,
                                            uint8_t dlc);

/**
 * @brief Initialise the CAN protocol layer.  Assumes MX_CAN1_Init has
 *        already configured the filter list.  Installs the RX FIFO0 callback.
 * @param hcan  Handle to CAN1.
 * @return HAL_OK on success, HAL_ERROR on failure.
 */
HAL_StatusTypeDef CANProto_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief Send a CAN1 standard-frame message with the given 11-bit ID and
 *        payload.  DLC is clamped to 8 bytes.
 * @param std_id  11-bit CAN ID (typically built from CAN_ID_DIR_* | type).
 * @param data    Pointer to up to 8 data bytes.
 * @param dlc     Data length in bytes (1..8).
 * @return HAL_OK on success, HAL_ERROR on no free mailbox / failure.
 */
HAL_StatusTypeDef CANProto_Send(uint16_t std_id, const uint8_t *data, uint8_t dlc);

/**
 * @brief Register a per-ID handler.  Subsequent CAN frames with matching
 *        std_id will invoke the handler from the CAN1 RX0 ISR.
 * @param std_id  11-bit CAN ID to match.
 * @param cb      Handler function pointer.
 * @return HAL_OK on success, HAL_ERROR if the dispatch table is full.
 */
HAL_StatusTypeDef CANProto_RegisterHandler(uint16_t std_id, can_handler_t cb);

/**
 * @brief Non-blocking receive.  Drains CAN1 RX FIFO0; called by the
 *        CommTask to surface messages that did not have a registered handler.
 * @param id_out   Output CAN ID.
 * @param data_out Output buffer (8 bytes).
 * @param dlc_out  Output DLC.
 * @return HAL_OK if a message was received, HAL_ERROR if FIFO was empty.
 */
HAL_StatusTypeDef CANProto_Receive(uint16_t *id_out, uint8_t *data_out, uint8_t *dlc_out);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROTOCOL_H */
