/**
 * @file protocol.h
 * @brief Top-level host-protocol API for the Thunders Robotic firmware.
 *
 * Defines the message-type enumeration used across both the UART and the
 * CAN transports, the `RobotState_t` telemetry struct, and the dispatch
 * entry points `Protocol_Init`, `Protocol_SendTelemetry`, and
 * `Protocol_HandleCommand`.
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* -------------------------------------------------------------------------
 * Message types.  These are encoded into the 1-byte TYPE field of a UART
 * framed packet and the lower 8 bits of the CAN standard-frame ID.  The
 * higher 8 bits of the CAN ID encode the direction (0x01 = cmd, 0x02 = tlm).
 * ------------------------------------------------------------------------- */
typedef enum
{
    MSG_TYPE_PING       = 0x00,  /* Host -> Robot, 0-byte ping */
    MSG_TYPE_PONG       = 0x01,  /* Robot -> Host, 16-byte pong (uptime, fw, hw) */
    MSG_TYPE_CMD_VEL    = 0x02,  /* Host -> Robot, 8-byte float[2] {lin, ang} */
    MSG_TYPE_SET_MODE   = 0x03,  /* Host -> Robot, 1-byte mode enum */
    MSG_TYPE_ESTOP      = 0x04,  /* Host -> Robot, 0-byte ESTOP */
    MSG_TYPE_TELEMETRY  = 0x05,  /* Robot -> Host, packed RobotState_t */
    MSG_TYPE_HEALTH     = 0x06,  /* Robot -> Host, fault mask + battery + link */
    MSG_TYPE_LIDAR      = 0x07,  /* Robot -> Host, per-frame distance table */
    MSG_TYPE_ACK        = 0x08   /* Robot -> Host, 2-byte ack {type, status} */
} MsgType_t;

/** @brief Telemetry snapshot pushed to the host every 100 ms. */
typedef struct
{
    float    lin_mps;
    float    ang_rps;
    float    rpm_left;
    float    rpm_right;
    int16_t  pwm_left;
    int16_t  pwm_right;
    float    imu_yaw;
    float    imu_pitch;
    float    imu_roll;
    float    battery_v;
    uint8_t  battery_pct;
    uint16_t faults;        /* bitfield from SafetyFault_t */
    uint32_t uptime_ms;
} RobotState_t;

/**
 * @brief Initialise the protocol layer.  Idempotent.  Must be called after
 *        UARTProto_Init and CANProto_Init have run (Comm_Init does this).
 * @return HAL_OK on success, HAL_ERROR on failure.
 */
HAL_StatusTypeDef Protocol_Init(void);

/**
 * @brief Pack the current RobotState_t into a MSG_TYPE_TELEMETRY frame and
 *        send it via the communication manager.
 * @param state  Pointer to the telemetry struct to send.
 * @return HAL_OK on success, HAL_ERROR on transport failure.
 */
HAL_StatusTypeDef Protocol_SendTelemetry(const RobotState_t *state);

/**
 * @brief Handle an inbound command frame.  Dispatches by MsgType_t and
 *        invokes the motor controller, safety manager, etc.
 * @param buf  Pointer to the raw payload (without framing).
 * @param len  Payload length in bytes.
 * @return HAL_OK on success, HAL_ERROR on malformed payload.
 */
HAL_StatusTypeDef Protocol_HandleCommand(const uint8_t *buf, uint16_t len);

/**
 * @brief Build the 16-byte PONG payload in the given buffer (used as the
 *        response to a MSG_TYPE_PING from the host).
 * @param out  16-byte output buffer.
 */
void Protocol_BuildPong(uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
