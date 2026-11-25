/**
 * @file    RobotCommon.h
 * @brief   Shared protocol layer for all Thunders Robotic Arduino nodes.
 *
 * Identical wire format to the STM32 master (see
 * firmware/stm32/Middlewares/Communication/Inc/protocol.h):
 *   [A5][5A][type][len][payload...][crc_lo][crc_hi]
 * CRC-16/CCITT-FALSE over type..payload, payload little-endian.
 */
#ifndef THUNDERS_ROBOT_COMMON_H
#define THUNDERS_ROBOT_COMMON_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Frame protocol                                                      */
/* ------------------------------------------------------------------ */
#define PROTOCOL_PREAMBLE0    0xA5U
#define PROTOCOL_PREAMBLE1    0x5AU
#define PROTOCOL_HEADER_SIZE  4U
#define PROTOCOL_CRC_SIZE     2U
#define PROTOCOL_MAX_PAYLOAD  64U
#define PROTOCOL_MAX_FRAME    (PROTOCOL_HEADER_SIZE + PROTOCOL_MAX_PAYLOAD + PROTOCOL_CRC_SIZE)

typedef enum
{
    MSG_PING            = 0x01,
    MSG_ACK             = 0x02,
    MSG_NACK            = 0x03,
    MSG_RESET           = 0x04,
    MSG_CMD_VELOCITY    = 0x10,
    MSG_CMD_ENABLE      = 0x11,
    MSG_CMD_SET_PID     = 0x12,
    MSG_CMD_GET_ODOM    = 0x13,
    MSG_CMD_GET_STATUS  = 0x14,
    MSG_CMD_SERVO       = 0x15,
    MSG_CMD_STEPPER     = 0x16,
    MSG_TLM_ODOM        = 0x20,
    MSG_TLM_STATUS      = 0x21,
    MSG_TLM_SCAN        = 0x22,
    MSG_TLM_IMU         = 0x24,
    MSG_TLM_ENCODERS    = 0x25,
    MSG_TLM_POWER       = 0x26,
    MSG_EVT_FAULT       = 0x30,
    MSG_EVT_TEXT        = 0x31,
    MSG_EVT_HEARTBEAT   = 0x32,
    MSG_CAN_BRIDGE_TX   = 0x40,
    MSG_CAN_BRIDGE_RX   = 0x41
} MsgType;

#define PROTO_PID_SCALE 10000.0f

/* ------------------------------------------------------------------ */
/* CRC-16/CCITT-FALSE                                                  */
/* ------------------------------------------------------------------ */
#define CRC16_CCITT_INIT 0xFFFFU
uint16_t crc16_byte(uint16_t crc, uint8_t byte);
uint16_t crc16(const uint8_t *data, size_t length);

/* ------------------------------------------------------------------ */
/* Streaming frame parser                                              */
/* ------------------------------------------------------------------ */
typedef enum
{
    PARSER_SYNC0 = 0,
    PARSER_SYNC1,
    PARSER_TYPE,
    PARSER_LENGTH,
    PARSER_PAYLOAD,
    PARSER_CRC_LO,
    PARSER_CRC_HI
} ParserState;

typedef struct
{
    uint8_t  type;
    uint8_t  length;
    uint8_t  payload[PROTOCOL_MAX_PAYLOAD];
} PacketFrame;

typedef struct
{
    ParserState state;
    uint8_t     type;
    uint8_t     length;
    uint8_t     payload[PROTOCOL_MAX_PAYLOAD];
    uint16_t    payload_index;
    uint16_t    crc_rx;
    uint16_t    crc_calc;
    uint32_t    frames_ok;
    uint32_t    frames_bad_crc;
    uint32_t    bytes_dropped;
} PacketParser;

void   PacketParser_Init(PacketParser *p);
bool   PacketParser_Feed(PacketParser *p, uint8_t byte, PacketFrame *out);
size_t Packet_Build(uint8_t *buf, size_t bufSize, uint8_t type,
                    const uint8_t *payload, uint8_t len);

/* ------------------------------------------------------------------ */
/* CAN 500 kbit/s backbone: ID map (same as master)                    */
/* ------------------------------------------------------------------ */
#define CAN_ID_HEARTBEAT_BASE   0x0A0U
#define CAN_ID_MOTOR_VELOCITY   0x100U
#define CAN_ID_MOTOR_ENABLE     0x101U
#define CAN_ID_MOTOR_SET_PID    0x102U
#define CAN_ID_ACT_SERVO        0x110U
#define CAN_ID_ACT_STEPPER      0x111U
#define CAN_ID_SENSOR_CONFIG    0x120U
#define CAN_ID_MASTER_TIME      0x130U
#define CAN_ID_MOTOR_ENCODERS0  0x200U
#define CAN_ID_MOTOR_ENCODERS1  0x201U
#define CAN_ID_MOTOR_PWM        0x210U
#define CAN_ID_MOTOR_CURRENT    0x211U
#define CAN_ID_SENSOR_DISTANCES 0x220U
#define CAN_ID_SENSOR_IMU       0x221U
#define CAN_ID_SENSOR_POWER     0x230U
#define CAN_ID_ACT_STATUS       0x2E0U
#define CAN_ID_FAULT_BASE       0x300U

typedef enum
{
    CAN_NODE_MASTER      = 0x00,
    CAN_NODE_MOTOR       = 0x01,
    CAN_NODE_SENSOR      = 0x02,
    CAN_NODE_ACTUATOR    = 0x03,
    CAN_NODE_COMM        = 0x04,
    CAN_NODE_DIAGNOSTICS = 0x05
} CanNodeId;

#define CAN_VEL_SCALE 1000.0f
#define CAN_YAW_SCALE 100.0f
#define CAN_DPS_SCALE 100.0f

/* Fault codes (subset mirrored from safety_manager.h) */
#define FAULT_NONE             0x00000000UL
#define FAULT_TASK_TIMEOUT     0x00000001UL
#define FAULT_BATTERY_CRITICAL 0x00000002UL
#define FAULT_OVERCURRENT      0x00000004UL
#define FAULT_TILT             0x00000008UL
#define FAULT_OBSTACLE         0x00000010UL
#define FAULT_COMM_TIMEOUT     0x00000020UL
#define FAULT_IMU_TIMEOUT      0x00000040UL
#define FAULT_LIDAR_ERROR      0x00000080UL
#define FAULT_CAN_TIMEOUT      0x00000100UL
#define FAULT_MOTOR_STALL      0x00000200UL
#define FAULT_ESTOP_BUTTON     0x00000400UL
#define FAULT_WATCHDOG         0x00000800UL

/* Little-endian field helpers */
static inline void can_put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void can_put_i16(uint8_t *p, int16_t v)  { can_put_u16(p, (uint16_t)v); }
static inline void can_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline void can_put_i32(uint8_t *p, int32_t v) { can_put_u32(p, (uint32_t)v); }
static inline uint16_t can_get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static inline int16_t  can_get_i16(const uint8_t *p) { return (int16_t)can_get_u16(p); }
static inline uint32_t can_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int32_t  can_get_i32(const uint8_t *p) { return (int32_t)can_get_u32(p); }

/* CAN payload codecs (identical to the master's can_protocol.c) */
uint8_t Can_EncodeVelocity(uint8_t *buf, const float wheelSpeed[4]);
uint8_t Can_EncodeServo(uint8_t *buf, uint8_t servoId, float angleDeg);
uint8_t Can_EncodeStepper(uint8_t *buf, uint8_t axis, int32_t pos, int16_t vel);
uint8_t Can_EncodeDistances(uint8_t *buf, const uint16_t mm[4]);
uint8_t Can_EncodeImu(uint8_t *buf, float yawDeg, const float dps[3]);
uint8_t Can_EncodePower(uint8_t *buf, uint16_t batteryMv, uint16_t currentMa, float tempC);
uint8_t Can_EncodeHeartbeat(uint8_t *buf, uint8_t state, uint16_t uptimeS, uint16_t faults);
bool Can_DecodeVelocity(const uint8_t *data, uint8_t dlc, float wheelSpeed[4]);
bool Can_DecodeServo(const uint8_t *data, uint8_t dlc, uint8_t *servoId, float *angleDeg);
bool Can_DecodeStepper(const uint8_t *data, uint8_t dlc, uint8_t *axis, int32_t *pos, int16_t *vel);
bool Can_DecodeDistances(const uint8_t *data, uint8_t dlc, uint16_t mm[4]);
bool Can_DecodeImu(const uint8_t *data, uint8_t dlc, float *yawDeg, float dps[3]);
bool Can_DecodePower(const uint8_t *data, uint8_t dlc, uint16_t *mv, uint16_t *ma, float *tempC);

/* Fixed-point clamp+pack used by velocity command payloads */
static inline int16_t proto_pack_i16(float v, float scale)
{
    if (v > 32767.0f / scale)  { v = 32767.0f / scale; }
    if (v < -32768.0f / scale) { v = -32768.0f / scale; }
    return (int16_t)(v * scale);
}

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_ROBOT_COMMON_H */
