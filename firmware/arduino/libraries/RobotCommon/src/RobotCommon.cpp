/**
 * @file    RobotCommon.cpp
 * @brief   Implementation of the shared frame layer and CAN codecs.
 *          Portable C (AVR-gcc + host gcc both supported for unit tests).
 */
#include "RobotCommon.h"

extern "C" {

/* ------------------------------------------------------------------ */
/* CRC-16/CCITT-FALSE                                                  */
/* ------------------------------------------------------------------ */
uint16_t crc16_byte(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)((uint16_t)byte << 8);
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        if (crc & 0x8000U) { crc = (uint16_t)((crc << 1) ^ 0x1021U); }
        else               { crc = (uint16_t)(crc << 1); }
    }
    return crc;
}

uint16_t crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = CRC16_CCITT_INIT;
    if (data != NULL)
    {
        for (size_t i = 0U; i < length; i++) { crc = crc16_byte(crc, data[i]); }
    }
    return crc;
}

/* ------------------------------------------------------------------ */
/* Frame parser (same state machine as the master)                     */
/* ------------------------------------------------------------------ */
void PacketParser_Init(PacketParser *p)
{
    memset(p, 0, sizeof(*p));
    p->state = PARSER_SYNC0;
}

bool PacketParser_Feed(PacketParser *p, uint8_t byte, PacketFrame *out)
{
    switch (p->state)
    {
    case PARSER_SYNC0:
        if (byte == PROTOCOL_PREAMBLE0) { p->state = PARSER_SYNC1; }
        else { p->bytes_dropped++; }
        break;

    case PARSER_SYNC1:
        if (byte == PROTOCOL_PREAMBLE1) { p->state = PARSER_TYPE; }
        else if (byte == PROTOCOL_PREAMBLE0) { p->bytes_dropped++; }
        else { p->state = PARSER_SYNC0; p->bytes_dropped += 2U; }
        break;

    case PARSER_TYPE:
        p->type = byte;
        p->crc_calc = crc16_byte(CRC16_CCITT_INIT, byte);
        p->state = PARSER_LENGTH;
        break;

    case PARSER_LENGTH:
        if (byte > PROTOCOL_MAX_PAYLOAD)
        {
            p->state = PARSER_SYNC0;
            p->bytes_dropped++;
            break;
        }
        p->length = byte;
        p->payload_index = 0U;
        p->crc_calc = crc16_byte(p->crc_calc, byte);
        p->state = (p->length == 0U) ? PARSER_CRC_LO : PARSER_PAYLOAD;
        break;

    case PARSER_PAYLOAD:
        p->payload[p->payload_index++] = byte;
        p->crc_calc = crc16_byte(p->crc_calc, byte);
        if (p->payload_index >= p->length) { p->state = PARSER_CRC_LO; }
        break;

    case PARSER_CRC_LO:
        p->crc_rx = byte;
        p->state = PARSER_CRC_HI;
        break;

    case PARSER_CRC_HI:
        p->crc_rx |= (uint16_t)((uint16_t)byte << 8);
        p->state = PARSER_SYNC0;
        if (p->crc_rx == p->crc_calc)
        {
            p->frames_ok++;
            if (out != NULL)
            {
                out->type   = p->type;
                out->length = p->length;
                memcpy(out->payload, p->payload, p->length);
            }
            return true;
        }
        p->frames_bad_crc++;
        break;

    default:
        p->state = PARSER_SYNC0;
        break;
    }
    return false;
}

size_t Packet_Build(uint8_t *buf, size_t bufSize, uint8_t type,
                    const uint8_t *payload, uint8_t len)
{
    if (buf == NULL || len > PROTOCOL_MAX_PAYLOAD ||
        bufSize < (size_t)PROTOCOL_HEADER_SIZE + len + PROTOCOL_CRC_SIZE)
    {
        return 0U;
    }
    size_t idx = 0U;
    buf[idx++] = PROTOCOL_PREAMBLE0;
    buf[idx++] = PROTOCOL_PREAMBLE1;
    buf[idx++] = type;
    uint16_t crc = crc16_byte(CRC16_CCITT_INIT, type);
    buf[idx++] = len;
    crc = crc16_byte(crc, len);
    for (uint8_t i = 0U; i < len; i++)
    {
        buf[idx++] = payload[i];
        crc = crc16_byte(crc, payload[i]);
    }
    buf[idx++] = (uint8_t)(crc & 0xFFU);
    buf[idx++] = (uint8_t)(crc >> 8);
    return idx;
}

/* ------------------------------------------------------------------ */
/* CAN payload codecs (mirror of master can_protocol.c)                */
/* ------------------------------------------------------------------ */
uint8_t Can_EncodeVelocity(uint8_t *buf, const float wheelSpeed[4])
{
    if (buf == NULL || wheelSpeed == NULL) { return 0U; }
    for (uint8_t i = 0U; i < 4U; i++)
    {
        can_put_i16(&buf[i * 2U], proto_pack_i16(wheelSpeed[i], CAN_VEL_SCALE));
    }
    return 8U;
}

uint8_t Can_EncodeServo(uint8_t *buf, uint8_t servoId, float angleDeg)
{
    if (buf == NULL) { return 0U; }
    buf[0] = servoId;
    can_put_i16(&buf[1], proto_pack_i16(angleDeg, 10.0f));
    return 3U;
}

uint8_t Can_EncodeStepper(uint8_t *buf, uint8_t axis, int32_t pos, int16_t vel)
{
    if (buf == NULL) { return 0U; }
    buf[0] = axis;
    can_put_i32(&buf[1], pos);
    can_put_i16(&buf[5], vel);
    return 7U;
}

uint8_t Can_EncodeDistances(uint8_t *buf, const uint16_t mm[4])
{
    if (buf == NULL || mm == NULL) { return 0U; }
    for (uint8_t i = 0U; i < 4U; i++) { can_put_u16(&buf[i * 2U], mm[i]); }
    return 8U;
}

uint8_t Can_EncodeImu(uint8_t *buf, float yawDeg, const float dps[3])
{
    if (buf == NULL || dps == NULL) { return 0U; }
    can_put_i16(&buf[0], proto_pack_i16(yawDeg, CAN_YAW_SCALE));
    can_put_i16(&buf[2], proto_pack_i16(dps[0], CAN_DPS_SCALE));
    can_put_i16(&buf[4], proto_pack_i16(dps[1], CAN_DPS_SCALE));
    can_put_i16(&buf[6], proto_pack_i16(dps[2], CAN_DPS_SCALE));
    return 8U;
}

uint8_t Can_EncodePower(uint8_t *buf, uint16_t batteryMv, uint16_t currentMa, float tempC)
{
    if (buf == NULL) { return 0U; }
    can_put_u16(&buf[0], batteryMv);
    can_put_u16(&buf[2], currentMa);
    can_put_i16(&buf[4], proto_pack_i16(tempC, 10.0f));
    return 6U;
}

uint8_t Can_EncodeHeartbeat(uint8_t *buf, uint8_t state, uint16_t uptimeS, uint16_t faults)
{
    if (buf == NULL) { return 0U; }
    buf[0] = state;
    can_put_u16(&buf[1], uptimeS);
    can_put_u16(&buf[3], faults);
    return 5U;
}

bool Can_DecodeVelocity(const uint8_t *data, uint8_t dlc, float wheelSpeed[4])
{
    if (data == NULL || wheelSpeed == NULL || dlc < 8U) { return false; }
    for (uint8_t i = 0U; i < 4U; i++)
    {
        wheelSpeed[i] = (float)can_get_i16(&data[i * 2U]) / CAN_VEL_SCALE;
    }
    return true;
}

bool Can_DecodeServo(const uint8_t *data, uint8_t dlc, uint8_t *servoId, float *angleDeg)
{
    if (data == NULL || dlc < 3U) { return false; }
    if (servoId)  { *servoId = data[0]; }
    if (angleDeg) { *angleDeg = (float)can_get_i16(&data[1]) / 10.0f; }
    return true;
}

bool Can_DecodeStepper(const uint8_t *data, uint8_t dlc, uint8_t *axis, int32_t *pos, int16_t *vel)
{
    if (data == NULL || dlc < 7U) { return false; }
    if (axis) { *axis = data[0]; }
    if (pos)  { *pos = can_get_i32(&data[1]); }
    if (vel)  { *vel = can_get_i16(&data[5]); }
    return true;
}

bool Can_DecodeDistances(const uint8_t *data, uint8_t dlc, uint16_t mm[4])
{
    if (data == NULL || mm == NULL || dlc < 8U) { return false; }
    for (uint8_t i = 0U; i < 4U; i++) { mm[i] = can_get_u16(&data[i * 2U]); }
    return true;
}

bool Can_DecodeImu(const uint8_t *data, uint8_t dlc, float *yawDeg, float dps[3])
{
    if (data == NULL || dlc < 8U) { return false; }
    if (yawDeg) { *yawDeg = (float)can_get_i16(&data[0]) / CAN_YAW_SCALE; }
    if (dps)
    {
        dps[0] = (float)can_get_i16(&data[2]) / CAN_DPS_SCALE;
        dps[1] = (float)can_get_i16(&data[4]) / CAN_DPS_SCALE;
        dps[2] = (float)can_get_i16(&data[6]) / CAN_DPS_SCALE;
    }
    return true;
}

bool Can_DecodePower(const uint8_t *data, uint8_t dlc, uint16_t *mv, uint16_t *ma, float *tempC)
{
    if (data == NULL || dlc < 6U) { return false; }
    if (mv)    { *mv = can_get_u16(&data[0]); }
    if (ma)    { *ma = can_get_u16(&data[2]); }
    if (tempC) { *tempC = (float)can_get_i16(&data[4]) / 10.0f; }
    return true;
}

} /* extern "C" */
