/** @file usb_bridge.cpp */
#include <Arduino.h>
#include "usb_bridge.h"
#include "can_interface.h"

/* ProtoCanFrame payload for CAN bridge messages: id u16, dlc u8, data 8 B */
void UsbBridge::begin(CanInterface *can)
{
    m_can = can;
    PacketParser_Init(&m_parser);
}

void UsbBridge::sendFrame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[PROTOCOL_MAX_FRAME];
    size_t n = Packet_Build(frame, sizeof(frame), type, payload, len);
    if (n > 0U)
    {
        Serial1.write(frame, (size_t)n);
        m_txCount++;
    }
}

void UsbBridge::onCanFrame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    uint8_t payload[11];
    can_put_u16(&payload[0], (uint16_t)id);
    payload[2] = dlc;
    memcpy(&payload[3], data, 8U);
    sendFrame(MSG_CAN_BRIDGE_RX, payload, 11U);
}

void UsbBridge::onByte(uint8_t byte)
{
    PacketFrame frame;
    if (PacketParser_Feed(&m_parser, byte, &frame))
    {
        switch (frame.type)
        {
        case MSG_CAN_BRIDGE_TX:
        {
            if (frame.length < 3U) { break; }
            uint16_t id = can_get_u16(&frame.payload[0]);
            uint8_t dlc = frame.payload[2];
            if (dlc > 8U) { break; }
            m_can->send(id, &frame.payload[3], dlc);
            break;
        }
        case MSG_PING:
        {
            uint8_t pong = 1U;
            sendFrame(MSG_ACK, &pong, 1U);
            break;
        }
        default:
        {
            uint8_t code = frame.type;
            sendFrame(MSG_NACK, &code, 1U);
            break;
        }
        }
    }
}

void UsbBridge::tick(uint32_t nowMs)
{
    if ((nowMs - m_lastBeatMs) >= 100U)
    {
        m_lastBeatMs = nowMs;
        m_can->sendHeartbeat(1U, (uint16_t)(nowMs / 1000U), 0U);
    }
}
