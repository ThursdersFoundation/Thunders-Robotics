/** @file can_interface.cpp : MCP2515 wiring + dispatch. */
#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>          /* autowp/arduino-mcp2515 */
#include "can_interface.h"
#include "command_handler.h"

static MCP_CAN s_mcpCan(10);          /* CS on D10 */
static const uint8_t kCanIntPin = 2U; /* INT on D2 (note: ENC3 reuses D2/D3;
                                       * on boards with 4 encoders move CAN INT
                                       * to D19 and ENC3 to D18/D16) */

void CanInterface::begin(CanNodeId nodeId)
{
    m_nodeId = nodeId;
    while (s_mcpCan.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK)
    {
        Serial.println(F("[can] init failed, retry"));
        delay(200);
    }
    s_mcpCan.setMode(MCP_NORMAL);
    pinMode(kCanIntPin, INPUT);
}

void CanInterface::poll(uint32_t nowMs)
{
    /* Drain everything pending in the RX buffers */
    while (!digitalRead(kCanIntPin))
    {
        uint32_t id;
        uint8_t len = 0U;
        uint8_t buf[8];
        if (s_mcpCan.readMsgBuf(&id, &len, buf) != CAN_OK) { break; }

        if (m_handler != NULL)
        {
            m_handler->onCanFrame(id, buf, len);
        }
    }
}

void CanInterface::sendHeartbeat(uint8_t state, uint16_t uptimeS, uint16_t faults)
{
    uint8_t payload[5];
    (void)Can_EncodeHeartbeat(payload, state, uptimeS, faults);
    s_mcpCan.sendMsgBuf((uint32_t)(CAN_ID_HEARTBEAT_BASE | m_nodeId), 0U, 5U, payload);
}

void CanInterface::sendMotorTelemetry(const volatile int32_t counts[4], const float duty[4],
                                      int16_t currentMa)
{
    uint8_t buf[8];
    can_put_i32(&buf[0], counts[0]);
    can_put_i32(&buf[4], counts[1]);
    s_mcpCan.sendMsgBuf(CAN_ID_MOTOR_ENCODERS0, 0U, 8U, buf);
    can_put_i32(&buf[0], counts[2]);
    can_put_i32(&buf[4], counts[3]);
    s_mcpCan.sendMsgBuf(CAN_ID_MOTOR_ENCODERS1, 0U, 8U, buf);

    for (uint8_t i = 0U; i < 4U; i++)
    {
        can_put_i16(&buf[i * 2U], proto_pack_i16(duty[i], CAN_VEL_SCALE));
    }
    s_mcpCan.sendMsgBuf(CAN_ID_MOTOR_PWM, 0U, 8U, buf);
    can_put_i16(&buf[0], currentMa);
    s_mcpCan.sendMsgBuf(CAN_ID_MOTOR_CURRENT, 0U, 2U, buf);
}

void CanInterface::sendFault(uint16_t code, uint16_t argument)
{
    uint8_t buf[4];
    can_put_u16(&buf[0], code);
    can_put_u16(&buf[2], argument);
    s_mcpCan.sendMsgBuf((uint32_t)(CAN_ID_FAULT_BASE | m_nodeId), 0U, 4U, buf);
}
