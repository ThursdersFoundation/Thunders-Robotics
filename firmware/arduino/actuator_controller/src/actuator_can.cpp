/** @file actuator_can.cpp */
#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include "actuator_can.h"
#include "servo_bank.h"
#include "stepper_driver.h"

static MCP_CAN s_mcpCan(10);
static const uint8_t kCanIntPin = 2U;

void ActuatorCan::begin(CanNodeId nodeId, ServoBank *servos, StepperDriver *steppers)
{
    m_nodeId = nodeId;
    m_servos = servos;
    m_steppers = steppers;
    while (s_mcpCan.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) { delay(200); }
    s_mcpCan.setMode(MCP_NORMAL);
    pinMode(kCanIntPin, INPUT);
}

void ActuatorCan::poll(uint32_t nowMs)
{
    (void)nowMs;
    uint32_t id;
    uint8_t len = 0U;
    uint8_t buf[8];
    while (!digitalRead(kCanIntPin))
    {
        if (s_mcpCan.readMsgBuf(&id, &len, buf) != CAN_OK) { break; }

        if (id == CAN_ID_ACT_SERVO)
        {
            uint8_t servoId;
            float angle;
            if (Can_DecodeServo(buf, len, &servoId, &angle) && m_servos != NULL)
            {
                m_servos->setAngle(servoId, angle);
            }
        }
        else if (id == CAN_ID_ACT_STEPPER)
        {
            uint8_t axis;
            int32_t pos;
            int16_t vel;
            if (Can_DecodeStepper(buf, len, &axis, &pos, &vel) && m_steppers != NULL)
            {
                m_steppers->moveTo(axis, pos, vel);
            }
        }
    }
}

void ActuatorCan::sendActStatus(const uint8_t status[8])
{
    s_mcpCan.sendMsgBuf(CAN_ID_ACT_STATUS, 0U, 8U, (uint8_t *)status);
}

void ActuatorCan::sendHeartbeat(uint8_t state, uint16_t uptimeS, uint16_t faults)
{
    uint8_t payload[5];
    (void)Can_EncodeHeartbeat(payload, state, uptimeS, faults);
    s_mcpCan.sendMsgBuf((uint32_t)(CAN_ID_HEARTBEAT_BASE | m_nodeId), 0U, 5U, payload);
}
