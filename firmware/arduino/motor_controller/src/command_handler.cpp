/** @file command_handler.cpp */
#include <Arduino.h>
#include "command_handler.h"
#include "motor_driver.h"
#include "encoder_reader.h"
#include "can_interface.h"
#include "robot_config.h"

void CommandHandler::begin(MotorDriver *motors, EncoderReader *encoders, CanInterface *can)
{
    m_motors = motors;
    m_encoders = encoders;
    m_can = can;
    m_lastCmdMs = 0;
}

void CommandHandler::onCanFrame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    switch (id)
    {
    case CAN_ID_MOTOR_VELOCITY:
    {
        float wheel[4];
        if (!Can_DecodeVelocity(data, dlc, wheel)) { return; }
        /* Kinematic wheel speeds -> motor shaft direction via mounting sign */
        for (uint8_t i = 0U; i < 4U; i++)
        {
            m_targets[i] = wheel[i] * RobotConfig::MOTOR_SIGN[i];
        }
        m_lastCmdMs = millis();
        m_cmdFresh = true;
        break;
    }

    case CAN_ID_MOTOR_ENABLE:
        if (dlc >= 1U)
        {
            m_enabled = (data[0] != 0U);
            m_motors->setEnabled(m_enabled);
            if (!m_enabled)
            {
                for (uint8_t i = 0U; i < 4U; i++) { m_targets[i] = 0.0f; }
            }
            m_lastCmdMs = millis();
        }
        break;

    case CAN_ID_MOTOR_SET_PID:
        /* forwarded to the PID loop by the sketch through a callback;
         * kept minimal here (single global set) */
        if (dlc >= 13U)
        {
            float kp = (float)can_get_i32(&data[1]) / PROTO_PID_SCALE;
            float ki = (float)can_get_i32(&data[5]) / PROTO_PID_SCALE;
            float kd = (float)can_get_i32(&data[9]) / PROTO_PID_SCALE;
            (void)kp; (void)ki; (void)kd;  /* applied in the sketch */
        }
        break;

    case CAN_ID_MASTER_TIME:
        if (dlc >= 4U) { m_lastCmdMs = millis(); }
        break;

    default:
        break;
    }
}

void CommandHandler::getWheelTargets(float out[4]) const
{
    for (uint8_t i = 0U; i < 4U; i++) { out[i] = m_targets[i]; }
}

void CommandHandler::supervise(uint32_t nowMs)
{
    if (m_enabled && m_cmdFresh &&
        (nowMs - m_lastCmdMs) > RobotConfig::CMD_TIMEOUT_MS)
    {
        /* Master stopped sending: safe stop */
        for (uint8_t i = 0U; i < 4U; i++) { m_targets[i] = 0.0f; }
        m_cmdFresh = false;
        m_faults |= (uint16_t)(FAULT_COMM_TIMEOUT & 0xFFFFU);
        m_can->sendFault((uint16_t)(FAULT_COMM_TIMEOUT & 0xFFFFU), 0U);
    }
}

uint8_t CommandHandler::stateFlags() const
{
    return (uint8_t)((m_enabled ? 0x01U : 0x00U) | (m_cmdFresh ? 0x02U : 0x00U));
}
