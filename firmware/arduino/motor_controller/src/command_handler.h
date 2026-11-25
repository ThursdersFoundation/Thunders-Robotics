/**
 * @file    command_handler.h
 * @brief   CAN command dispatch + supervision for the motor node.
 */
#ifndef MOTOR_NODE_COMMAND_HANDLER_H
#define MOTOR_NODE_COMMAND_HANDLER_H

#include <stdint.h>
#include "RobotCommon.h"

class MotorDriver;
class EncoderReader;
class CanInterface;

class CommandHandler
{
public:
    void begin(MotorDriver *motors, EncoderReader *encoders, CanInterface *can);
    void onCanFrame(uint32_t id, const uint8_t *data, uint8_t dlc);
    void getWheelTargets(float out[4]) const;
    void supervise(uint32_t nowMs);
    uint8_t stateFlags() const;      /* bit0 = enabled, bit1 = fresh command */
    uint16_t faultBits() const { return m_faults; }

private:
    MotorDriver    *m_motors = 0;
    EncoderReader  *m_encoders = 0;
    CanInterface   *m_can = 0;
    float    m_targets[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    uint32_t m_lastCmdMs = 0;
    bool     m_enabled = false;
    bool     m_cmdFresh = false;
    uint16_t m_faults = 0U;
};
#endif /* MOTOR_NODE_COMMAND_HANDLER_H */
