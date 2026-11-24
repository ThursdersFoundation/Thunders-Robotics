/**
 * @file    actuator_can.h
 * @brief   CAN wrapper for the actuator node (commands + status).
 */
#ifndef ACT_NODE_CAN_H
#define ACT_NODE_CAN_H
#include <stdint.h>
#include "RobotCommon.h"

class ServoBank;
class StepperDriver;

class ActuatorCan
{
public:
    void begin(CanNodeId nodeId, ServoBank *servos, StepperDriver *steppers);
    void poll(uint32_t nowMs);
    void sendActStatus(const uint8_t status[8]);
    void sendHeartbeat(uint8_t state, uint16_t uptimeS, uint16_t faults);
private:
    ServoBank      *m_servos = 0;
    StepperDriver  *m_steppers = 0;
    CanNodeId m_nodeId = CAN_NODE_ACTUATOR;
};
#endif /* ACT_NODE_CAN_H */
