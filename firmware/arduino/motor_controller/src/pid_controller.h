/**
 * @file    pid_controller.h
 * @brief   Velocity PID for AVR: float math with clamped integrator and
 *          derivative on measurement (identical policy to the master).
 */
#ifndef MOTOR_NODE_PID_CONTROLLER_H
#define MOTOR_NODE_PID_CONTROLLER_H

#include <stdint.h>

class PidController
{
public:
    void configure(float kp, float ki, float kd, float integMax);
    void setOutputLimits(float minOut, float maxOut);
    /** measurement in rad/s, setpoint in rad/s, dt in seconds -> duty. */
    float update(float measurement, float setpoint, float dtSec);
    void reset();

private:
    float m_kp = 0.0f, m_ki = 0.0f, m_kd = 0.0f;
    float m_integ = 0.0f, m_integMax = 0.6f;
    float m_prevMeasurement = 0.0f;
    float m_outMin = -1.0f, m_outMax = 1.0f;
    bool  m_first = true;
};
#endif /* MOTOR_NODE_PID_CONTROLLER_H */
