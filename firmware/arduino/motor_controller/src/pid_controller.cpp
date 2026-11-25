/** @file pid_controller.cpp */
#include "pid_controller.h"

void PidController::configure(float kp, float ki, float kd, float integMax)
{
    m_kp = kp; m_ki = ki; m_kd = kd; m_integMax = integMax;
}

void PidController::setOutputLimits(float minOut, float maxOut)
{
    m_outMin = minOut; m_outMax = maxOut;
}

void PidController::reset()
{
    m_integ = 0.0f;
    m_first = true;
}

float PidController::update(float measurement, float setpoint, float dtSec)
{
    if (dtSec <= 0.0f) { return 0.0f; }

    float error = setpoint - measurement;
    float deriv = m_first ? 0.0f : -(measurement - m_prevMeasurement) / dtSec;
    m_first = false;
    m_prevMeasurement = measurement;

    m_integ += error * dtSec;
    if (m_integ > m_integMax)  { m_integ = m_integMax; }
    if (m_integ < -m_integMax) { m_integ = -m_integMax; }

    float out = m_kp * error + m_ki * m_integ + m_kd * deriv;
    if (out > m_outMax) { out = m_outMax; }
    if (out < m_outMin) { out = m_outMin; }
    return out;
}
