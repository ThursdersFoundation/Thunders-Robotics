/** @file stepper_driver.cpp */
#include <Arduino.h>
#include "stepper_driver.h"

/* {STEP, DIR, ENABLE} pins per axis (A4988) */
static const uint8_t kStepPin[2] = { 26U, 27U };
static const uint8_t kDirPin[2]  = { 24U, 25U };
static const uint8_t kEnPin      = 28U;

void StepperDriver::begin()
{
    for (uint8_t i = 0U; i < 2U; i++)
    {
        pinMode(kStepPin[i], OUTPUT);
        pinMode(kDirPin[i], OUTPUT);
        m_pos[i] = 0;
        m_target[i] = 0;
        m_speed[i] = 0.0f;
        m_lastStepUs[i] = 0U;
    }
    pinMode(kEnPin, OUTPUT);
    digitalWrite(kEnPin, LOW);   /* enable both drivers */
}

void StepperDriver::moveTo(uint8_t axis, int32_t targetSteps, int16_t speedStepsPerSec)
{
    if (axis >= 2U) { return; }
    m_target[axis] = targetSteps;
    float s = (float)speedStepsPerSec;
    if (s > 4000.0f)  { s = 4000.0f; }
    if (s < -4000.0f) { s = -4000.0f; }
    /* direction follows the position error, magnitude from the command */
    float err = (float)(targetSteps - m_pos[axis]);
    m_speed[axis] = (err >= 0.0f ? s : -s);
}

void StepperDriver::stepPulse(uint8_t axis)
{
    digitalWrite(kStepPin[axis], HIGH);
    delayMicroseconds(2);
    digitalWrite(kStepPin[axis], LOW);
}

void StepperDriver::tick(uint32_t nowMs)
{
    for (uint8_t i = 0U; i < 2U; i++)
    {
        int32_t err = m_target[i] - m_pos[i];
        if (err == 0 || m_speed[i] == 0.0f) { continue; }

        digitalWrite(kDirPin[i], (m_speed[i] > 0.0f) ? HIGH : LOW);
        uint32_t interval = (uint32_t)(1000000UL / (m_speed[i] < 0.0f ? -m_speed[i] : m_speed[i]));
        uint32_t nowUs = micros();
        if ((nowUs - m_lastStepUs[i]) >= interval)
        {
            stepPulse(i);
            m_lastStepUs[i] = nowUs;
            m_pos[i] += (m_speed[i] > 0.0f) ? 1 : -1;
        }
        (void)err;
    }
}

int32_t StepperDriver::position(uint8_t axis) const
{
    return (axis < 2U) ? m_pos[axis] : 0;
}
