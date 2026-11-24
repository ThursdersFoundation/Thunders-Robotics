/**
 * @file    stepper_driver.h
 * @brief   2x A4988 steppers, trapezoid-free velocity profile (fixed accel).
 */
#ifndef ACT_NODE_STEPPER_H
#define ACT_NODE_STEPPER_H
#include <stdint.h>

class StepperDriver
{
public:
    void begin();
    void moveTo(uint8_t axis, int32_t targetSteps, int16_t speedStepsPerSec);
    /** Call every loop: emits steps when due. */
    void tick(uint32_t nowMs);
    int32_t position(uint8_t axis) const;
private:
    void stepPulse(uint8_t axis);
    int32_t m_pos[2];
    int32_t m_target[2];
    float   m_speed[2];        /* signed steps/s */
    uint32_t m_lastStepUs[2];
};
#endif /* ACT_NODE_STEPPER_H */
