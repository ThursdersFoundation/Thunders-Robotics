/**
 * @file    motor_driver.h
 * @brief   TB6612FNG dual H-bridge driver for 4 motors (2 carriers).
 *
 * Truth table (x = don't care):  IN1=1 IN2=0 CW | IN1=0 IN2=1 CCW
 *                               IN1=0 IN2=0 stop (short brake)
 *                               IN1=1 IN2=1 stop (brake) - unused
 */
#ifndef MOTOR_NODE_MOTOR_DRIVER_H
#define MOTOR_NODE_MOTOR_DRIVER_H

#include <stdint.h>

#define MOTOR_COUNT 4U

class MotorDriver
{
public:
    void begin();
    void setEnabled(bool on);
    bool enabled() const { return m_enabled; }
    /** duty -1.0..1.0 (negative = reverse), immediately applied. */
    void setDuty(uint8_t idx, float duty);
    void stopAll();
    const float *duty() const { return m_duty; }

private:
    void writeChannel(uint8_t idx, float duty);
    float m_duty[MOTOR_COUNT];
    bool  m_enabled;
};
#endif /* MOTOR_NODE_MOTOR_DRIVER_H */
