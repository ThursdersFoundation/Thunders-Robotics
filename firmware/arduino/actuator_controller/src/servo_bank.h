/**
 * @file    servo_bank.h
 * @brief   6 servo channels with soft angle rate limiting.
 */
#ifndef ACT_NODE_SERVO_BANK_H
#define ACT_NODE_SERVO_BANK_H
#include <stdint.h>
#include <Servo.h>

class ServoBank
{
public:
    void begin();
    void setAngle(uint8_t idx, float angleDeg);
    void fillStatus(uint8_t out[8]);
private:
    Servo m_servo[6];
    float m_angle[6];
    static const uint8_t kPins[6];
};
#endif /* ACT_NODE_SERVO_BANK_H */
