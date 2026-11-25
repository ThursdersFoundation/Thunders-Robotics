/** @file motor_driver.cpp : TB6612 implementation (pins from docs/pinmaps.md). */
#include <Arduino.h>
#include "motor_driver.h"

/* Per-motor pin map: {PWM, IN1, IN2} */
static const uint8_t kPins[MOTOR_COUNT][3] = {
    { 5, 4, 7 },     /* FL: PWMA AIN1 AIN2 */
    { 6, 8, 12 },    /* FR: PWMB BIN1 BIN2 */
    { 46, 44, 45 },  /* RL: second carrier */
    { 9, 22, 23 },   /* RR: second carrier */
};
static const uint8_t kStandby = 24U;   /* both TB6612 STBY tied together */

void MotorDriver::begin()
{
    pinMode(kStandby, OUTPUT);
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        pinMode(kPins[i][0], OUTPUT);
        pinMode(kPins[i][1], OUTPUT);
        pinMode(kPins[i][2], OUTPUT);
        digitalWrite(kPins[i][1], LOW);
        digitalWrite(kPins[i][2], LOW);
        analogWrite(kPins[i][0], 0);
        m_duty[i] = 0.0f;
    }
    digitalWrite(kStandby, LOW);   /* coast while disabled */
    m_enabled = false;
}

void MotorDriver::setEnabled(bool on)
{
    m_enabled = on;
    digitalWrite(kStandby, on ? HIGH : LOW);
    if (!on)
    {
        for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
        {
            m_duty[i] = 0.0f;
            analogWrite(kPins[i][0], 0);
            digitalWrite(kPins[i][1], LOW);
            digitalWrite(kPins[i][2], LOW);
        }
    }
}

void MotorDriver::writeChannel(uint8_t idx, float duty)
{
    uint8_t pwm = (uint8_t)(duty < 0.0f ? -duty : duty * 255.0f);
    if (pwm > 255U) { pwm = 255U; }
    analogWrite(kPins[idx][0], pwm);
    if (duty > 0.0f)
    {
        digitalWrite(kPins[idx][1], HIGH);
        digitalWrite(kPins[idx][2], LOW);
    }
    else if (duty < 0.0f)
    {
        digitalWrite(kPins[idx][1], LOW);
        digitalWrite(kPins[idx][2], HIGH);
    }
    else
    {
        digitalWrite(kPins[idx][1], LOW);
        digitalWrite(kPins[idx][2], LOW);   /* short brake */
    }
}

void MotorDriver::setDuty(uint8_t idx, float duty)
{
    if (idx >= MOTOR_COUNT || !m_enabled) { return; }
    if (duty > 1.0f)  { duty = 1.0f; }
    if (duty < -1.0f) { duty = -1.0f; }
    m_duty[idx] = duty;
    writeChannel(idx, duty);
}

void MotorDriver::stopAll()
{
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        m_duty[i] = 0.0f;
        writeChannel(i, 0.0f);
    }
}
