/** @file encoder_reader.cpp : quadrature decoding for the motor node. */
#include <Arduino.h>
#include "encoder_reader.h"

/* 11 PPR * 4 quadrature * 30:1 gearbox */
static const int32_t kCountsPerRev = 1320;

/* Pin pairs per encoder: {A, B} */
static const uint8_t kEncPins[MOTOR_COUNT][2] = {
    { 18, 19 },   /* ENC1: INT5/INT4 */
    { 20, 21 },   /* ENC2: INT3/INT2 */
    { 2, 3 },     /* ENC3: INT0/INT1 (shared with CAN INT: see config note) */
    { 14, 15 },   /* ENC4: polled fallback */
};

static EncoderReader *s_activeReader = NULL;

EncoderReader readerInstance;   /* single instance wired to the ISRs */

/* Quadrature state lookup: (prevA<<3)|(prevB<<2)|(newA<<1)|newB */
static const int8_t kQuadrature[16] = { 0, -1, +1, 0,  +1, 0, 0, -1,  -1, 0, 0, +1,  0, +1, -1, 0 };

void EncoderReader::begin()
{
    s_activeReader = this;
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        pinMode(kEncPins[i][0], INPUT_PULLUP);
        pinMode(kEncPins[i][1], INPUT_PULLUP);
        m_counts[i] = 0;
        m_lastSnapshot[i] = 0;
        m_radPerSec[i] = 0.0f;
    }
    attachInterrupt(digitalPinToInterrupt(kEncPins[0][0]), EncoderReader::isr0A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kEncPins[0][1]), EncoderReader::isr0B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kEncPins[1][0]), EncoderReader::isr1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kEncPins[1][1]), EncoderReader::isr1B, CHANGE);
}

void EncoderReader::onEdge0A() { m_counts[0] += (digitalRead(kEncPins[0][0]) == digitalRead(kEncPins[0][1])) ? 1 : -1; }
void EncoderReader::onEdge0B() { m_counts[0] += (digitalRead(kEncPins[0][0]) != digitalRead(kEncPins[0][1])) ? 1 : -1; }
void EncoderReader::onEdge1A() { m_counts[1] += (digitalRead(kEncPins[1][0]) == digitalRead(kEncPins[1][1])) ? 1 : -1; }
void EncoderReader::onEdge1B() { m_counts[1] += (digitalRead(kEncPins[1][0]) != digitalRead(kEncPins[1][1])) ? 1 : -1; }

void EncoderReader::isr0A() { if (s_activeReader) s_activeReader->onEdge0A(); }
void EncoderReader::isr0B() { if (s_activeReader) s_activeReader->onEdge0B(); }
void EncoderReader::isr1A() { if (s_activeReader) s_activeReader->onEdge1A(); }
void EncoderReader::isr1B() { if (s_activeReader) s_activeReader->onEdge1B(); }

void EncoderReader::updateSpeeds(float dtSec)
{
    if (dtSec <= 0.0f) { return; }
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        int32_t now = m_counts[i];
        int32_t delta = now - m_lastSnapshot[i];
        m_lastSnapshot[i] = now;
        /* rev/s -> rad/s */
        m_radPerSec[i] = ((float)delta / (float)kCountsPerRev) * 6.2831853f / dtSec;
    }
}

int32_t EncoderReader::totalCounts(uint8_t idx) const
{
    return (idx < MOTOR_COUNT) ? m_counts[idx] : 0;
}

float EncoderReader::wheelRadPerSec(uint8_t idx) const
{
    return (idx < MOTOR_COUNT) ? m_radPerSec[idx] : 0.0f;
}

void EncoderReader::resetOdometer()
{
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        m_counts[i] = 0;
        m_lastSnapshot[i] = 0;
    }
}
