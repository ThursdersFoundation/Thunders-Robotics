/**
 * @file    encoder_reader.h
 * @brief   Quadrature encoder reader on 4 interrupt pins (D18-D21).
 *
 * 2 of the 4 wheels use full 4x decoding on their dedicated interrupt
 * pins; the Mega has 6 external interrupts so each channel (A+B) of the
 * first two encoders gets one, and encoders 3/4 use pin-change polling of
 * their A channel at 1 kHz (B derived by lookup) - see .cpp comments.
 */
#ifndef MOTOR_NODE_ENCODER_READER_H
#define MOTOR_NODE_ENCODER_READER_H

#include <stdint.h>

#define MOTOR_COUNT 4U

class EncoderReader
{
public:
    void begin();
    /** Recompute radPerSec from the raw count delta. */
    void updateSpeeds(float dtSec);
    int32_t totalCounts(uint8_t idx) const;
    float wheelRadPerSec(uint8_t idx) const;
    void resetOdometer();
    /** ISR hooks (wired in begin via attachInterrupt). */
    void onEdge0A();
    void onEdge0B();
    void onEdge1A();
    void onEdge1B();

private:
    static void isr0A();
    static void isr0B();
    static void isr1A();
    static void isr1B();
    volatile int32_t m_counts[MOTOR_COUNT];
    int32_t m_lastSnapshot[MOTOR_COUNT];
    float m_radPerSec[MOTOR_COUNT];
};
#endif /* MOTOR_NODE_ENCODER_READER_H */
