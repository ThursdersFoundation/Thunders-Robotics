/**
 * @file    usonic_pair.h
 * @brief   Two HC-SR04 with ~20 ms staggered triggering (avoids crosstalk).
 */
#ifndef SENSOR_NODE_USONIC_PAIR_H
#define SENSOR_NODE_USONIC_PAIR_H
#include <stdint.h>

class UsonicPair
{
public:
    void begin(uint8_t trig0, uint8_t echo0, uint8_t trig1, uint8_t echo1);
    void triggerNext();
    uint16_t distanceMm(uint8_t idx) const;

private:
    void pulse(uint8_t idx);
    uint8_t m_trig[2];
    uint8_t m_echo[2];
    volatile uint16_t m_rise[2];
    volatile uint16_t m_dist[2];
    volatile bool m_active[2];
    uint8_t m_next = 0;
};
#endif /* SENSOR_NODE_USONIC_PAIR_H */
