/** @file usonic_pair.cpp */
#include <Arduino.h>
#include "usonic_pair.h"

static UsonicPair *s_active = NULL;

/* Echo pins on D23/D25 = INT6/PE6... Mega external int pins are limited,
 * so measure the echo width by polling the pin with micros() (block max
 * 25 ms, only for the sensor that was just triggered). */
static const uint32_t kEchoTimeoutUs = 25000UL;

void UsonicPair::begin(uint8_t trig0, uint8_t echo0, uint8_t trig1, uint8_t echo1)
{
    m_trig[0] = trig0; m_echo[0] = echo0;
    m_trig[1] = trig1; m_echo[1] = echo1;
    s_active = this;
    for (uint8_t i = 0; i < 2; i++)
    {
        pinMode(m_trig[i], OUTPUT);
        pinMode(m_echo[i], INPUT);
        digitalWrite(m_trig[i], LOW);
        m_dist[i] = 0xFFFFU;
        m_active[i] = false;
    }
}

void UsonicPair::pulse(uint8_t idx)
{
    /* 10 us trigger pulse */
    digitalWrite(m_trig[idx], HIGH);
    delayMicroseconds(10);
    digitalWrite(m_trig[idx], LOW);

    /* wait for echo rise (max 5 ms after trigger) */
    uint32_t start = micros();
    while (!digitalRead(m_echo[idx]))
    {
        if ((micros() - start) > 5000UL) { return; }
    }
    uint32_t rise = micros();

    /* wait for echo fall */
    while (digitalRead(m_echo[idx]))
    {
        if ((micros() - rise) > kEchoTimeoutUs) { m_active[idx] = false; return; }
    }
    uint32_t width = micros() - rise;

    if (width >= 116UL && width <= 25000UL)
    {
        m_dist[idx] = (uint16_t)((width * 343UL) / 2000UL);
        m_active[idx] = false;
    }
}

void UsonicPair::triggerNext()
{
    pulse(m_next);
    m_next = (uint8_t)((m_next + 1U) & 1U);
}

uint16_t UsonicPair::distanceMm(uint8_t idx) const
{
    return (idx < 2U) ? m_dist[idx] : 0xFFFFU;
}
