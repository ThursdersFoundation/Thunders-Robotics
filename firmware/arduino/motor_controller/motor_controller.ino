/**
 * @file    motor_controller.ino
 * @brief   Thunders Robotic motor node (Arduino Mega 2560, CAN node 1).
 *
 * Responsibilities:
 *  - Receive wheel velocity commands from the master via CAN
 *    (CAN_ID_MOTOR_VELOCITY, 4x int16 m/s x1e-3).
 *  - Run a per-wheel velocity PID at 100 Hz (encoder feedback).
 *  - Drive the two TB6612 channels (4 motors on 2 dual drivers).
 *  - Publish encoder telemetry (CAN_ID_MOTOR_ENCODERS0/1, PWM, current)
 *    and a 100 ms heartbeat.
 *
 * Timing budget (16 MHz AVR): PID 100 Hz, CAN pump 1 kHz poll, loop tick
 * 1 ms via millis() rounding - well within the Mega's capabilities.
 */
#include <Arduino.h>
#include "RobotCommon.h"
#include "src/robot_config.h"
#include "src/motor_driver.h"
#include "src/encoder_reader.h"
#include "src/pid_controller.h"
#include "src/can_interface.h"
#include "src/command_handler.h"

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */
static MotorDriver  s_motors;
static EncoderReader s_encoders;
static CanInterface s_can;
static CommandHandler s_commands;
static PidController s_pid[MOTOR_COUNT];

static uint32_t s_lastPidMs = 0U;
static uint32_t s_lastBeatMs = 0U;
static uint32_t s_lastTelemMs = 0U;
static uint32_t s_bootMs = 0U;

/* ------------------------------------------------------------------ */
void setup()
{
    Serial.begin(115200UL);
    Serial.println(F("[motor] boot"));

    s_motors.begin();
    s_encoders.begin();
    s_can.begin(CAN_NODE_MOTOR);
    s_commands.begin(&s_motors, &s_encoders, &s_can);

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        s_pid[i].configure(RobotConfig::PID_KP, RobotConfig::PID_KI,
                           RobotConfig::PID_KD, RobotConfig::PID_INTEG_MAX);
    }

    /* Keep motors disabled until the master sends CMD_ENABLE */
    s_motors.setEnabled(false);
    s_bootMs = millis();
}

/* ------------------------------------------------------------------ */
void loop()
{
    uint32_t now = millis();

    /* 1. CAN pump at full loop rate (RX + TX queue drain) */
    s_can.poll(now);

    /* 2. PID at 100 Hz */
    if ((now - s_lastPidMs) >= RobotConfig::PID_PERIOD_MS)
    {
        s_lastPidMs = now;

        float target[MOTOR_COUNT];
        s_commands.getWheelTargets(target);

        for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
        {
            float measured = s_encoders.wheelRadPerSec(i);
            float duty = s_pid[i].update(measured, target[i] / RobotConfig::WHEEL_RADIUS_M,
                                         RobotConfig::PID_PERIOD_S);
            if (!s_motors.enabled())
            {
                duty = 0.0f;
            }
            s_motors.setDuty(i, duty);
        }
        s_encoders.updateSpeeds(RobotConfig::PID_PERIOD_S);
    }

    /* 3. Telemetry at 50 Hz */
    if ((now - s_lastTelemMs) >= 20U)
    {
        s_lastTelemMs = now;
        int32_t counts[4];
        for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
        {
            counts[i] = s_encoders.totalCounts(i);
        }
        s_can.sendMotorTelemetry((const volatile int32_t *)counts, s_motors.duty(),
                                 RobotConfig::mockCurrentMa());
    }

    /* 4. Heartbeat at 100 ms */
    if ((now - s_lastBeatMs) >= 100U)
    {
        s_lastBeatMs = now;
        s_can.sendHeartbeat(s_commands.stateFlags(), (uint16_t)((now - s_bootMs) / 1000U),
                            (uint16_t)s_commands.faultBits());
        digitalWrite(LED_BUILTIN, (now / 500U) & 1U);
    }

    /* 5. Master link supervision: no velocity command for 500 ms -> stop */
    s_commands.supervise(now);
}
