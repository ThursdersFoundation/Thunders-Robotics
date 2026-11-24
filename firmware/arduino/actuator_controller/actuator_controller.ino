/**
 * @file    actuator_controller.ino
 * @brief   Thunders Robotic actuator node (CAN node 3): 6 servos + 2 steppers.
 */
#include <Servo.h>
#include "RobotCommon.h"
#include "src/actuator_can.h"
#include "src/servo_bank.h"
#include "src/stepper_driver.h"

static ActuatorCan s_can;
static ServoBank   s_servos;
static StepperDriver s_steppers;

static uint32_t s_lastBeatMs = 0;
static uint32_t s_bootMs = 0;

void setup()
{
    Serial.begin(115200UL);
    s_servos.begin();
    s_steppers.begin();
    s_can.begin(CAN_NODE_ACTUATOR, &s_servos, &s_steppers);
    s_bootMs = millis();
}

void loop()
{
    uint32_t now = millis();
    s_can.poll(now);
    s_steppers.tick(now);

    if ((now - s_lastBeatMs) >= 100U)
    {
        s_lastBeatMs = now;
        uint8_t status[8];
        s_servos.fillStatus(status);
        s_can.sendActStatus(status);
        s_can.sendHeartbeat(1U, (uint16_t)((now - s_bootMs) / 1000U), 0U);
    }
}
