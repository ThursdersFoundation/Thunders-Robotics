/**
 * @file    robot_config.h
 * @brief   Motor node configuration constants (mirrors the master geometry).
 */
#ifndef MOTOR_NODE_CONFIG_H
#define MOTOR_NODE_CONFIG_H

#include <stdint.h>

namespace RobotConfig
{
/* Mechanical (must match firmware/stm32 Core/Inc/main.h) */
const float WHEEL_RADIUS_M = 0.040f;    /* 80 mm omni wheel            */
const float GEAR_RATIO     = 30.0f;
const float ENCODER_PPR    = 11.0f;
/* 11 PPR * 4 (quadrature) * 30 = 1320 counts per wheel revolution */
const int32_t COUNTS_PER_REV = 1320;

/* Velocity PID (wheel shaft rad/s -> duty -1..1) */
const float PID_KP         = 0.28f;
const float PID_KI         = 3.2f;
const float PID_KD         = 0.0f;
const float PID_INTEG_MAX  = 0.6f;
const uint32_t PID_PERIOD_MS = 10U;     /* 100 Hz */
const float PID_PERIOD_S   = 0.010f;

/* Safety */
const uint32_t CMD_TIMEOUT_MS = 500U;   /* no command -> motors stop */
const int16_t MAX_CURRENT_MA  = 3000;   /* per-motor stall threshold */

/* Mounting sign per motor (matches master MOTOR_SIGN[]) */
const float MOTOR_SIGN[4] = { 1.0f, -1.0f, -1.0f, 1.0f };

inline int16_t mockCurrentMa(void) { return 0; }  /* wired to ACS712 later */

}  /* namespace RobotConfig */
#endif /* MOTOR_NODE_CONFIG_H */
