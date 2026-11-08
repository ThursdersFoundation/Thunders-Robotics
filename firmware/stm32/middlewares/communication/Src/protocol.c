/**
 * @file protocol.c
 * @brief Top-level protocol dispatcher.  Receives inbound commands from the
 *        comm manager, decodes them by MsgType_t, and forwards to the motor
 *        controller / safety manager / sensor manager.  Also packs the
 *        telemetry / health frames for outbound delivery.
 */
#include "protocol.h"

#include <string.h>
#include <stdio.h>

#include "motor_control.h"
#include "safety_manager.h"
#include "sensor_manager.h"
#include "main.h"

/* -------------------------------------------------------------------------
 * Private state.
 * ------------------------------------------------------------------------- */
static RobotState_t s_state;
static uint8_t      s_inited = 0;

HAL_StatusTypeDef Protocol_Init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_inited = 1;
    return HAL_OK;
}

static void pack_telemetry(const RobotState_t *state, uint8_t *buf)
{
    /* Big-endian field order - matches the Python host-side reader. */
    uint16_t i = 0;
    memcpy(&buf[i], &state->lin_mps, sizeof(float)); i += 4;
    memcpy(&buf[i], &state->ang_rps, sizeof(float)); i += 4;
    memcpy(&buf[i], &state->rpm_left,  sizeof(float)); i += 4;
    memcpy(&buf[i], &state->rpm_right, sizeof(float)); i += 4;
    memcpy(&buf[i], &state->pwm_left,  sizeof(int16_t)); i += 2;
    memcpy(&buf[i], &state->pwm_right, sizeof(int16_t)); i += 2;
    memcpy(&buf[i], &state->imu_yaw,   sizeof(float)); i += 4;
    memcpy(&buf[i], &state->imu_pitch, sizeof(float)); i += 4;
    memcpy(&buf[i], &state->imu_roll,  sizeof(float)); i += 4;
    memcpy(&buf[i], &state->battery_v, sizeof(float)); i += 4;
    buf[i++] = state->battery_pct;
    buf[i++] = (uint8_t)(state->faults & 0xFFU);
    buf[i++] = (uint8_t)((state->faults >> 8) & 0xFFU);
    uint32_t up = state->uptime_ms;
    memcpy(&buf[i], &up, sizeof(uint32_t)); i += 4;
}

HAL_StatusTypeDef Protocol_SendTelemetry(const RobotState_t *state)
{
    if (state == NULL) {
        return HAL_ERROR;
    }
    uint8_t buf[sizeof(RobotState_t) + 4];
    pack_telemetry(state, buf);
    /* Comm_Send is the manager entry point - it picks the active transport. */
    extern HAL_StatusTypeDef Comm_Send(uint8_t, const uint8_t *, uint16_t);
    return Comm_Send((uint8_t)MSG_TYPE_TELEMETRY, buf, (uint16_t)sizeof(RobotState_t));
}

void Protocol_BuildPong(uint8_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, 16);
    /* 4 bytes uptime + 8 bytes firmware id + 4 bytes magic. */
    uint32_t uptime = HAL_GetTick();
    memcpy(&out[0], &uptime, 4);
    const char *id = Main_GetBoardID();
    size_t n = strlen(id);
    if (n > 8) n = 8;
    memcpy(&out[4], id, n);
    out[12] = 'T';
    out[13] = 'H';
    out[14] = 'U';
    out[15] = 'N';
}

HAL_StatusTypeDef Protocol_HandleCommand(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0U) {
        return HAL_ERROR;
    }
    uint8_t type = buf[0];
    const uint8_t *payload = buf + 1;
    uint16_t plen = (uint16_t)(len - 1U);

    switch (type) {
    case MSG_TYPE_PING: {
        uint8_t pong[16];
        Protocol_BuildPong(pong);
        extern HAL_StatusTypeDef Comm_Send(uint8_t, const uint8_t *, uint16_t);
        Comm_Send((uint8_t)MSG_TYPE_PONG, pong, 16);
        return HAL_OK;
    }
    case MSG_TYPE_CMD_VEL: {
        if (plen != 8U) {
            return HAL_ERROR;
        }
        float lin, ang;
        memcpy(&lin, &payload[0], 4);
        memcpy(&ang, &payload[4], 4);
        return MotorControl_SetVelocity(lin, ang);
    }
    case MSG_TYPE_SET_MODE: {
        if (plen != 1U) {
            return HAL_ERROR;
        }
        MotorControl_SetMode((MotorMode_t)payload[0]);
        return HAL_OK;
    }
    case MSG_TYPE_ESTOP: {
        Safety_SetFault(SAFETY_FAULT_ESTOP);
        return HAL_OK;
    }
    default:
        return HAL_ERROR;
    }
}

/**
 * @brief Helper called by the comm manager to assemble a fresh
 *        RobotState_t from the live motor / sensor / safety subsystems.
 *        Exposed via the protocol.h API surface for the telemetry task.
 */
HAL_StatusTypeDef Protocol_SampleState(void)
{
    if (!s_inited) {
        return HAL_ERROR;
    }
    MotorState_t m;
    MotorControl_GetState(&m);
    const SensorSnapshot_t *snap = SensorManager_GetSnapshot();

    s_state.lin_mps      = m.command_lin_mps;
    s_state.ang_rps      = m.command_ang_rps;
    s_state.rpm_left     = m.measured_rpm_left;
    s_state.rpm_right    = m.measured_rpm_right;
    s_state.pwm_left     = m.pwm_left;
    s_state.pwm_right    = m.pwm_right;
    s_state.imu_yaw      = snap->imu.yaw;
    s_state.imu_pitch    = snap->imu.pitch;
    s_state.imu_roll     = snap->imu.roll;
    s_state.battery_v    = snap->battery_volts;
    s_state.battery_pct  = snap->battery_pct;
    s_state.faults       = Safety_GetFaults();
    s_state.uptime_ms    = HAL_GetTick();
    return HAL_OK;
}

HAL_StatusTypeDef Protocol_GetState(RobotState_t *out)
{
    if (out == NULL) {
        return HAL_ERROR;
    }
    *out = s_state;
    return HAL_OK;
}
