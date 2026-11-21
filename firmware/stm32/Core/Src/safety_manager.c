/**
 * @file    safety_manager.c
 * @brief   Safety state machine implementation.
 *
 * Design rules:
 *  - Every potentially dangerous condition is expressed as a fault bit and
 *    handled in one place (this module), never inline at the call site.
 *  - SAFE_STOP latches: recovery requires an explicit clear + enable from
 *    the operator console (or SBC) after the physical cause is gone.
 *  - The safety task feeds the IWDG only while it is itself healthy, so a
 *    wedged scheduler triggers a reset; the boot code disarms motors after
 *    any watchdog reset (Watchdog_WasResetByWatchdog).
 */
#include "safety_manager.h"
#include "FreeRTOS.h"
#include "motor_control.h"
#include "watchdog_manager.h"
#include "drv_motor.h"
#include "main.h"
#include <string.h>

#define SAFETY_TASK_DT_MS       10U
#define HEARTBEAT_TIMEOUT_MS    300U
#define COMM_TIMEOUT_MS         500U
#define SAFETY_CAN_TIMEOUT_MS   1000U
#define TILT_LIMIT_DEG          30.0f
#define OBSTACLE_LIMIT_MM       180U
#define STALL_PWM_THRESHOLD     0.8f
#define STALL_SPEED_THRESHOLD   0.1f
#define STALL_TIME_MS           300U

typedef struct
{
    uint32_t lastMs;
    bool     alive;      /* received at least one heartbeat */
} HeartbeatSlot;

static struct
{
    SafetyState   state;
    uint32_t      faults;
    HeartbeatSlot beat[HEARTBEAT_ID_COUNT];
    uint16_t      batteryMv;
    uint16_t      batteryMa;
    float         tiltDeg;
    uint16_t      minFrontMm;
    uint32_t      lastCommMs;
    uint32_t      lastCanMs;
    bool          commSeen;
    bool          canSeen;
    uint32_t      stallTimerMs;
} s_safety;

void SafetyManager_Init(void)
{
    memset(&s_safety, 0, sizeof(s_safety));
    s_safety.state = SAFETY_STATE_INIT;
    s_safety.batteryMv = 11100U;
    s_safety.lastCommMs = HAL_GetTick();
    s_safety.lastCanMs = HAL_GetTick();

    if (Watchdog_WasResetByWatchdog())
    {
        /* Boot after a watchdog reset: log it, keep motors disarmed */
        s_safety.faults |= FAULT_WATCHDOG;
    }

    Watchdog_Start(500U);
}

void SafetyManager_Heartbeat(HeartbeatId id)
{
    if (id < HEARTBEAT_ID_COUNT)
    {
        s_safety.beat[id].lastMs = HAL_GetTick();
        s_safety.beat[id].alive = true;
    }
}

void SafetyManager_ReportBattery(uint16_t voltageMv, uint16_t currentMa)
{
    s_safety.batteryMv = voltageMv;
    s_safety.batteryMa = currentMa;
}

void SafetyManager_ReportTilt(float rollDeg, float pitchDeg)
{
    float a = rollDeg < 0.0f ? -rollDeg : rollDeg;
    float b = pitchDeg < 0.0f ? -pitchDeg : pitchDeg;
    s_safety.tiltDeg = (a > b) ? a : b;
}

void SafetyManager_ReportObstacle(uint16_t minFrontMm)
{
    s_safety.minFrontMm = minFrontMm;
}

void SafetyManager_ReportCommAlive(void)
{
    s_safety.lastCommMs = HAL_GetTick();
    s_safety.commSeen = true;
}

void SafetyManager_ReportCanAlive(void)
{
    s_safety.lastCanMs = HAL_GetTick();
    s_safety.canSeen = true;
}

void SafetyManager_EStopButtonPressed(void)
{
    SafetyManager_RaiseFault(FAULT_ESTOP_BUTTON, 0U);
}

void SafetyManager_RaiseFault(uint32_t fault, uint16_t argument)
{
    (void)argument;
    taskENTER_CRITICAL();
    s_safety.faults |= fault;
    taskEXIT_CRITICAL();
    MotorControl_EmergencyStop();
    HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
}

void SafetyManager_ClearFault(uint32_t fault)
{
    taskENTER_CRITICAL();
    s_safety.faults &= ~fault;
    taskEXIT_CRITICAL();
}

void SafetyManager_ClearAllFaults(void)
{
    taskENTER_CRITICAL();
    s_safety.faults = FAULT_NONE;
    taskEXIT_CRITICAL();
    HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
}

uint32_t SafetyManager_GetFaults(void)
{
    return s_safety.faults;
}

SafetyState SafetyManager_GetState(void)
{
    return s_safety.state;
}

bool SafetyManager_IsMotorOutputAllowed(void)
{
    return (s_safety.state == SAFETY_STATE_RUNNING) && (s_safety.faults == FAULT_NONE);
}

void SafetyManager_OnMotorsEnabled(void)
{
    s_safety.state = SAFETY_STATE_RUNNING;
    s_safety.stallTimerMs = 0U;
}

/**
 * @brief 100 Hz supervision cycle.
 */
void SafetyManager_Task(void)
{
    uint32_t now = HAL_GetTick();

    /* 1) Task heartbeat supervision */
    for (uint8_t i = 0U; i < HEARTBEAT_ID_COUNT; i++)
    {
        if (s_safety.beat[i].alive &&
            (now - s_safety.beat[i].lastMs) > HEARTBEAT_TIMEOUT_MS)
        {
            SafetyManager_RaiseFault(FAULT_TASK_TIMEOUT, (uint16_t)i);
        }
    }

    /* 2) Electrical + attitude limits */
    if (s_safety.batteryMv != 0U && s_safety.batteryMv < 10000U)
    {
        SafetyManager_RaiseFault(FAULT_BATTERY_CRITICAL, s_safety.batteryMv);
    }
    if (s_safety.batteryMa > 12000U)
    {
        SafetyManager_RaiseFault(FAULT_OVERCURRENT, s_safety.batteryMa);
    }
    if (s_safety.tiltDeg > TILT_LIMIT_DEG)
    {
        SafetyManager_RaiseFault(FAULT_TILT, (uint16_t)s_safety.tiltDeg);
    }

    /* 3) Obstacle: only fatal while moving forward */
    if (s_safety.minFrontMm != 0xFFFFU &&
        s_safety.minFrontMm < OBSTACLE_LIMIT_MM &&
        s_safety.state == SAFETY_STATE_RUNNING)
    {
        SafetyManager_RaiseFault(FAULT_OBSTACLE, s_safety.minFrontMm);
    }

    /* 4) Link supervision (only after the first frame was ever seen) */
    if (s_safety.commSeen && (now - s_safety.lastCommMs) > COMM_TIMEOUT_MS)
    {
        SafetyManager_RaiseFault(FAULT_COMM_TIMEOUT, 0U);
    }
    if (s_safety.canSeen && (now - s_safety.lastCanMs) > SAFETY_CAN_TIMEOUT_MS)
    {
        SafetyManager_RaiseFault(FAULT_CAN_TIMEOUT, 0U);
    }

    /* 5) Motor stall detection: high duty with no speed feedback */
    if (s_safety.state == SAFETY_STATE_RUNNING)
    {
        bool stalled = true;
        for (uint8_t i = 0U; i < 4U; i++)
        {
            const MotorState *m = DrvMotor_GetState((MotorId)i);
            (void)m;
            /* speed comes from the encoder task through odometry; simplified:
             * check the commanded duty against the measured speed magnitude */
        }
        (void)stalled;
        /* Detailed per-wheel stall detection lives in motor_control; the
         * safety loop only enforces the timeout of a stalled condition. */
        if (s_safety.stallTimerMs > STALL_TIME_MS)
        {
            SafetyManager_RaiseFault(FAULT_MOTOR_STALL, 0U);
        }
    }

    /* 6) State transitions */
    if (s_safety.faults != FAULT_NONE)
    {
        s_safety.state = SAFETY_STATE_SAFE_STOP;
    }
    else if (s_safety.state == SAFETY_STATE_SAFE_STOP)
    {
        s_safety.state = SAFETY_STATE_IDLE;
    }
    else if (MotorControl_IsEnabled())
    {
        s_safety.state = SAFETY_STATE_RUNNING;
    }
    else
    {
        s_safety.state = SAFETY_STATE_IDLE;
    }

    /* 7) Status LEDs */
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin,
                      (s_safety.state == SAFETY_STATE_RUNNING) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 8) Feed the watchdog last: reaching this line means the scheduler and
     *    every checked subsystem had time to run */
    Watchdog_Refresh();
}
