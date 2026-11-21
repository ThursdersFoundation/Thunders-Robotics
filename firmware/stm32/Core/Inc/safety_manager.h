/**
 * @file    safety_manager.h
 * @brief   Central safety state machine: fault registry, task supervision,
 *          emergency stop and watchdog feeding.
 *
 * States:
 *   INIT      boot self-test
 *   IDLE      healthy, motors disarmed
 *   RUNNING   healthy, motors enabled
 *   SAFE_STOP latched fault, motors braked, must be cleared explicitly
 */
#ifndef THUNDERS_SAFETY_MANAGER_H
#define THUNDERS_SAFETY_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Fault codes (bitmask; also used on the wire and by the Arduino nodes) */
typedef enum
{
    FAULT_NONE             = 0x00000000UL,
    FAULT_TASK_TIMEOUT     = 0x00000001UL,
    FAULT_BATTERY_CRITICAL = 0x00000002UL,
    FAULT_OVERCURRENT      = 0x00000004UL,
    FAULT_TILT             = 0x00000008UL,
    FAULT_OBSTACLE         = 0x00000010UL,
    FAULT_COMM_TIMEOUT     = 0x00000020UL,
    FAULT_IMU_TIMEOUT      = 0x00000040UL,
    FAULT_LIDAR_ERROR      = 0x00000080UL,
    FAULT_CAN_TIMEOUT      = 0x00000100UL,
    FAULT_MOTOR_STALL      = 0x00000200UL,
    FAULT_ESTOP_BUTTON     = 0x00000400UL,
    FAULT_WATCHDOG         = 0x00000800UL
} FaultCode;

/* Supervised tasks */
typedef enum
{
    HEARTBEAT_ID_CONTROL = 0,
    HEARTBEAT_ID_SENSORS,
    HEARTBEAT_ID_COMM,
    HEARTBEAT_ID_LIDAR,
    HEARTBEAT_ID_COUNT
} HeartbeatId;

typedef enum
{
    SAFETY_STATE_INIT = 0,
    SAFETY_STATE_IDLE,
    SAFETY_STATE_RUNNING,
    SAFETY_STATE_SAFE_STOP
} SafetyState;

void SafetyManager_Init(void);
void SafetyManager_Heartbeat(HeartbeatId id);
void SafetyManager_Task(void);          /* 100 Hz supervision */

/* Inputs from other modules */
void SafetyManager_ReportBattery(uint16_t voltageMv, uint16_t currentMa);
void SafetyManager_ReportTilt(float rollDeg, float pitchDeg);
void SafetyManager_ReportObstacle(uint16_t minFrontMm);
void SafetyManager_ReportCommAlive(void);
void SafetyManager_ReportCanAlive(void);
void SafetyManager_EStopButtonPressed(void);

void SafetyManager_RaiseFault(uint32_t fault, uint16_t argument);
void SafetyManager_ClearFault(uint32_t fault);
void SafetyManager_ClearAllFaults(void);
uint32_t SafetyManager_GetFaults(void);
SafetyState SafetyManager_GetState(void);
bool SafetyManager_IsMotorOutputAllowed(void);

/* Called by TaskControl right after enabling motors */
void SafetyManager_OnMotorsEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_SAFETY_MANAGER_H */
