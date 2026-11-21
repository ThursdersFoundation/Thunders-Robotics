/**
 * @file    communication_manager.h
 * @brief   Communication task: SBC UART link, CAN node dispatch, lidar pump.
 *
 *   SBC  (UART4, 921.6k, DMA circular + protocol frames)
 *     RX: CMD_VELOCITY, CMD_ENABLE, CMD_SET_PID, GET_ODOM, GET_STATUS, PING
 *     TX: TLM_ODOM (50 Hz), TLM_STATUS (10 Hz), TLM_IMU (10 Hz), TLM_SCAN (5 Hz)
 *   CAN1 (500k): commands to nodes, telemetry/faults from nodes
 *   USART1: lidar byte pump (from DMA ring into the protocol parser)
 *   USART3: direct motor-node link (secondary, same frame protocol)
 */
#ifndef THUNDERS_COMMUNICATION_MANAGER_H
#define THUNDERS_COMMUNICATION_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float    cmdVx;
    float    cmdVy;
    float    cmdWz;
    bool     enabled;
    uint32_t lastCmdMs;
    uint32_t rxFrames;
    uint32_t txFrames;
} CommStatus;

void CommunicationManager_Init(void);
/** Frees the DMA rings into the parsers; call at COMM_TASK_HZ. */
void CommunicationManager_Task(void);
/** Handle one complete SBC frame (also used by the direct motor link). */
void CommunicationManager_HandleFrame(const uint8_t type, const uint8_t *payload, uint8_t len);
const CommStatus *CommunicationManager_GetStatus(void);

/* CAN callbacks (invoked from CAN1 IRQ in stm32f4xx_it.c) */
void CommunicationManager_OnCanFrame(uint32_t stdId, const uint8_t *data, uint8_t dlc);
/** Periodic CAN housekeeping: heartbeats, node timeout supervision. */
void CommunicationManager_CanTick(void);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_COMMUNICATION_MANAGER_H */
