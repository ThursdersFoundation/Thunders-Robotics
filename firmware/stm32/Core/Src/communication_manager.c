/**
 * @file    communication_manager.c
 * @brief   Communication task implementation (see header for the link plan).
 */
#include "communication_manager.h"
#include "uart_protocol.h"
#include "can_protocol.h"
#include "protocol.h"
#include "packet_parser.h"
#include "sensor_manager.h"
#include "motor_control.h"
#include "safety_manager.h"
#include "encoder.h"
#include "drv_lidar.h"
#include "usart.h"
#include "can.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

#define ODOM_PERIOD_MS    20U    /* 50 Hz */
#define STATUS_PERIOD_MS  100U   /* 10 Hz */
#define SCAN_PERIOD_MS    200U   /* 5 Hz  */
#define IMU_PERIOD_MS     100U

/* DMA ring buffers */
static uint8_t s_sbcRxDma[256];
static uint8_t s_lidarRxDma[512];

static UartProtocol s_sbcLink;
static CommStatus   s_comm;
static uint32_t     s_lastOdomMs, s_lastStatusMs, s_lastScanMs, s_lastImuMs;

/* Node heartbeat freshness */
static struct { uint32_t lastMs; bool seen; } s_nodeBeat[8];

/* ------------------------------------------------------------------ */
/* TX helpers                                                          */
/* ------------------------------------------------------------------ */
static void Sbc_Send(const uint8_t *data, size_t len)
{
    (void)HAL_UART_Transmit(&huart4, data, (uint16_t)len, 10U);
    s_comm.txFrames++;
}

/* ------------------------------------------------------------------ */
/* SBC frame dispatcher                                                */
/* ------------------------------------------------------------------ */
static void Sbc_OnFrame(const PacketFrame *frame)
{
    s_comm.rxFrames++;
    SafetyManager_ReportCommAlive();
    HAL_GPIO_TogglePin(LED_COMM_GPIO_Port, LED_COMM_Pin);
    CommunicationManager_HandleFrame(frame->type, frame->payload, frame->length);
}

void CommunicationManager_HandleFrame(const uint8_t type, const uint8_t *payload, uint8_t len)
{
    switch (type)
    {
    case MSG_PING:
    {
        uint8_t pong = 1U;
        (void)UartProtocol_Send(MSG_ACK, &pong, 1U);
        break;
    }

    case MSG_CMD_VELOCITY:
    {
        if (!Protocol_PayloadSizeValid(type, len)) { break; }
        float vx, vy, wz;
        UartProtocol_DecodeVelocity(payload, &vx, &vy, &wz);
        s_comm.cmdVx = vx;
        s_comm.cmdVy = vy;
        s_comm.cmdWz = wz;
        s_comm.lastCmdMs = HAL_GetTick();
        MotorControl_SetVelocity(vx, vy, wz);
        break;
    }

    case MSG_CMD_ENABLE:
    {
        if (len < 1U) { break; }
        if (payload[0] != 0U)
        {
            SafetyManager_ClearAllFaults();
            if (MotorControl_Recover() || MotorControl_IsEnabled())
            {
                SafetyManager_OnMotorsEnabled();
            }
        }
        else
        {
            MotorControl_Disable();
        }
        s_comm.enabled = (payload[0] != 0U);
        break;
    }

    case MSG_CMD_SET_PID:
    {
        if (len < 13U) { break; }
        PidGains gains;
        gains.kp = (float)proto_get_i32(&payload[1]) / (float)PROTO_PID_SCALE;
        gains.ki = (float)proto_get_i32(&payload[5]) / (float)PROTO_PID_SCALE;
        gains.kd = (float)proto_get_i32(&payload[9]) / (float)PROTO_PID_SCALE;
        gains.integMax = 0.6f;
        MotorControl_SetPid(payload[0], &gains);
        break;
    }

    case MSG_CMD_GET_ODOM:
    case MSG_CMD_GET_STATUS:
        /* The periodic telemetry loop answers within one period anyway */
        break;

    case MSG_RESET:
        MotorControl_Disable();
        NVIC_SystemReset();
        break;

    default:
    {
        uint8_t code = type;
        (void)UartProtocol_Send(MSG_NACK, &code, 1U);
        break;
    }
    }
}

/* ------------------------------------------------------------------ */
/* Periodic telemetry                                                  */
/* ------------------------------------------------------------------ */
static void SendTelemetry(void)
{
    uint32_t now = HAL_GetTick();
    SensorSnapshot snap;

    if (!SensorManager_GetSnapshot(&snap))
    {
        return;
    }

    if ((now - s_lastOdomMs) >= ODOM_PERIOD_MS)
    {
        s_lastOdomMs = now;
        ProtoOdom odom;
        odom.x = (int32_t)(snap.odom.x * 1000.0);
        odom.y = (int32_t)(snap.odom.y * 1000.0);
        odom.theta = (int16_t)(snap.odom.theta * 5729.5779f);   /* x0.01 deg */
        odom.v = (int16_t)(snap.odom.vx * 1000.0f);
        odom.w = (int16_t)(snap.odom.wz * 100.0f);
        odom.seq = (uint16_t)snap.snapshotSeq;
        uint8_t buf[16];
        (void)UartProtocol_Send(MSG_TLM_ODOM, buf,
                                (uint8_t)UartProtocol_EncodeOdom(buf, &odom));
    }

    if ((now - s_lastStatusMs) >= STATUS_PERIOD_MS)
    {
        s_lastStatusMs = now;
        ProtoStatus st;
        st.battery_mv = (int16_t)snap.battery.voltageMv;
        st.battery_ma = (int16_t)snap.battery.currentMa;
        st.state = (uint8_t)SafetyManager_GetState();
        st.faults = SafetyManager_GetFaults();
        st.seq = (uint16_t)snap.snapshotSeq;
        uint8_t buf[12];
        (void)UartProtocol_Send(MSG_TLM_STATUS, buf,
                                (uint8_t)UartProtocol_EncodeStatus(buf, &st));
    }

    if ((now - s_lastImuMs) >= IMU_PERIOD_MS)
    {
        s_lastImuMs = now;
        ProtoImu imu;
        imu.roll = (int16_t)(snap.imu.rollDeg * 100.0f);
        imu.pitch = (int16_t)(snap.imu.pitchDeg * 100.0f);
        imu.yaw = (int16_t)(snap.imu.yawDeg * 100.0f);
        imu.gx = (int16_t)(snap.imu.gyroDps[0] * 100.0f);
        imu.gy = (int16_t)(snap.imu.gyroDps[1] * 100.0f);
        imu.gz = (int16_t)(snap.imu.gyroDps[2] * 100.0f);
        uint8_t buf[12];
        (void)UartProtocol_Send(MSG_TLM_IMU, buf,
                                (uint8_t)UartProtocol_EncodeImu(buf, &imu));
    }

    if ((now - s_lastScanMs) >= SCAN_PERIOD_MS && snap.scanValid)
    {
        s_lastScanMs = now;
        uint8_t buf[16];
        (void)UartProtocol_Send(MSG_TLM_SCAN, buf,
                                (uint8_t)UartProtocol_EncodeScan(buf, snap.scanSectorCm, 16U));
    }
}

/* ------------------------------------------------------------------ */
/* DMA ring pumps                                                      */
/* ------------------------------------------------------------------ */
static void PumpDmaToParser(UART_HandleTypeDef *huart, uint8_t *ring, uint16_t size,
                            UartProtocol *link)
{
    if (huart->hdmarx == NULL)
    {
        return;
    }
    uint16_t ndtr = (uint16_t)(huart->hdmarx->NDTR & 0xFFFFU);
    uint16_t head = (uint16_t)(size - ndtr);
    static uint16_t s_tail[2] = { 0U, 0U };
    uint16_t *tail = (huart->Instance == UART4) ? &s_tail[0] : &s_tail[1];

    while (*tail != head)
    {
        UartProtocol_OnByte(link, ring[*tail]);
        *tail = (uint16_t)((*tail + 1U) % size);
    }
}

static void PumpLidar(void)
{
    if (huart1.hdmarx == NULL)
    {
        return;
    }
    uint16_t ndtr = (uint16_t)(huart1.hdmarx->NDTR & 0xFFFFU);
    uint16_t head = (uint16_t)(sizeof(s_lidarRxDma) - ndtr);
    static uint16_t tail = 0U;

    while (tail != head)
    {
        DrvLidar_OnByte(s_lidarRxDma[tail]);
        tail = (uint16_t)((tail + 1U) % sizeof(s_lidarRxDma));
        if (DrvLidar_GetState()->scanComplete)
        {
            SafetyManager_Heartbeat(HEARTBEAT_ID_LIDAR);
        }
    }
}

/* ------------------------------------------------------------------ */
/* CAN dispatch                                                        */
/* ------------------------------------------------------------------ */
static void Can_Send(uint32_t stdId, const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef header = { 0 };
    header.StdId = stdId;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = dlc;
    uint32_t mailbox;
    (void)HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
}

void CommunicationManager_OnCanFrame(uint32_t stdId, const uint8_t *data, uint8_t dlc)
{
    SafetyManager_ReportCanAlive();

    if ((stdId & 0xFF0U) == CAN_ID_HEARTBEAT_BASE)
    {
        uint8_t node = (uint8_t)(stdId & 0x0FU);
        if (node < 8U)
        {
            s_nodeBeat[node].lastMs = HAL_GetTick();
            s_nodeBeat[node].seen = true;
        }
        return;
    }

    switch (stdId)
    {
    case CAN_ID_MOTOR_ENCODERS0:
    case CAN_ID_MOTOR_ENCODERS1:
        /* Node encoders are used by diagnostics, not by the drive loop */
        break;

    case CAN_ID_SENSOR_DISTANCES:
    {
        uint16_t mm[4];
        (void)Can_DecodeDistances(data, dlc, mm);
        break;
    }

    case CAN_ID_SENSOR_POWER:
    {
        uint16_t mv, ma;
        float tempC;
        (void)Can_DecodePower(data, dlc, &mv, &ma, &tempC);
        break;
    }

    default:
        if ((stdId & 0xF00U) == 0x300U)
        {
            /* Node fault event: data[0..1] = fault code */
            uint16_t code = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
            SafetyManager_RaiseFault(FAULT_CAN_TIMEOUT, code);
        }
        break;
    }
}

void CommunicationManager_CanTick(void)
{
    uint32_t now = HAL_GetTick();

    /* Periodic master heartbeat on CAN */
    static uint32_t lastBeat = 0U;
    if ((now - lastBeat) >= 100U)
    {
        lastBeat = now;
        uint8_t payload[5];
        Can_EncodeHeartbeat(payload, (uint8_t)SafetyManager_GetState(),
                            (uint16_t)(now / 1000U), (uint16_t)SafetyManager_GetFaults());
        Can_Send(CAN_ID_HEARTBEAT_BASE | CAN_NODE_MASTER, payload, 5U);
    }

    /* Node supervision */
    for (uint8_t n = 1U; n < 8U; n++)
    {
        if (s_nodeBeat[n].seen && (now - s_nodeBeat[n].lastMs) > 1500U)
        {
            s_nodeBeat[n].seen = false;
            SafetyManager_RaiseFault(FAULT_CAN_TIMEOUT, n);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */
void CommunicationManager_Init(void)
{
    memset(&s_comm, 0, sizeof(s_comm));
    memset(s_nodeBeat, 0, sizeof(s_nodeBeat));
    UartProtocol_Init(&s_sbcLink, Sbc_OnFrame);
    UartProtocol_SetSender(Sbc_Send);

    /* Start the DMA circular receivers */
    (void)HAL_UART_Receive_DMA(&huart4, s_sbcRxDma, sizeof(s_sbcRxDma));
    (void)HAL_UART_Receive_DMA(&huart1, s_lidarRxDma, sizeof(s_lidarRxDma));
}

void CommunicationManager_Task(void)
{
    PumpDmaToParser(&huart4, s_sbcRxDma, sizeof(s_sbcRxDma), &s_sbcLink);
    PumpLidar();
    SendTelemetry();
    CommunicationManager_CanTick();
    SafetyManager_Heartbeat(HEARTBEAT_ID_COMM);
}

const CommStatus *CommunicationManager_GetStatus(void)
{
    return &s_comm;
}
