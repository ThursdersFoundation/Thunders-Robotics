/**
 * @file    main.c
 * @brief   Thunders Robotic master firmware entry point.
 *
 * Task layout (mini-RTOS, priorities 1..6, 7 = highest):
 *
 *   TaskSafety   prio 6  100 Hz   supervision, IWDG feed, fault handling
 *   TaskControl  prio 5  1 kHz    encoders -> PID -> PWM
 *   TaskComm     prio 4  200 Hz   SBC frames, CAN dispatch, telemetry
 *   TaskSensors  prio 3  200 Hz   IMU/odom/battery/ultrasonic/lidar
 *   Idle         prio 0           WFI
 */
#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor_control.h"
#include "encoder.h"
#include "sensor_manager.h"
#include "communication_manager.h"
#include "safety_manager.h"
#include "drv_lidar.h"
#include <stdio.h>

/* Task bodies: the loop lives in main, the periodic work in the managers */
static void TaskControl(void *argument);
static void TaskSensors(void *argument);
static void TaskComm(void *argument);
static void TaskSafety(void *argument);

static void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* Bring-up order: buses -> peripherals -> drivers -> managers */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_CAN1_Init();
    MX_I2C1_Init();
    MX_SPI2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM6_Init();
    MX_TIM8_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_UART4_Init();

    /* Peripheral start (outside the RTOS: IRQs configured before tasks) */
    ADC_Start();
    CAN_Start();

    /* Application modules */
    MotorControl_Init();
    Odometry_Init();
    SafetyManager_Init();
    SensorManager_Init();
    CommunicationManager_Init();

    /* TIM6 = HAL timebase */
    HAL_TIM_Base_Start_IT(&htim6);
    /* TIM5 = ultrasonic capture */
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);

    /* Tasks */
    xTaskCreate(TaskSafety, "Safety", 256U, NULL, 6U, NULL);
    xTaskCreate(TaskControl, "Control", 512U, NULL, 5U, NULL);
    xTaskCreate(TaskComm, "Comm", 512U, NULL, 4U, NULL);
    xTaskCreate(TaskSensors, "Sensors", 384U, NULL, 3U, NULL);

    vTaskStartScheduler();

    /* Unreachable: scheduler runs forever */
    for (;;)
    {
    }
}

/* ------------------------------------------------------------------ */
/* RTOS task bodies                                                    */
/* ------------------------------------------------------------------ */
static void TaskControl(void *argument)
{
    (void)argument;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        MotorControl_ControlTick();
        SafetyManager_Heartbeat(HEARTBEAT_ID_CONTROL);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000U / CONTROL_LOOP_HZ));
    }
}

static void TaskSensors(void *argument)
{
    (void)argument;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        SensorManager_Task();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000U / SENSOR_TASK_HZ));
    }
}

static void TaskComm(void *argument)
{
    (void)argument;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        CommunicationManager_Task();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000U / COMM_TASK_HZ));
    }
}

static void TaskSafety(void *argument)
{
    (void)argument;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        SafetyManager_Task();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000U / SAFETY_TASK_HZ));
    }
}

/* ------------------------------------------------------------------ */
/* Clock tree                                                          */
/* ------------------------------------------------------------------ */
static void SystemClock_Config(void)
{
    /* SystemInit() (system_stm32f4xx.c) already configured the PLL.
     * NVIC priority grouping is the only runtime setting here. */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}
