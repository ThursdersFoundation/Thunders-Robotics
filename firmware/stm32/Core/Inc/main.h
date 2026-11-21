/**
 * @file    main.h
 * @brief   Thunders Robotic main firmware header: pin map, versions, exports.
 *
 * Board: custom carrier with STM32F407VGT6 (LQFP100), 8 MHz HSE.
 * Role:  master controller - omni drive (4x VNH5019), navigation sensors,
 *        CAN backbone to the Arduino Mega nodes, UART link to the SBC.
 */
#ifndef THUNDERS_MAIN_H
#define THUNDERS_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define THUNDERS_FW_VERSION_MAJOR 1
#define THUNDERS_FW_VERSION_MINOR 0
#define THUNDERS_FW_VERSION_PATCH 0

/* ================================================================== */
/* Status LEDs (active high)                                          */
/* ================================================================== */
#define LED_STATUS_Pin        GPIO_PIN_12   /* green  - RUNNING   */
#define LED_STATUS_GPIO_Port  GPIOD
#define LED_ERROR_Pin         GPIO_PIN_13   /* red    - FAULT     */
#define LED_ERROR_GPIO_Port   GPIOD
#define LED_COMM_Pin          GPIO_PIN_14   /* blue   - link alive */
#define LED_COMM_GPIO_Port    GPIOD
#define LED_HEARTBEAT_Pin     GPIO_PIN_15   /* amber  - scheduler */
#define LED_HEARTBEAT_GPIO_Port GPIOD

/* ================================================================== */
/* User button (active low, EXTI13)                                   */
/* ================================================================== */
#define USER_BTN_Pin          GPIO_PIN_13
#define USER_BTN_GPIO_Port    GPIOC

/* ================================================================== */
/* VNH5019 motor driver: direction inputs on GPIOE (PWM on TIM2)       */
/* ================================================================== */
#define MOTOR1_INA_Pin        GPIO_PIN_0    /* front left  */
#define MOTOR1_INA_GPIO_Port  GPIOE
#define MOTOR1_INB_Pin        GPIO_PIN_1
#define MOTOR1_INB_GPIO_Port  GPIOE
#define MOTOR2_INA_Pin        GPIO_PIN_2    /* front right */
#define MOTOR2_INA_GPIO_Port  GPIOE
#define MOTOR2_INB_Pin        GPIO_PIN_3
#define MOTOR2_INB_GPIO_Port  GPIOE
#define MOTOR3_INA_Pin        GPIO_PIN_4    /* rear left   */
#define MOTOR3_INA_GPIO_Port  GPIOE
#define MOTOR3_INB_Pin        GPIO_PIN_5
#define MOTOR3_INB_GPIO_Port  GPIOE
#define MOTOR4_INA_Pin        GPIO_PIN_6    /* rear right  */
#define MOTOR4_INA_GPIO_Port  GPIOE
#define MOTOR4_INB_Pin        GPIO_PIN_7
#define MOTOR4_INB_GPIO_Port  GPIOE

/* ================================================================== */
/* Ultrasonic HC-SR04 triggers (echo measured by TIM5 IC)              */
/* ================================================================== */
#define USONIC_FRONT_TRIG_Pin       GPIO_PIN_0
#define USONIC_FRONT_TRIG_GPIO_Port GPIOB
#define USONIC_LEFT_TRIG_Pin        GPIO_PIN_1
#define USONIC_LEFT_TRIG_GPIO_Port  GPIOB
#define USONIC_RIGHT_TRIG_Pin       GPIO_PIN_4
#define USONIC_RIGHT_TRIG_GPIO_Port GPIOC
#define USONIC_REAR_TRIG_Pin        GPIO_PIN_5
#define USONIC_REAR_TRIG_GPIO_Port  GPIOC

/* ================================================================== */
/* RPLidar A1M8                                                        */
/* ================================================================== */
#define LIDAR_MOTO_EN_Pin       GPIO_PIN_8
#define LIDAR_MOTO_EN_GPIO_Port GPIOA

/* ================================================================== */
/* Robot geometry (omni 4-wheel, X configuration, wheels at +-45 deg)  */
/* ================================================================== */
#define ROBOT_WHEEL_RADIUS_M      0.040f    /* 80 mm omni wheels      */
#define ROBOT_WHEELBASE_X_M       0.150f    /* half distance L/R axles */
#define ROBOT_WHEELBASE_Y_M       0.150f    /* half distance F/R axles */
#define ROBOT_GEAR_RATIO          30.0f
#define ROBOT_ENCODER_PPR         11.0f

/* Control rates */
#define CONTROL_LOOP_HZ           1000U     /* PID + motor tick        */
#define SENSOR_TASK_HZ            200U      /* IMU + odometry          */
#define USONIC_TASK_HZ            40U       /* one trigger per call    */
#define COMM_TASK_HZ              200U      /* frame pump              */
#define SAFETY_TASK_HZ            100U      /* supervision + IWDG      */

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_MAIN_H */
