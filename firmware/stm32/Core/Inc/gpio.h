/**
 * @file    gpio.h
 * @brief   Digital I/O configuration: LEDs, button (EXTI), motor direction,
 *          ultrasonic triggers, lidar motor enable.
 */
#ifndef THUNDERS_GPIO_H
#define THUNDERS_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_GPIO_H */
