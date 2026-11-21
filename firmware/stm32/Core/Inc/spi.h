/**
 * @file    spi.h
 * @brief   SPI2 master (PB12-PB15): reserved for external SPI devices.
 */
#ifndef THUNDERS_SPI_H
#define THUNDERS_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern SPI_HandleTypeDef hspi2;

void MX_SPI2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_SPI_H */
