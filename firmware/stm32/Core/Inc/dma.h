/**
 * @file    dma.h
 * @brief   DMA controller init: clocks + NVIC (stream setup is done by the
 *          HAL peripherals that own each stream).
 */
#ifndef THUNDERS_DMA_H
#define THUNDERS_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void MX_DMA_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_DMA_H */
