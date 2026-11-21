/**
 * @file    spi.c
 * @brief   SPI2 master @ ~5.25 MHz (PCLK1 42 MHz / 8), mode 0, 8-bit.
 *          Reserved for external SPI devices (flash log, RF module).
 */
#include "spi.h"
#include "stm32f4xx_hal_spi.h"

SPI_HandleTypeDef hspi2;

void MX_SPI2_Init(void)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = 0U;                       /* 2 lines full duplex */
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi2);
}
