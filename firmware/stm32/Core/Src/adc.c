/**
 * @file    adc.c
 * @brief   ADC1 scan of the power board: VBAT divider (PC0/CH10),
 *          ACS712 current (PC1/CH11), NTC temperature (PC2/CH12).
 *          Continuous scan streamed by DMA2 Stream0 into adc_raw[3].
 */
#include "adc.h"

ADC_HandleTypeDef hadc1;
volatile uint16_t adc_raw[3] = { 0U, 0U, 0U };

void MX_ADC1_Init(void)
{
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;   /* 21 MHz */
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = 0U;                         /* right  */
    hadc1.Init.ScanConvMode          = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.NbrOfConversion       = 3U;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef cfg;
    cfg.Rank = 1U;
    cfg.Channel = ADC_CHANNEL_10;
    cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    cfg.Offset = 0U;
    HAL_ADC_ConfigChannel(&hadc1, &cfg);

    cfg.Rank = 2U;
    cfg.Channel = ADC_CHANNEL_11;
    HAL_ADC_ConfigChannel(&hadc1, &cfg);

    cfg.Rank = 3U;
    cfg.Channel = ADC_CHANNEL_12;
    HAL_ADC_ConfigChannel(&hadc1, &cfg);
}

void ADC_Start(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw, 3U);
}
