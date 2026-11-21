/**
 * @file    adc.h
 * @brief   ADC1: battery voltage / current / temperature scan into DMA.
 */
#ifndef THUNDERS_ADC_H
#define THUNDERS_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern volatile uint16_t adc_raw[3];   /* [0]=Vbat [1]=Ibat [2]=Temp */

void MX_ADC1_Init(void);
void ADC_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_ADC_H */
