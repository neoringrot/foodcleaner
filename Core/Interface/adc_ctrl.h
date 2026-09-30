#ifndef INTERFACE_ADC_CTRL_H_
#define INTERFACE_ADC_CTRL_H_

#include "adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Shared ADC1 access layer.
 *
 * ADC1 is wired to seven analog inputs (schematic nets):
 *   Distance-SEN  PA0 = ADC1_IN0   -> GP2Y0A41SK IR range sensor  (distance.c)
 *   THERMISTER1   PC0 = ADC1_IN10  -> NTC 10k B3950               (thermistor.c)
 *   THERMISTER2   PC1 = ADC1_IN11  -> NTC 10k B3950               (thermistor.c)
 *   THERMISTER3   PC2 = ADC1_IN12  -> NTC 10k B3950               (thermistor.c)
 *   DC-ADC1       PC5 = ADC1_IN15  -> DC motor current sense (R1 new)
 *   DC-ADC2       PB0 = ADC1_IN8   -> DC motor current sense (R1 new)
 *   DC-ADC3       PB1 = ADC1_IN9   -> DC motor current sense (R1 new)
 *
 * CubeMX generates ADC1 as a 7-rank *scan* sequence with no DMA. Polling that
 * sequence only ever yields the last rank, so this layer instead runs ADC1 in
 * single-conversion mode and reads one channel at a time. AdcCtrl_Init() must
 * be called once (after MX_ADC1_Init()) before any read; it calibrates the ADC
 * and switches it into single-conversion mode.
 *
 * Thread-safety: reads reconfigure the shared ADC1 and are NOT reentrant. All
 * AdcCtrl_* reads are expected from a single task (defaultTask). If you need to
 * read from several tasks, wrap the reads in a mutex.
 */

typedef enum
{
	ADC_CH_DISTANCE = ADC_CHANNEL_0,    /* PA0  Distance-SEN */
	ADC_CH_THERM1   = ADC_CHANNEL_10,   /* PC0  THERMISTER1  */
	ADC_CH_THERM2   = ADC_CHANNEL_11,   /* PC1  THERMISTER2  */
	ADC_CH_THERM3   = ADC_CHANNEL_12,   /* PC2  THERMISTER3  */
	ADC_CH_DC_CURR1 = ADC_CHANNEL_15,   /* PC5  DC-ADC1 (R1) */
	ADC_CH_DC_CURR2 = ADC_CHANNEL_8,    /* PB0  DC-ADC2 (R1) */
	ADC_CH_DC_CURR3 = ADC_CHANNEL_9,    /* PB1  DC-ADC3 (R1) */
	ADC_CH_WEIGHT   = ADC_CHANNEL_13,   /* PC3  weight-ADC (REV02 J40 → R178/C128 → R174, C078·§0.37) */
} AdcCtrl_Channel_t;

#define ADC_CTRL_ADC_MAX   4095u        /* 12-bit ADC full scale             */

void              AdcCtrl_Init(void);   /* calibrate + single-conversion mode */

/* One polled conversion of `channel`. Returns HAL_OK and writes *out (raw
 * 12-bit counts) on success. */
HAL_StatusTypeDef AdcCtrl_Read(AdcCtrl_Channel_t channel, uint16_t *out);

/* Median of `samples` conversions of `channel` (samples clamped to 1..15).
 * Returns raw 12-bit counts, or 0 if every conversion failed. */
uint16_t          AdcCtrl_ReadMedian(AdcCtrl_Channel_t channel, uint8_t samples);

#ifdef __cplusplus
}
#endif

#endif /* INTERFACE_ADC_CTRL_H_ */
