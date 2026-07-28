#ifndef DEVICES_THERMISTOR_H_
#define DEVICES_THERMISTOR_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* NTC thermistors x3 -- HCET-103F3950 (Nanjing Haichuan), 10k @ 25C, B3950.
 *
 * Hardware (schematic nets THERMISTER1/2/3, U21 pins 26/27/28):
 *   THERMISTER1  PC0 = ADC1_IN10   (connector J19)
 *   THERMISTER2  PC1 = ADC1_IN11   (connector J22)
 *   THERMISTER3  PC2 = ADC1_IN12   (connector J26)
 *
 * Divider per channel (net 3P3V-MCU = 3.3V):
 *   3.3V --[ NTC (external probe, Jxx) ]--+--[ 1k Rfixed (R50/R56/R69) ]-- GND
 *                                         |
 *                                         +--[ 1k series (R49/R55/R68) ]-- ADC pin
 *                                         +--[ 0.1uF ]-- GND
 * The ADC pin is high-impedance, so the 1k series drops ~0V and Vadc = Vnode.
 * NTC sits on top, Rfixed on the bottom:
 *   Vadc = 3.3 * Rfixed / (Rntc + Rfixed)
 * Because the divider top and the ADC reference are both 3.3V the result is
 * ratiometric, so Rntc follows from the raw count alone (Vref cancels):
 *   Rntc = Rfixed * (ADC_MAX - raw) / raw
 * Temperature then comes from the Beta equation:
 *   1/T = 1/T0 + (1/B) * ln(Rntc / R0),  T0 = 298.15K, R0 = 10k, B = 3950.
 *
 * ADC access uses the shared adc_ctrl layer; call AdcCtrl_Init() once at
 * startup before Thermistor_Read*().
 *
 * Temperature is reported in Celsius to 0.1C resolution (three significant
 * digits for the normal operating range, e.g. 253 -> 25.3 C).
 */

#define THERMISTOR_COUNT        3u
#define THERMISTOR_ERR_D10  ((int16_t)-32768) /* Thermistor_Read*_d10 error   */

/* idx = 0..2  ->  THERMISTER1..3 */
void    Thermistor_Init(void);
float   Thermistor_ReadCelsius(uint8_t idx);      /* NAN on error              */
int16_t Thermistor_ReadCelsius_d10(uint8_t idx);  /* tenths of C, ERR on error */
uint16_t Thermistor_ReadRaw(uint8_t idx);         /* 12-bit counts, 0 on error */

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_THERMISTOR_H_ */
