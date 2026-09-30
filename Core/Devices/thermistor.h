#ifndef DEVICES_THERMISTOR_H_
#define DEVICES_THERMISTOR_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* NTC thermistors x3 -- HCET-103F3950 (Nanjing Haichuan), 10k @ 25C, B3950.
 *
 * Hardware (schematic nets THERMISTER1/2/3, U21 pins 26/27/28):
 *   THERMISTER1  PC0 = ADC1_IN10   (connector J21)
 *   THERMISTER2  PC1 = ADC1_IN11   (connector J22)
 *   THERMISTER3  PC2 = ADC1_IN12   (connector J23)
 * ★R3 C074(검토서 §8-⑤): 위 커넥터 번호는 J19/J22/J26 으로 잘못 적혀 있었다.
 * REV02 넷리스트 실측 — THERM1: R106 -> NetC93_2 -> **J21-1**, THERM2: R108 ->
 * NetC95_2 -> **J22-1**, THERM3: R110 -> NetC97_2 -> **J23-1**. (J26 은 마개 홀.)
 * 분압은 각 채널 1K(하단) + 10K(상단, R107/R109/R111). 동작 영향 없는 주석 정정.
 *
 * Divider per channel (net 3P3V-MCU = 3.3V). Rfixed is bench-confirmed 10k
 * (R107/R111/R109), NOT the 1k an earlier BOM read suggested:
 *   3.3V --[ NTC (external probe, Jxx) ]--+--[ 10k Rfixed (R107/R111/R109) ]-- GND
 *                                         |
 *                                         +--[ 1k series (R49/R55/R68) ]-- ADC pin
 *                                         +--[ 0.1uF ]-- GND
 * The ADC pin is high-impedance, so the 1k series drops ~0V and Vadc = Vnode.
 * With a 10k NTC and 10k Rfixed, 25C sits at mid-scale (raw~2048): room temp is
 * well resolved but HIGH-temp resolution is poor (~46 counts span 150..200C).
 * NTC sits on top, Rfixed on the bottom:
 *   Vadc = 3.3 * Rfixed / (Rntc + Rfixed)
 * Because the divider top and the ADC reference are both 3.3V the result is
 * ratiometric, so Rntc follows from the raw count alone (Vref cancels):
 *   Rntc = Rfixed * (ADC_MAX - raw) / raw
 * Temperature then comes from an R-T LOOKUP TABLE, interpolated piecewise-
 * linearly in (ln Rntc, 1/T). This replaces the earlier single-Beta equation,
 * which over-read by up to ~+10C at dryer temperatures. The table's -40..150C
 * rows are the VENDOR datasheet (batch A24-0428H-9, R25=10.0K/B3950), held to
 * ~0.1C; 160..200C continue the same (Adafruit 103_3950) curve and 210/220C are
 * extrapolated for 200C headroom -- all >150C rows are vendor-UNVERIFIED. Note
 * 200C sits near ADC saturation (~1C/count) on this 10k divider. See
 * thermistor.c for the table and the accuracy/headroom rationale.
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

/* Instantaneous (unfiltered) reads -- one fresh ADC conversion set each call. */
float   Thermistor_ReadCelsius(uint8_t idx);      /* NAN on error              */
int16_t Thermistor_ReadCelsius_d10(uint8_t idx);  /* tenths of C, ERR on error */
uint16_t Thermistor_ReadRaw(uint8_t idx);         /* 12-bit counts, 0 on error */

/* ---- Smoothed reads (first-order complementary / EMA low-pass) ----------
 * Instantaneous reads jitter, badly at high temperature where the 10k divider
 * gives coarse resolution (see divider note above). Thermistor_Tick() reads all
 * channels once and folds each into a per-channel EMA:
 *     filtered = filtered*(1 - alpha) + sample*alpha
 * so the reported value tracks real changes but rejects sample-to-sample noise.
 *
 * OWNERSHIP: call Thermistor_Tick() exactly ONCE per fixed period from the sole
 * sensor task (StartDefaultTask, 100 ms) -- the EMA assumes a steady dt and one
 * writer. Then read the smoothed value via Get*; a brief bad-read glitch is
 * ridden out, a persistent fault (THERM_ERR_LIMIT ticks) surfaces as NAN/ERR. */
extern volatile float g_therm_filter_alpha;       /* new-sample weight 0..1 (0.4) */

void     Thermistor_Tick(void);                   /* read all ch once + update EMA */
float    Thermistor_GetCelsius(uint8_t idx);      /* filtered, NAN on error/uninit */
int16_t  Thermistor_GetCelsius_d10(uint8_t idx);  /* filtered tenths, ERR on error */
uint16_t Thermistor_GetRaw(uint8_t idx);          /* raw from the last Tick, 0 else */

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_THERMISTOR_H_ */
