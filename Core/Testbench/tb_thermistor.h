#ifndef SRC_TB_THERMISTOR_H_
#define SRC_TB_THERMISTOR_H_

#include "main.h"
#include "thermistor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * tb_thermistor - monitor testbed for the 3 NTC thermistors
 * (THERMISTER1/2/3, PC0/PC1/PC2 = ADC1_IN10/11/12; see thermistor.c/adc_ctrl.h).
 *
 * Purpose: a debugger-watchable snapshot of all three probes. Set
 * tb_therm_enable = 1 (live watch / expression) and TB_Thermistor_Poll()
 * refreshes tb_therm_raw / tb_therm_c / tb_therm_c_d10 each cycle; set it back
 * to 0 and monitoring STOPS (the last values are frozen for inspection, no new
 * ADC reads happen). Index 0..2 = THERMISTER1..3.
 *
 * WHERE TO POLL - ADC1 THREAD-SAFETY:
 *   AdcCtrl_* reads reconfigure the shared ADC1 and are NOT reentrant; the
 *   whole design expects every ADC read from ONE task (defaultTask). So this
 *   testbed must be polled from StartDefaultTask (freertos.c), alongside the
 *   existing Distance_/Thermistor_ reads - never from StartMotorTask, or the
 *   two tasks would race on ADC1. The 100 ms defaultTask cadence is plenty for
 *   temperature.
 *
 * Call model (StartDefaultTask, freertos.c):
 *     AdcCtrl_Init();  Thermistor_Init();  TB_Thermistor_Init();
 *     for(;;) { ...; TB_Thermistor_Poll(); osDelay(100); }
 * ---------------------------------------------------------------------- */

/* Runtime switch -- set from the debugger while running. Default off.
 *   1 = monitor: each Poll() reads all 3 probes into the snapshot below.
 *   0 = stop:    Poll() does nothing; the last snapshot is kept for inspection. */
extern volatile uint8_t tb_therm_enable;

/* Latest snapshot, one entry per probe (idx 0..2 = THERMISTER1..3). Refreshed
 * only while tb_therm_enable != 0. Handy debugger watches:
 *   tb_therm_raw    - raw 12-bit ADC counts (last Tick)   (0 on a failed read)
 *   tb_therm_c      - SMOOTHED temperature in Celsius      (NAN on error)  <-- system value
 *   tb_therm_c_d10  - SMOOTHED temperature in 0.1 C units  (e.g. 253 -> 25.3 C;
 *                     THERMISTOR_ERR_D10 == -32768 on error)
 *   tb_therm_c_inst - INSTANTANEOUS (unfiltered) Celsius   (NAN on error)
 * tb_therm_c is the EMA-smoothed value the whole system uses (== g_therm_c_d10);
 * tb_therm_c_inst is a raw comparison read. The smoothing weight is the shared
 * g_therm_filter_alpha (thermistor.h, default 0.4) -- tune it live to taste. */
extern volatile uint16_t tb_therm_raw[THERMISTOR_COUNT];
extern volatile float    tb_therm_c[THERMISTOR_COUNT];
extern volatile int16_t  tb_therm_c_d10[THERMISTOR_COUNT];
extern volatile float    tb_therm_c_inst[THERMISTOR_COUNT];

/* Liveness counter: increments once per monitored Poll() (i.e. only while
 * enabled). Watch it tick to confirm monitoring is running. */
extern volatile uint32_t tb_therm_samples;

/* ---- Lifecycle ------------------------------------------------------------ */
/* Reset the switch (off) and clear the snapshot. Call once, after AdcCtrl_Init()
 * and Thermistor_Init(). */
void TB_Thermistor_Init(void);

/* One service cycle: if tb_therm_enable, read all 3 probes into the snapshot;
 * otherwise do nothing. Call from StartDefaultTask (~100 ms). */
void TB_Thermistor_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_THERMISTOR_H_ */
