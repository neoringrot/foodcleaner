#ifndef SRC_TB_DISTANCE_H_
#define SRC_TB_DISTANCE_H_

#include "main.h"
#include "distance.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * tb_distance - monitor testbed for the Sharp GP2Y0A41SK0F IR distance sensor
 * (schematic net "Distance-SEN", PA0-WKUP = ADC1_IN0; see distance.c/adc_ctrl.h).
 *
 * Purpose: a debugger-watchable snapshot of the bin fill-level probe. Set
 * tb_dist_enable = 1 (live watch / expression) and TB_Distance_Poll() reads the
 * Distance-SEN pin each cycle, refreshing the snapshot below; set it back to 0
 * and monitoring STOPS (the last values are frozen for inspection, no new ADC
 * reads happen).
 *
 * WHERE TO POLL - ADC1 THREAD-SAFETY:
 *   AdcCtrl_* reads reconfigure the shared ADC1 and are NOT reentrant; the whole
 *   design expects every ADC read from ONE task (defaultTask). So this testbed
 *   must be polled from StartDefaultTask (freertos.c), alongside the existing
 *   Distance_/Thermistor_ reads - never from StartMotorTask, or the two tasks
 *   would race on ADC1. The 100 ms defaultTask cadence is fine (sensor cycle is
 *   16.5 ms).
 *
 * Call model (StartDefaultTask, freertos.c):
 *     AdcCtrl_Init();  Distance_Init();  TB_Distance_Init();
 *     for(;;) { ...; TB_Distance_Poll(); osDelay(100); }
 * ---------------------------------------------------------------------- */

/* Runtime switch -- set from the debugger while running. Default off.
 *   1 = monitor: each Poll() reads the Distance-SEN pin into the snapshot below.
 *   0 = stop:    Poll() does nothing; the last snapshot is kept for inspection. */
extern volatile uint8_t tb_dist_enable;

/* Latest snapshot. Refreshed only while tb_dist_enable != 0. Handy watches:
 *   tb_dist_raw     - raw 12-bit ADC counts   (median; 0 on a failed read)
 *   tb_dist_volt    - recovered sensor Vo [V]  (<0 on error)
 *   tb_dist_cm      - distance [cm], 4..30      (<0 on error)
 *   tb_dist_mm      - distance [mm]             (DISTANCE_ERR_MM == 0 on error)
 *   tb_dist_fill    - bin fill level 0..100 (%) */
extern volatile uint16_t tb_dist_raw;
extern volatile float    tb_dist_volt;
extern volatile float    tb_dist_cm;
extern volatile uint16_t tb_dist_mm;
extern volatile uint8_t  tb_dist_fill;

/* Liveness counter: increments once per monitored Poll() (i.e. only while
 * enabled). Watch it tick to confirm monitoring is running. */
extern volatile uint32_t tb_dist_samples;

/* ---- Lifecycle ------------------------------------------------------------ */
/* Reset the switch (off) and clear the snapshot. Call once, after AdcCtrl_Init()
 * and Distance_Init(). */
void TB_Distance_Init(void);

/* One service cycle: if tb_dist_enable, read the Distance-SEN pin into the
 * snapshot; otherwise do nothing. Call from StartDefaultTask (~100 ms). */
void TB_Distance_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_DISTANCE_H_ */
