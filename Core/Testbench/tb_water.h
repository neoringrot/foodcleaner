#ifndef SRC_TB_WATER_H_
#define SRC_TB_WATER_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * tb_water - testbed for the clean-water fill path: the WATER-ON supply
 * output plus the two water-level sensors WATER-SEN1 / WATER-SEN2.
 *
 *   o_WATER_ON      (PE2) : water supply valve/pump enable (output, act.high)
 *   exti6_WATER_SEN1(PF6) : water-level sensor 1 (EXTI, falling edge)
 *   exti7_WATER_SEN2(PF7) : water-level sensor 2 (EXTI, falling edge)
 *
 * Flow (matches the requested behaviour):
 *   if (tb_water_enable) {
 *       drive WATER-ON high;                 // open the supply
 *       recognise WATER-SEN1 / WATER-SEN2;   // levels + present + edge counts
 *   } else {
 *       drive WATER-ON low;                  // supply off, safe
 *   }
 *
 * Sensor polarity: the water-level sensors are wired active-low on this board
 * (water bridging the probe pulls the pin LOW), the same convention the Dongjak
 * scenario uses (DJ_WATER_ACTIVE_LOW). "present" applies that inversion so it
 * reads 1 = water reached, independent of the raw level. Override
 * TB_WATER_ACTIVE_LOW before including this header to test active-high probes.
 *
 * Edge counts come from the shared gpio_ctrl EXTI flags (PF6/PF7 are configured
 * GPIO_MODE_IT_FALLING): every falling edge latched by the ISR is counted here.
 * The flags are cleared on the enable edge so a level already asserted when the
 * testbed starts is not miscounted as a fresh event.
 *
 * Usage: set tb_water_enable = 1 in the debugger. WATER-ON turns on and the
 * tb_water_sen*_level / _present / _events fields update every poll; clear it
 * back to 0 to shut the supply off and stop monitoring.
 *
 * Call model (StartDefaultTask, freertos.c - the 100 ms sensor loop that also
 * owns the other enable-gated monitors and polls the EXTI flags):
 *     TB_Water_Init();
 *     for (;;) { ...; TB_Water_Poll(); osDelay(100); }
 *
 * TESTBENCH ONLY: keep tb_water_enable = 0 while running the Moeum/Dongjak
 * scenarios - those own WATER-ON sequencing and consume the same SEN flags, so
 * enabling this testbed would fight them over the supply and the EXTI flags.
 * ---------------------------------------------------------------------- */

/* Sensor polarity: 1 = active-low (water present pulls the pin LOW). */
#ifndef TB_WATER_ACTIVE_LOW
#define TB_WATER_ACTIVE_LOW  1
#endif

/* Runtime switch -- set from the debugger while running. Default off/safe. */
extern volatile uint8_t tb_water_enable;      /* 1 = supply on + monitor, 0 = off */

/* Latest snapshot (updated only while enabled). */
extern volatile uint8_t tb_water_on_state;    /* WATER-ON output latch read-back 0/1 */

extern volatile uint8_t tb_water_sen1_level;  /* WATER-SEN1 raw pin level 0/1        */
extern volatile uint8_t tb_water_sen2_level;  /* WATER-SEN2 raw pin level 0/1        */

extern volatile uint8_t tb_water_sen1_present;/* 1 = water reached SEN1 (polarity applied) */
extern volatile uint8_t tb_water_sen2_present;/* 1 = water reached SEN2 (polarity applied) */
extern volatile uint8_t tb_water_present;     /* 1 = either sensor reports water     */

extern volatile uint32_t tb_water_sen1_events;/* WATER-SEN1 falling-edge count       */
extern volatile uint32_t tb_water_sen2_events;/* WATER-SEN2 falling-edge count       */

extern volatile uint32_t tb_water_samples;    /* poll count while enabled            */

/* Force WATER-ON off and clear all state/flags. Call once, after MX_GPIO_Init(). */
void TB_Water_Init(void);

/* Apply the enable flag: drive WATER-ON and refresh the sensor fields. Call
 * every poll. */
/* tb_active: 1 = 벤치가 WATER-ON(PE2)을 소유해도 되는 상태(g_app_mode ==
 * APP_MODE_TESTBENCH). 0 이면 시나리오가 그 핀을 쓰는 중이므로 아무것도
 * 하지 않고 돌아온다 - tb_heat 의 HT-POWER 와 같은 규약이다. */
void TB_Water_Poll(uint8_t tb_active);

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_WATER_H_ */
