#ifndef SRC_TB_HALLSENSOR_H_
#define SRC_TB_HALLSENSOR_H_

#include "main.h"
#include "hallsensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * tb_hallsensor - monitor testbed for the Hall sensors read through the U24
 * TCA9554A expander (I2C1, schematic net "HALL-INT1" = PF9). Reuses the
 * production hallsensor.c driver, so the bits here match the firmware exactly;
 * this testbed only gates the reads behind tb_hall_enable and decodes the mask
 * into the four named monitoring channels below.
 *
 * U24 pin map (HallSensor mask bit i = HSi+1 = U24 Pi, polarity-normalised so a
 * set bit always means the magnet is present):
 *   P0 (bit0, HS1) : 강음(강한음식물) trigger  -> tb_hall_trig[0] / tb_hall_hard_food
 *   P1 (bit1, HS2) : 동작 trigger              -> tb_hall_trig[1] / tb_hall_run
 *   P2 (bit2, HS3) : 정지 trigger              -> tb_hall_trig[2] / tb_hall_stop
 *   P3 (bit3, HS4) : 배수 trigger              -> tb_hall_trig[3] / tb_hall_drain
 *   P4 (bit4, HS5) : 모음 trigger              -> tb_hall_trig[4] / tb_hall_collect
 *   P5 (bit5, HS6) : 교반원점 홀센서           -> tb_hall_stir_home
 *   P6 (bit6, HS7) : 수거통 홀센서             -> tb_hall_bin
 *   P7 (bit7, HS8) : 리프트하단 홀센서         -> tb_hall_lift_bottom
 *
 * INTERRUPT: U24 pulls HALL-INT1 (PF9, active-low open-drain) low on any input
 * change; the EXTI edge is latched by gpio_ctrl and serviced by
 * HallSensor_ServiceInt(). While enabled, each Poll() services that edge for
 * low latency and also does a periodic HallSensor_Update() as a safety net for a
 * missed edge, then refreshes the snapshot below.
 *
 * WHERE TO POLL: hallsensor.c talks to U24 over I2C1. Poll this testbed from the
 * same task that already owns the HS reads (StartDefaultTask in freertos.c),
 * after HallSensor_Init(), so the bus is only driven from one place.
 *
 * Call model (StartDefaultTask, freertos.c):
 *     HallSensor_Init();  TB_HallSensor_Init();
 *     for(;;) { ...; TB_HallSensor_Poll(); osDelay(100); }
 * ---------------------------------------------------------------------- */

/* Bit index of each channel inside the HallSensor mask (bit i = U24 Pi). */
#define TB_HALL_TRIG_COUNT      5U   /* P0..P4 trigger group */
#define TB_HALL_BIT_HARD_FOOD   0U   /* P0 강음(강한음식물) */
#define TB_HALL_BIT_COLLECT     4U   /* P4 모음  (2026-08-25 라벨 재정의) */
#define TB_HALL_BIT_STOP        2U   /* P2 정지  */
#define TB_HALL_BIT_DRAIN       3U   /* P3 배수  */
#define TB_HALL_BIT_RUN         1U   /* P1 동작  (2026-08-25 라벨 재정의) */
#define TB_HALL_BIT_STIR_HOME   5U   /* P5 교반원점  */
#define TB_HALL_BIT_BIN         6U   /* P6 수거통    */
#define TB_HALL_BIT_LIFT_BOTTOM 7U   /* P7 리프트하단 */

/* Runtime switch -- set from the debugger while running. Default off.
 *   1 = monitor: each Poll() services HALL-INT1, reads U24 and refreshes the
 *                snapshot / edge counters below.
 *   0 = stop:    Poll() does nothing; the last snapshot is kept for inspection
 *                and no U24 read is issued from this testbed. */
extern volatile uint8_t tb_hall_enable;

/* Latest snapshot. Refreshed only while tb_hall_enable != 0. */
extern volatile uint8_t tb_hall_mask;          /* full P0..P7 detected mask     */

/* Diagnostic: the actual pin VOLTAGE level of P0..P7 (1 = pin HIGH). Because the
 * driver programs U24's polarity-inversion register to 0xFF (active-low
 * sensors), this is just ~tb_hall_mask, but it is the view to watch when hunting
 * a wiring fault: HS1..HS8 have NO pull-up resistors in the netlist and the
 * TCA9554A input port is high-impedance (no internal pull-up), so any HS line
 * whose sensor is not actively driving it FLOATS. Floating neighbours couple, so
 * triggering one such line drags the others - which reads as several detected
 * bits at once. If bits move together HERE (voltage domain), the fault is
 * electrical (missing pull-up / unconnected sensor), not firmware. */
extern volatile uint8_t tb_hall_pinlevel;      /* full P0..P7 pin-voltage mask  */

/* Channel 1 - trigger group P0..P4. tb_hall_trig[i] is the raw index view; the
 * named aliases below mirror the same bits for readable debugger watches. */
extern volatile uint8_t tb_hall_trig[TB_HALL_TRIG_COUNT]; /* per-trigger 0/1     */
extern volatile uint8_t tb_hall_trig_mask;     /* P0..P4 packed (0..0x1F)        */
extern volatile uint8_t tb_hall_hard_food;     /* P0 강음(강한음식물) */
extern volatile uint8_t tb_hall_collect;       /* P4 모음 */
extern volatile uint8_t tb_hall_stop;          /* P2 정지 */
extern volatile uint8_t tb_hall_drain;         /* P3 배수 */
extern volatile uint8_t tb_hall_run;           /* P1 동작 */

/* Channels 2..4 - named position sensors (1 = magnet present). */
extern volatile uint8_t tb_hall_stir_home;     /* P5 교반원점   */
extern volatile uint8_t tb_hall_bin;           /* P6 수거통     */
extern volatile uint8_t tb_hall_lift_bottom;   /* P7 리프트하단 */

/* Rising-edge counters (each ++ on a 0->1 transition of its channel). Handy to
 * confirm a trigger/sensor actually fired without catching the level live. */
extern volatile uint32_t tb_hall_trig_events[TB_HALL_TRIG_COUNT];
extern volatile uint32_t tb_hall_stir_home_events;
extern volatile uint32_t tb_hall_bin_events;
extern volatile uint32_t tb_hall_lift_bottom_events;

/* Liveness / diagnostics. */
extern volatile uint32_t tb_hall_samples;      /* ++ per enabled Poll()          */
extern volatile uint32_t tb_hall_int_events;   /* ++ per serviced HALL-INT1 edge */
extern volatile uint32_t tb_hall_changes;      /* ++ whenever the mask changes   */

/* ---- Lifecycle ------------------------------------------------------------ */
/* Reset the switch (off) and clear the snapshot / counters. Call once, after
 * HallSensor_Init(). */
void TB_HallSensor_Init(void);

/* One service cycle: if tb_hall_enable, service HALL-INT1 + read U24 and refresh
 * the snapshot; otherwise do nothing. Call from StartDefaultTask (~100 ms). */
void TB_HallSensor_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_HALLSENSOR_H_ */
