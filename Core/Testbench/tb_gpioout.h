#ifndef SRC_TB_GPIOOUT_H_
#define SRC_TB_GPIOOUT_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * tb_gpioout - testbed for the plain ON/OFF GPIO output loads that have no
 * dedicated motor driver: one drain solenoid valve and three fans. Each is a
 * bare pin driven through gpio_ctrl, so there is no duty/direction -- only an
 * enable flag per load.
 *
 *   tb_valve_drain_en  -> VALVE_DRAIN_CLN (PB14) : cleaning-water drain valve
 *   tb_valve_dry_en    -> VALVE_DRY_IN    (PB13) : water-supply (dry-in) valve
 *   tb_fan_vapor_en    -> FAN_VAPOR       (PB15) : vapor fan
 *   tb_fan_exhaust_en  -> FAN_EXHAUST     (PG2)  : exhaust fan
 *   tb_bldc_fan_en     -> BLDC_FAN        (PG1)  : BLDC-driver cooling fan (R1)
 *
 * Usage: set a flag to 1 in the debugger (live watch / expression) and that
 * load turns on; set it back to 0 and it turns off. TB_GpioOut_Poll() mirrors
 * every flag onto its pin each cycle, so the pin latch always matches the flag.
 *
 * Call model (StartMotorTask, freertos.c, testbench mode only):
 *     TB_GpioOut_Init();
 *     for (;;) { ...; TB_GpioOut_Poll(); osDelay(1); }
 *
 * TESTBENCH ONLY: Poll() runs from MotorTask_RunTestbench(), so it is inert in
 * the Moeum/Dongjak scenarios (which own these same valves/fans themselves).
 * ---------------------------------------------------------------------- */

extern volatile uint8_t tb_valve_drain_en;  /* 1 = open drain valve, 0 = closed */
extern volatile uint8_t tb_valve_dry_en;    /* 1 = open supply valve,0 = closed */
extern volatile uint8_t tb_fan_vapor_en;    /* 1 = vapor fan on,   0 = off      */
extern volatile uint8_t tb_fan_exhaust_en;  /* 1 = exhaust fan on, 0 = off      */
extern volatile uint8_t tb_bldc_fan_en;     /* 1 = BLDC fan on,    0 = off      */

/* Force every load off and clear the flags. Call once, after MX_GPIO_Init(). */
void TB_GpioOut_Init(void);

/* Apply the four enable flags to their pins. Call every poll. */
void TB_GpioOut_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_GPIOOUT_H_ */
