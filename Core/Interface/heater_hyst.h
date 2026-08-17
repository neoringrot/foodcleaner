#ifndef INTERFACE_HEATER_HYST_H_
#define INTERFACE_HEATER_HYST_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * heater_hyst - shared, HW-agnostic heater hysteresis + over-temp decision.
 *
 * Both the 동작(dongjak) scenario's dj_heater_tick() and the tb_heat testbed
 * run the SAME 190/195C hysteresis (with a 210C over-temp cut) on the same
 * NTC temperature. This module is the single source of that decision so the
 * two cannot drift apart: change the band here (or the thresholds each caller
 * passes) and both follow.
 *
 * It is a PURE function of (config, temperature, temp-valid): no GPIO, no HAL,
 * no state. The caller owns the pin and applies its own policies around the
 * decision (the scenario HOLDs the pin on a sensor error; the testbed forces
 * OFF and adds a watchdog / fault latch). Temperatures are in 0.1C units, so
 * 1900 == 190.0C.
 * ---------------------------------------------------------------------- */

typedef struct
{
	int16_t on_d10;      /* temp <= on_d10  -> heater ON   (e.g. 1900 = 190.0C) */
	int16_t off_d10;     /* temp >= off_d10 -> heater OFF  (e.g. 1950 = 195.0C) */
	int16_t safety_d10;  /* temp >= safety  -> heater OFF  (e.g. 2100 = 210.0C) */
} HeaterHyst_t;

typedef enum
{
	HEATER_CMD_OFF  = 0,  /* drive OFF: above the off threshold or over-temp     */
	HEATER_CMD_ON   = 1,  /* drive ON:  at/below the on threshold                */
	HEATER_CMD_HOLD = 2   /* keep previous level: inside the band, or no valid   */
	                      /*   temperature (caller decides what HOLD means)      */
} HeaterHystCmd_t;

/* Hysteresis + over-temp decision. temp_valid == 0 (probe open/short -> ERR)
 * yields HOLD so the caller can choose its own error behaviour: the scenario
 * leaves the pin as-is (HOLD == keep), the testbed treats HOLD-on-error as OFF.
 * Precedence when temp_valid: over-temp -> OFF, else >= off -> OFF, else
 * <= on -> ON, else (inside band) -> HOLD. */
HeaterHystCmd_t Heater_HystDecide(const HeaterHyst_t *cfg,
                                  int16_t temp_d10, uint8_t temp_valid);

/* Convenience predicate: is the over-temp safety threshold tripped right now?
 * (temp_valid && temp_d10 >= safety_d10). Callers use this to raise/latch an
 * over-temp fault distinct from the ordinary OFF from the hysteresis band. */
uint8_t Heater_IsOverTemp(const HeaterHyst_t *cfg,
                          int16_t temp_d10, uint8_t temp_valid);

#ifdef __cplusplus
}
#endif

#endif /* INTERFACE_HEATER_HYST_H_ */
