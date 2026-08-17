#include "heater_hyst.h"

#include <stddef.h>   /* NULL */

/* Shared heater hysteresis/over-temp decision. See heater_hyst.h for the
 * contract. Pure: no side effects, no HW access. */

uint8_t Heater_IsOverTemp(const HeaterHyst_t *cfg,
                          int16_t temp_d10, uint8_t temp_valid)
{
	if ((cfg == NULL) || !temp_valid)
	{
		return 0u;
	}
	return (uint8_t)(temp_d10 >= cfg->safety_d10);
}

HeaterHystCmd_t Heater_HystDecide(const HeaterHyst_t *cfg,
                                  int16_t temp_d10, uint8_t temp_valid)
{
	/* No config or no trustworthy reading -> hold; the caller applies its own
	 * error policy (scenario keeps the pin, testbed forces OFF). */
	if ((cfg == NULL) || !temp_valid)
	{
		return HEATER_CMD_HOLD;
	}

	/* Over-temp beats everything (FW backstop; HW bimetal is the real guard). */
	if (temp_d10 >= cfg->safety_d10)
	{
		return HEATER_CMD_OFF;
	}

	/* Ordinary hysteresis band: off at/above off_d10, on at/below on_d10,
	 * hold in between so the output keeps its last state (needs on < off). */
	if (temp_d10 >= cfg->off_d10)
	{
		return HEATER_CMD_OFF;
	}
	if (temp_d10 <= cfg->on_d10)
	{
		return HEATER_CMD_ON;
	}
	return HEATER_CMD_HOLD;
}
