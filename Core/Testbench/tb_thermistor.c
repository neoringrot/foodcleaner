#include "tb_thermistor.h"

/* Monitor testbed for the 3 NTC thermistors. See tb_thermistor.h for the
 * contract and the ADC1-single-task constraint (poll from defaultTask only).
 *
 * The SMOOTHED temperature (tb_therm_c / tb_therm_c_d10) is taken straight from
 * the shared thermistor.c EMA via the Get* getters, so it matches g_therm_c_d10[]
 * exactly -- the filter itself is driven once per 100 ms cycle by the production
 * Thermistor_Tick() (called before this poll), NOT here. tb_therm_c_inst is a
 * fresh unfiltered read exposed alongside so the filter's effect is visible:
 * watch tb_therm_c_inst / tb_therm_raw jitter while tb_therm_c stays smooth, and
 * tune g_therm_filter_alpha (thermistor.c) live to taste. */

/* Runtime switch -- set from the debugger while running. Default off/safe. */
volatile uint8_t tb_therm_enable = 0;

/* Latest snapshot (idx 0..2 = THERMISTER1..3). */
volatile uint16_t tb_therm_raw[THERMISTOR_COUNT]    = {0};  /* raw from last Tick   */
volatile float    tb_therm_c[THERMISTOR_COUNT]      = {0};  /* SMOOTHED Celsius     */
volatile int16_t  tb_therm_c_d10[THERMISTOR_COUNT]  = {0};  /* SMOOTHED tenths      */
volatile float    tb_therm_c_inst[THERMISTOR_COUNT] = {0};  /* instantaneous (raw)  */

/* Liveness counter: ticks once per monitored poll. */
volatile uint32_t tb_therm_samples = 0;

void TB_Thermistor_Init(void)
{
	uint8_t i;

	tb_therm_enable  = 0;
	tb_therm_samples = 0;

	for (i = 0; i < THERMISTOR_COUNT; i++)
	{
		tb_therm_raw[i]    = 0;
		tb_therm_c[i]      = 0.0f;
		tb_therm_c_d10[i]  = 0;
		tb_therm_c_inst[i] = 0.0f;
	}
}

void TB_Thermistor_Poll(void)
{
	uint8_t i;

	/* Disabled: monitoring stopped. Keep the last snapshot untouched so it can
	 * still be inspected. (The production filter keeps running regardless; this
	 * only gates the snapshot copy and the instantaneous compare-read.) */
	if (!tb_therm_enable)
	{
		return;
	}

	/* Enabled: copy the smoothed system values (cached by Thermistor_Tick this
	 * cycle) and take one fresh unfiltered read per channel for comparison. */
	for (i = 0; i < THERMISTOR_COUNT; i++)
	{
		tb_therm_raw[i]    = Thermistor_GetRaw(i);
		tb_therm_c[i]      = Thermistor_GetCelsius(i);
		tb_therm_c_d10[i]  = Thermistor_GetCelsius_d10(i);
		tb_therm_c_inst[i] = Thermistor_ReadCelsius(i);   /* unfiltered snapshot */
	}

	tb_therm_samples++;
}
