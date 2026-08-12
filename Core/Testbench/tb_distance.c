#include "tb_distance.h"

/* Monitor testbed for the Distance-SEN IR distance sensor. See tb_distance.h
 * for the contract and the ADC1-single-task constraint (poll from defaultTask
 * only).
 *
 * Reads go through the same distance.c API used in production, so the values
 * here match the firmware exactly; this testbed just gates the reads behind
 * tb_dist_enable and exposes the raw counts / voltage / cm / mm / fill% views
 * for probing. */

/* Runtime switch -- set from the debugger while running. Default off/safe. */
volatile uint8_t tb_dist_enable = 0;

/* Latest snapshot. */
volatile uint16_t tb_dist_raw  = 0;
volatile float    tb_dist_volt = 0.0f;
volatile float    tb_dist_cm   = 0.0f;
volatile uint16_t tb_dist_mm   = 0;
volatile uint8_t  tb_dist_fill = 0;

/* Liveness counter: ticks once per monitored poll. */
volatile uint32_t tb_dist_samples = 0;

void TB_Distance_Init(void)
{
	tb_dist_enable  = 0;
	tb_dist_samples = 0;

	tb_dist_raw  = 0;
	tb_dist_volt = 0.0f;
	tb_dist_cm   = 0.0f;
	tb_dist_mm   = 0;
	tb_dist_fill = 0;
}

void TB_Distance_Poll(void)
{
	/* Disabled: monitoring stopped. Keep the last snapshot untouched so it can
	 * still be inspected, and take no ADC reads. */
	if (!tb_dist_enable)
	{
		return;
	}

	/* Enabled: refresh the probe. Raw first, then the derived views (each goes
	 * through distance.c and reads the Distance-SEN pin on its own). */
	tb_dist_raw  = Distance_ReadRaw();
	tb_dist_volt = Distance_ReadVoltage();
	tb_dist_cm   = Distance_ReadCm();
	tb_dist_mm   = Distance_ReadMm();
	tb_dist_fill = Distance_ReadFillPercent();

	tb_dist_samples++;
}
