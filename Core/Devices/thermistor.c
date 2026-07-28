#include "thermistor.h"
#include "adc_ctrl.h"
#include <math.h>

/* ---- NTC / divider constants ------------------------------------------- */
#define THERM_ADC_MAX       4095.0f     /* 12-bit full scale                 */
#define THERM_R_FIXED       1000.0f     /* R50/R56/R69 pull-down [ohm]       */
#define THERM_R0            10000.0f    /* NTC R @ 25C  [ohm]                */
#define THERM_T0_K          298.15f     /* 25C in Kelvin                    */
#define THERM_BETA          3950.0f     /* B(25/50) [K]                      */
#define THERM_KELVIN0       273.15f

#define THERM_SAMPLES       5u          /* median filter window (odd)        */

/* idx (0..2) -> adc_ctrl channel */
static const AdcCtrl_Channel_t therm_ch[THERMISTOR_COUNT] =
{
	ADC_CH_THERM1,
	ADC_CH_THERM2,
	ADC_CH_THERM3,
};

void Thermistor_Init(void)
{
	/* ADC calibration / mode is handled centrally by AdcCtrl_Init(); nothing
	 * device-specific to do here. */
}

uint16_t Thermistor_ReadRaw(uint8_t idx)
{
	if (idx >= THERMISTOR_COUNT)
	{
		return 0;
	}
	return AdcCtrl_ReadMedian(therm_ch[idx], THERM_SAMPLES);
}

float Thermistor_ReadCelsius(uint8_t idx)
{
	uint16_t raw = Thermistor_ReadRaw(idx);

	/* raw == 0        -> read failed or probe pulled to GND (open NTC / 3.3V?)
	 * raw >= max-1    -> node at ~3.3V, NTC shorted -> Rntc ~ 0, log undefined */
	if (raw == 0 || raw >= (uint16_t)(THERM_ADC_MAX))
	{
		return NAN;
	}

	/* Ratiometric: Rntc = Rfixed * (ADC_MAX - raw) / raw. */
	float r_ntc = THERM_R_FIXED * ((THERM_ADC_MAX - (float)raw) / (float)raw);

	/* Beta equation. */
	float inv_t = (1.0f / THERM_T0_K) + (logf(r_ntc / THERM_R0) / THERM_BETA);
	float t_c   = (1.0f / inv_t) - THERM_KELVIN0;
	return t_c;
}

int16_t Thermistor_ReadCelsius_d10(uint8_t idx)
{
	float t_c = Thermistor_ReadCelsius(idx);
	if (isnan(t_c))
	{
		return THERMISTOR_ERR_D10;
	}

	/* Round to tenths of a degree. */
	float scaled = t_c * 10.0f;
	scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
	return (int16_t)scaled;
}
