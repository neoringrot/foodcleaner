#include "thermistor.h"
#include "adc_ctrl.h"
#include <math.h>

/* ---- NTC / divider constants ------------------------------------------- */
#define THERM_ADC_MAX       4095.0f     /* 12-bit full scale                 */
#define THERM_R_FIXED       10000.0f    /* R107/R111/R109 pull-down [ohm]    */
#define THERM_KELVIN0       273.15f
/* NOTE: bench-confirmed the fixed divider leg is 10k (R107/R111/R109), NOT the
 * 1k shown on the earlier BOM read. With a 10k NTC this puts 25C at mid-scale
 * (raw~2048), so room temp is well resolved but HIGH-temp resolution is poor:
 * only ~46 counts span 150..200C (~0.7 count/C at 200C, raw saturates toward
 * 4095). This is a hardware trait of the divider, not the maths below. */

#define THERM_SAMPLES       5u          /* median filter window (odd)        */

/* ---- Smoothing (first-order complementary / EMA low-pass) ---------------
 * On top of the per-read 5-sample median above, Thermistor_Tick() folds each
 * new sample into a running value:  filt = filt*(1-a) + sample*a. This is the
 * "new = old*0.7 + input*0.3" idea (a = g_therm_filter_alpha, default 0.3).
 *
 * WHY EMA (not a 100-sample moving average): O(1) state -- one float per channel
 * instead of a 100-deep ring per channel -- no buffer bookkeeping, tunable, and
 * for equal noise rejection it lags less. At the 100 ms tick, a=0.3 gives a time
 * constant ~0.23 s (95% of a step in ~0.8 s): plenty fast for a thermal load,
 * yet it halves the sample-to-sample jitter that the coarse high-temp divider
 * resolution would otherwise show. Lower a = smoother/slower, higher a = livelier. */
#define THERM_FILTER_ALPHA_DEFAULT  0.3f
#define THERM_ERR_LIMIT             5u  /* consecutive bad reads before EMA -> error */

/* ---- R-T lookup table (supersedes the single-Beta equation) ------------
 * Source: Adafruit "103_3950" 10K NTC lookup table (R25=10K, B25/50=3950),
 * https://cdn-shop.adafruit.com/datasheets/103_3950_lookuptable.pdf -- same
 * sensor family as the HCET-103F3950 fitted here (datasheet/온도센서.pdf:
 * R25=10K +/-1%, B25/50=3950 +/-1%).
 *
 * WHY A TABLE: the old single-Beta model (T0=25C, B=3950) is only accurate in
 * the 20..60C band it is fitted to; measured against this table it OVER-reads
 * by ~+1.2C at 100C, ~+5.6C at 150C and ~+10.7C at 200C -- unacceptable for the
 * dryer/heater temperatures the 동작 scenario measures. Interpolating the real
 * curve holds the whole -40..200C range to ~0.1C.
 *
 * INTERPOLATION: piecewise-linear in (ln R, 1/T[K]) -- i.e. a per-segment Beta
 * fit -- which is near-exact even at the 10C row spacing below (25C is NOT a
 * table row yet interpolates to 24.98C between the 20C/30C rows). Rows are T
 * ascending / R descending; R in ohms. */
typedef struct { int16_t t_c; float r_ohm; } therm_rt_t;
static const therm_rt_t THERM_RT[] =
{
	{ -40, 277200.0f }, { -30, 157200.0f }, { -20, 87430.0f }, { -10, 51820.0f },
	{   0,  31770.0f }, {  10,  19680.0f }, {  20, 12470.0f }, {  30,  8064.0f },
	{  40,   5327.0f }, {  50,   3592.0f }, {  60,  2472.0f }, {  70,  1735.0f },
	{  80,   1243.0f }, {  90,    908.3f }, { 100,  674.4f }, { 110,   508.3f },
	{ 120,    383.5f }, { 130,    292.4f }, { 140,  225.8f }, { 150,   176.9f },
	{ 160,    141.0f }, { 170,    113.9f }, { 180,   92.8f }, { 190,    75.9f },
	{ 200,     61.9f },
};
#define THERM_RT_N  (sizeof(THERM_RT) / sizeof(THERM_RT[0]))

/* idx (0..2) -> adc_ctrl channel */
static const AdcCtrl_Channel_t therm_ch[THERMISTOR_COUNT] =
{
	ADC_CH_THERM1,
	ADC_CH_THERM2,
	ADC_CH_THERM3,
};

/* EMA smoothing weight of the NEWEST sample (0..1). Exposed so it can be tuned
 * live from the debugger while hunting a good value. */
volatile float g_therm_filter_alpha = THERM_FILTER_ALPHA_DEFAULT;

/* Per-channel filter state, updated by Thermistor_Tick(). */
static float    s_filt_c[THERMISTOR_COUNT];   /* running EMA of Celsius        */
static uint8_t  s_filt_valid[THERMISTOR_COUNT];/* 0 until first good sample     */
static uint8_t  s_err_cnt[THERMISTOR_COUNT];  /* consecutive bad-read counter  */
static uint16_t s_last_raw[THERMISTOR_COUNT]; /* raw from the last Tick        */

void Thermistor_Init(void)
{
	uint8_t i;

	/* ADC calibration / mode is handled centrally by AdcCtrl_Init(); nothing
	 * device-specific to do here except clear the EMA state so the first Tick
	 * seeds cleanly instead of ramping up from a stale value. */
	for (i = 0u; i < THERMISTOR_COUNT; i++)
	{
		s_filt_c[i]      = NAN;
		s_filt_valid[i]  = 0u;
		s_err_cnt[i]     = 0u;
		s_last_raw[i]    = 0u;
	}
}

uint16_t Thermistor_ReadRaw(uint8_t idx)
{
	if (idx >= THERMISTOR_COUNT)
	{
		return 0;
	}
	return AdcCtrl_ReadMedian(therm_ch[idx], THERM_SAMPLES);
}

/* Convert an NTC resistance [ohm] to Celsius via the R-T table above,
 * interpolating piecewise-linearly in (ln R, 1/T[K]) -- locally a per-segment
 * Beta fit, so it is near-exact between rows. Returns NAN when R falls off
 * either end of the table (colder than -40C / hotter than 200C), which for
 * this divider corresponds to an open or shorted probe. */
static float therm_r_to_c(float r_ntc)
{
	uint8_t i;

	/* Off the ends. R is DESCENDING, so > first row = too cold, < last = too hot.
	 * Endpoints themselves are in range (strict compare), so exactly -40C / 200C
	 * still resolve instead of reading as a fault. */
	if (r_ntc > THERM_RT[0].r_ohm || r_ntc < THERM_RT[THERM_RT_N - 1u].r_ohm)
	{
		return NAN;
	}

	/* Bracket: first i with r_ntc >= R[i+1] gives R[i] > r_ntc >= R[i+1]. */
	for (i = 0u; i < (THERM_RT_N - 1u); i++)
	{
		if (r_ntc >= THERM_RT[i + 1u].r_ohm)
		{
			break;
		}
	}

	{
		float x  = logf(r_ntc);
		float x0 = logf(THERM_RT[i].r_ohm);
		float x1 = logf(THERM_RT[i + 1u].r_ohm);
		float y0 = 1.0f / ((float)THERM_RT[i].t_c      + THERM_KELVIN0);
		float y1 = 1.0f / ((float)THERM_RT[i + 1u].t_c + THERM_KELVIN0);
		float inv_t = y0 + (x - x0) * (y1 - y0) / (x1 - x0);
		return (1.0f / inv_t) - THERM_KELVIN0;
	}
}

/* Convert one raw ADC count to Celsius (guard + ratiometric Rntc + table). */
static float therm_raw_to_c(uint16_t raw)
{
	/* raw == 0        -> read failed or probe pulled to GND (open NTC / 3.3V?)
	 * raw >= max-1    -> node at ~3.3V, NTC shorted -> Rntc ~ 0, log undefined
	 * Guard here to avoid forming Rntc with a zero denominator. */
	if (raw == 0 || raw >= (uint16_t)(THERM_ADC_MAX))
	{
		return NAN;
	}

	/* Ratiometric: Rntc = Rfixed * (ADC_MAX - raw) / raw. */
	{
		float r_ntc = THERM_R_FIXED * ((THERM_ADC_MAX - (float)raw) / (float)raw);
		return therm_r_to_c(r_ntc);   /* R-T table lookup */
	}
}

/* Round Celsius to signed tenths, or the error sentinel when NAN. */
static int16_t therm_c_to_d10(float t_c)
{
	if (isnan(t_c))
	{
		return THERMISTOR_ERR_D10;
	}
	{
		float scaled = t_c * 10.0f;
		scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
		return (int16_t)scaled;
	}
}

/* ---- Instantaneous (unfiltered) reads ---------------------------------- */
float Thermistor_ReadCelsius(uint8_t idx)
{
	return therm_raw_to_c(Thermistor_ReadRaw(idx));
}

int16_t Thermistor_ReadCelsius_d10(uint8_t idx)
{
	return therm_c_to_d10(Thermistor_ReadCelsius(idx));
}

/* ---- Smoothed reads (EMA) ---------------------------------------------- */
void Thermistor_Tick(void)
{
	/* Effective alpha, clamped to a sane (0,1] so a bad live edit can't freeze
	 * the filter (a<=0) or overflow it (a>1). */
	float a = g_therm_filter_alpha;
	uint8_t i;

	if (!(a > 0.0f)) { a = THERM_FILTER_ALPHA_DEFAULT; }
	if (a > 1.0f)    { a = 1.0f; }

	for (i = 0u; i < THERMISTOR_COUNT; i++)
	{
		uint16_t raw = Thermistor_ReadRaw(i);
		float    t   = therm_raw_to_c(raw);

		s_last_raw[i] = raw;

		if (isnan(t))
		{
			/* Ride out brief glitches (hold last value); only report an error
			 * once the fault persists for THERM_ERR_LIMIT ticks. */
			if (s_err_cnt[i] < THERM_ERR_LIMIT)
			{
				s_err_cnt[i]++;
			}
			if (s_err_cnt[i] >= THERM_ERR_LIMIT)
			{
				s_filt_valid[i] = 0u;
				s_filt_c[i]     = NAN;
			}
			continue;
		}

		s_err_cnt[i] = 0u;
		if (!s_filt_valid[i])
		{
			s_filt_c[i]     = t;      /* seed on first good sample (no ramp) */
			s_filt_valid[i] = 1u;
		}
		else
		{
			s_filt_c[i] = s_filt_c[i] * (1.0f - a) + t * a;   /* EMA */
		}
	}
}

float Thermistor_GetCelsius(uint8_t idx)
{
	if (idx >= THERMISTOR_COUNT || !s_filt_valid[idx])
	{
		return NAN;
	}
	return s_filt_c[idx];
}

int16_t Thermistor_GetCelsius_d10(uint8_t idx)
{
	return therm_c_to_d10(Thermistor_GetCelsius(idx));
}

uint16_t Thermistor_GetRaw(uint8_t idx)
{
	if (idx >= THERMISTOR_COUNT)
	{
		return 0;
	}
	return s_last_raw[idx];
}
