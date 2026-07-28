#include "distance.h"
#include "adc_ctrl.h"

/* ---- Front-end / ADC scaling ------------------------------------------- */
#define DISTANCE_ADC_MAX        4095.0f   /* STM32F103 ADC = 12-bit          */
#define DISTANCE_VREF           3.3f      /* Vdda / ADC reference [V]         */
#define DISTANCE_DIV_RSERIES    1.0f      /* R51 [k] Vo -> PA0 (series)       */
#define DISTANCE_DIV_RSHUNT     2.7f      /* R52 [k] PA0 -> GND (shunt)       */
/* Recover sensor Vo from the pin voltage: Vo = Vpa0 * (Rs+Rsh)/Rsh          */
#define DISTANCE_DIV_GAIN  ((DISTANCE_DIV_RSERIES + DISTANCE_DIV_RSHUNT) / DISTANCE_DIV_RSHUNT)

/* ---- GP2Y0A41SK0F transfer curve (linear in 1/L, datasheet-fit) -------- */
#define DISTANCE_FIT_K          10.385f   /* L[cm] = K / (Vo - OFFSET)        */
#define DISTANCE_FIT_OFFSET     0.0539f
#define DISTANCE_MIN_CM         4.0f      /* datasheet valid range            */
#define DISTANCE_MAX_CM         30.0f

#define DISTANCE_SAMPLES        5u        /* median filter window (odd)       */

void Distance_Init(void)
{
	/* ADC calibration / mode is handled centrally by AdcCtrl_Init(); nothing
	 * device-specific to do here. Kept for API symmetry with the other devices. */
}

uint16_t Distance_ReadRaw(void)
{
	return AdcCtrl_ReadMedian(ADC_CH_DISTANCE, DISTANCE_SAMPLES);
}

float Distance_ReadVoltage(void)
{
	uint16_t raw = Distance_ReadRaw();
	if (raw == 0)
	{
		return -1.0f;
	}
	float v_pa0 = ((float)raw / DISTANCE_ADC_MAX) * DISTANCE_VREF;
	return v_pa0 * DISTANCE_DIV_GAIN;   /* recover sensor Vo */
}

float Distance_ReadCm(void)
{
	float vo = Distance_ReadVoltage();
	if (vo < 0.0f)
	{
		return -1.0f;
	}

	float denom = vo - DISTANCE_FIT_OFFSET;
	if (denom <= 0.0f)
	{
		return DISTANCE_MAX_CM;         /* object beyond range / no target */
	}

	float cm = DISTANCE_FIT_K / denom;
	if (cm < DISTANCE_MIN_CM) cm = DISTANCE_MIN_CM;
	if (cm > DISTANCE_MAX_CM) cm = DISTANCE_MAX_CM;
	return cm;
}

uint16_t Distance_ReadMm(void)
{
	float cm = Distance_ReadCm();
	if (cm < 0.0f)
	{
		return DISTANCE_ERR_MM;
	}
	return (uint16_t)(cm * 10.0f + 0.5f);
}

uint8_t Distance_FillFromMm(uint16_t mm)
{
	if (mm == DISTANCE_ERR_MM)       return 0;
	if (mm <= DISTANCE_BIN_FULL_MM)  return 100;
	if (mm >= DISTANCE_BIN_EMPTY_MM) return 0;

	/* Nearer target => fuller bin. */
	uint32_t span = (uint32_t)DISTANCE_BIN_EMPTY_MM - DISTANCE_BIN_FULL_MM;
	uint32_t fill = ((uint32_t)DISTANCE_BIN_EMPTY_MM - mm) * 100u / span;
	return (uint8_t)fill;
}

uint8_t Distance_ReadFillPercent(void)
{
	return Distance_FillFromMm(Distance_ReadMm());
}
