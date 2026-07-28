#include "adc_ctrl.h"

/* High-impedance NTC dividers (~10k source) need a long sampling window; the
 * 71.5-cycle default is fine for the low-impedance distance divider too, but
 * 239.5 cycles keeps every channel accurate with margin. */
#define ADC_CTRL_SAMPLETIME     ADC_SAMPLETIME_239CYCLES_5
#define ADC_CTRL_TIMEOUT_MS     10u
#define ADC_CTRL_MAX_SAMPLES    15u

void AdcCtrl_Init(void)
{
	/* Collapse the CubeMX 4-rank scan into single-conversion mode so a plain
	 * polled read returns the requested channel. */
	hadc1.Init.ScanConvMode    = ADC_SCAN_DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	(void)HAL_ADC_Init(&hadc1);

	/* CubeMX omits calibration; the STM32F1 needs it for accurate results. */
	(void)HAL_ADCEx_Calibration_Start(&hadc1);
}

HAL_StatusTypeDef AdcCtrl_Read(AdcCtrl_Channel_t channel, uint16_t *out)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	HAL_StatusTypeDef st;

	sConfig.Channel      = (uint32_t)channel;
	sConfig.Rank         = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_CTRL_SAMPLETIME;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		return HAL_ERROR;
	}

	st = HAL_ADC_Start(&hadc1);
	if (st == HAL_OK)
	{
		st = HAL_ADC_PollForConversion(&hadc1, ADC_CTRL_TIMEOUT_MS);
		if (st == HAL_OK)
		{
			*out = (uint16_t)HAL_ADC_GetValue(&hadc1);
		}
	}
	HAL_ADC_Stop(&hadc1);
	return st;
}

uint16_t AdcCtrl_ReadMedian(AdcCtrl_Channel_t channel, uint8_t samples)
{
	uint16_t buf[ADC_CTRL_MAX_SAMPLES];
	uint8_t  n = 0;

	if (samples == 0)                    samples = 1;
	if (samples > ADC_CTRL_MAX_SAMPLES)  samples = ADC_CTRL_MAX_SAMPLES;

	for (uint8_t i = 0; i < samples; i++)
	{
		uint16_t v;
		if (AdcCtrl_Read(channel, &v) == HAL_OK)
		{
			buf[n++] = v;
		}
	}
	if (n == 0)
	{
		return 0;
	}

	/* Insertion sort, return the median. */
	for (uint8_t i = 1; i < n; i++)
	{
		uint16_t key = buf[i];
		int8_t   j   = (int8_t)i - 1;
		while (j >= 0 && buf[j] > key)
		{
			buf[j + 1] = buf[j];
			j--;
		}
		buf[j + 1] = key;
	}
	return buf[n / 2];
}
