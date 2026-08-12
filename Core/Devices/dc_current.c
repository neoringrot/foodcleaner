#include "dc_current.h"
#include "adc_ctrl.h"

/* ch -> adc_ctrl channel. See dc_current.h for the pin/op-amp/motor wiring. */
static const AdcCtrl_Channel_t s_ch[DC_CURRENT_COUNT] =
{
	[DC_CURR_LIFT]  = ADC_CH_DC_CURR1,   /* PC5 ADC1_IN15 U4->U6  lift       */
	[DC_CURR_TDOOR] = ADC_CH_DC_CURR2,   /* PB0 ADC1_IN8  U5->U9  trash door */
	[DC_CURR_WDOOR] = ADC_CH_DC_CURR3,   /* PB1 ADC1_IN9  U25->U11 water door*/
};

/* raw ADC code -> motor current [mA].
 *   I[mA] = raw * VREF_MV * 1000 / (ADC_MAX * shunt_mohm * gain)
 * Numerator up to 4095*3300*1000 ~= 1.35e10 exceeds 32-bit, so use uint64.
 * Denominator = 4095*10*2 = 81900. Clamp the (physically impossible) rail case
 * so the uint16 return never wraps. */
static uint16_t raw_to_ma(uint16_t raw)
{
	uint64_t num = (uint64_t)raw * DC_CURRENT_VREF_MV * 1000U;
	uint32_t den = (uint32_t)ADC_CTRL_ADC_MAX * DC_CURRENT_SHUNT_MOHM * DC_CURRENT_GAIN;
	uint32_t ma  = (uint32_t)(num / den);
	return (ma > 65535U) ? 65535U : (uint16_t)ma;
}

uint16_t DcCurrent_ReadRaw(DcCurrent_Ch_t ch)
{
	if (ch >= DC_CURRENT_COUNT)
		return 0U;
	return AdcCtrl_ReadMedian(s_ch[ch], DC_CURRENT_SAMPLES);
}

uint16_t DcCurrent_ReadMilliAmp(DcCurrent_Ch_t ch)
{
	return raw_to_ma(DcCurrent_ReadRaw(ch));
}

uint8_t DcCurrent_IsOver(DcCurrent_Ch_t ch)
{
	if (ch >= DC_CURRENT_COUNT)
		return 0U;
	return (DcCurrent_ReadMilliAmp(ch) >= DC_CURRENT_TRIP_MA) ? 1U : 0U;
}

uint8_t DcCurrent_UpdateAll(uint16_t out_ma[DC_CURRENT_COUNT])
{
	uint8_t mask = 0U;
	for (uint8_t ch = 0U; ch < DC_CURRENT_COUNT; ch++)
	{
		uint16_t ma = DcCurrent_ReadMilliAmp((DcCurrent_Ch_t)ch);
		if (out_ma != NULL)
			out_ma[ch] = ma;
		if (ma >= DC_CURRENT_TRIP_MA)
			mask = (uint8_t)(mask | (1U << ch));
	}
	return mask;
}
