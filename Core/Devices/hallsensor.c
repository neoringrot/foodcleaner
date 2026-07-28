#include "hallsensor.h"

/* 7 Hall sensors on the U10 TCA9554A (I2C1). See hallsensor.h for wiring,
 * address and polarity notes. */

#define HALLSENSOR_MASK   0x7FU   /* HS1..HS7 = P0..P6; P7 unused, masked off */

static tca9554_t s_dev;           /* U10 - HS inputs */
static uint8_t   s_mask;          /* last detected (polarity-applied) mask */

HAL_StatusTypeDef HallSensor_Init(void)
{
	HAL_StatusTypeDef st;

	s_mask = 0x00U;

	/* U10: all pins inputs (HS1..7 + unused P7). */
	st = TCA9554_Init(&s_dev, &hi2c1, TCA9554_U10_ADDR,
	                  TCA9554_ALL_INPUTS, 0x00U);
	if (st != HAL_OK)
		return st;

	/* Active-low sensors: invert the input readings in hardware so a detected
	 * magnet reads as 1 and the rest of the code stays polarity-agnostic. */
#if !HALLSENSOR_ACTIVE_HIGH
	st = TCA9554_SetPolarity(&s_dev, HALLSENSOR_MASK);
	if (st != HAL_OK)
		return st;
#endif

	/* Seed the cache from the current state. */
	(void)HallSensor_Update();
	return HAL_OK;
}

uint8_t HallSensor_IsPresent(void)
{
	return TCA9554_IsPresent(&s_dev);
}

HAL_StatusTypeDef HallSensor_Read(uint8_t *mask)
{
	uint8_t raw;
	HAL_StatusTypeDef st;

	if (mask == NULL)
		return HAL_ERROR;

	st = TCA9554_ReadInput(&s_dev, &raw);
	if (st != HAL_OK)
		return st;

	/* Polarity is handled in HW via the inversion register, so raw already
	 * reads 1 = detected. Just strip the unused P7 bit. */
	*mask = (uint8_t)(raw & HALLSENSOR_MASK);
	return HAL_OK;
}

HAL_StatusTypeDef HallSensor_ReadRaw(uint8_t *mask)
{
	uint8_t raw;
	HAL_StatusTypeDef st;

	if (mask == NULL)
		return HAL_ERROR;

	st = TCA9554_ReadReg(&s_dev, TCA9554_REG_INPUT, &raw);
	if (st != HAL_OK)
		return st;

	*mask = (uint8_t)(raw & HALLSENSOR_MASK);
	return HAL_OK;
}

HAL_StatusTypeDef HallSensor_Update(void)
{
	uint8_t m;
	HAL_StatusTypeDef st = HallSensor_Read(&m);
	if (st == HAL_OK)
		s_mask = m;
	return st;
}

uint8_t HallSensor_GetMask(void)
{
	return s_mask;
}

uint8_t HallSensor_Get(uint8_t idx)
{
	if (idx >= HALLSENSOR_COUNT)
		return 0U;
	return (uint8_t)((s_mask >> idx) & 1U);
}
