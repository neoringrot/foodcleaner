#include "tca9554.h"

/* TI TCA9554A 8-bit I2C I/O expander - register-level driver.
 * See tca9554.h for the wiring, address map and register model.
 *
 * All bus traffic uses the TCA9554 "command byte" protocol, which maps 1:1
 * onto HAL's memory-access API: the command byte (register index 0..3) is the
 * "memory address", so HAL_I2C_Mem_Read/Write do the whole transaction. */

#define TCA9554_I2C_TIMEOUT   100U   /* ms, blocking bus access */

/* ---------------------------------------------------------------------- */
/* Low-level register access                                               */
/* ---------------------------------------------------------------------- */
HAL_StatusTypeDef TCA9554_WriteReg(tca9554_t *dev, uint8_t reg, uint8_t val)
{
	if (dev == NULL || dev->hi2c == NULL)
		return HAL_ERROR;

	return HAL_I2C_Mem_Write(dev->hi2c,
	                         (uint16_t)(dev->addr7 << 1),
	                         reg, I2C_MEMADD_SIZE_8BIT,
	                         &val, 1, TCA9554_I2C_TIMEOUT);
}

HAL_StatusTypeDef TCA9554_ReadReg(tca9554_t *dev, uint8_t reg, uint8_t *val)
{
	if (dev == NULL || dev->hi2c == NULL || val == NULL)
		return HAL_ERROR;

	return HAL_I2C_Mem_Read(dev->hi2c,
	                        (uint16_t)(dev->addr7 << 1),
	                        reg, I2C_MEMADD_SIZE_8BIT,
	                        val, 1, TCA9554_I2C_TIMEOUT);
}

/* ---------------------------------------------------------------------- */
/* Lifecycle                                                               */
/* ---------------------------------------------------------------------- */
HAL_StatusTypeDef TCA9554_Init(tca9554_t *dev, I2C_HandleTypeDef *hi2c,
                               uint8_t addr7, uint8_t config, uint8_t out_init)
{
	HAL_StatusTypeDef st;

	if (dev == NULL || hi2c == NULL)
		return HAL_ERROR;

	dev->hi2c       = hi2c;
	dev->addr7      = addr7;
	dev->cfg_shadow = config;
	dev->out_shadow = out_init;

	/* Drive the output latch *before* flipping pins to outputs, so no pin
	 * glitches through the power-on-default (0xFF) on the way to out_init. */
	st = TCA9554_WriteReg(dev, TCA9554_REG_OUTPUT, out_init);
	if (st != HAL_OK)
		return st;

	/* Clear polarity inversion (default), then set pin directions.
	 * config bit = 1 -> input (Hi-Z), 0 -> output. */
	st = TCA9554_WriteReg(dev, TCA9554_REG_POLARITY, 0x00);
	if (st != HAL_OK)
		return st;

	return TCA9554_WriteReg(dev, TCA9554_REG_CONFIG, config);
}

uint8_t TCA9554_IsPresent(tca9554_t *dev)
{
	if (dev == NULL || dev->hi2c == NULL)
		return 0;

	return (HAL_I2C_IsDeviceReady(dev->hi2c, (uint16_t)(dev->addr7 << 1),
	                              2, TCA9554_I2C_TIMEOUT) == HAL_OK) ? 1U : 0U;
}

/* ---------------------------------------------------------------------- */
/* Direction / polarity                                                    */
/* ---------------------------------------------------------------------- */
HAL_StatusTypeDef TCA9554_SetConfig(tca9554_t *dev, uint8_t config)
{
	HAL_StatusTypeDef st = TCA9554_WriteReg(dev, TCA9554_REG_CONFIG, config);
	if (st == HAL_OK && dev != NULL)
		dev->cfg_shadow = config;
	return st;
}

HAL_StatusTypeDef TCA9554_SetPolarity(tca9554_t *dev, uint8_t polarity)
{
	return TCA9554_WriteReg(dev, TCA9554_REG_POLARITY, polarity);
}

/* ---------------------------------------------------------------------- */
/* Port-level output                                                       */
/* ---------------------------------------------------------------------- */
HAL_StatusTypeDef TCA9554_WriteOutput(tca9554_t *dev, uint8_t val)
{
	HAL_StatusTypeDef st = TCA9554_WriteReg(dev, TCA9554_REG_OUTPUT, val);
	if (st == HAL_OK && dev != NULL)
		dev->out_shadow = val;
	return st;
}

uint8_t TCA9554_GetOutput(const tca9554_t *dev)
{
	return (dev != NULL) ? dev->out_shadow : 0U;
}

HAL_StatusTypeDef TCA9554_WritePin(tca9554_t *dev, uint8_t pin, uint8_t level)
{
	uint8_t val;

	if (dev == NULL || pin > 7U)
		return HAL_ERROR;

	val = dev->out_shadow;
	if (level)
		val |= (uint8_t)(1U << pin);
	else
		val &= (uint8_t)~(1U << pin);

	return TCA9554_WriteOutput(dev, val);
}

HAL_StatusTypeDef TCA9554_TogglePin(tca9554_t *dev, uint8_t pin)
{
	if (dev == NULL || pin > 7U)
		return HAL_ERROR;

	return TCA9554_WriteOutput(dev, (uint8_t)(dev->out_shadow ^ (1U << pin)));
}

/* ---------------------------------------------------------------------- */
/* Port-level input                                                        */
/* ---------------------------------------------------------------------- */
HAL_StatusTypeDef TCA9554_ReadInput(tca9554_t *dev, uint8_t *val)
{
	return TCA9554_ReadReg(dev, TCA9554_REG_INPUT, val);
}
