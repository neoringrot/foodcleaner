/* -------------------------------------------------------------------------
 * gpio_ctrl.c - named GPIO access for the foodcleaner board.
 *
 * The tables below are the single source of truth mapping each enum in
 * gpio_ctrl.h to its (port, pin) pair. They are indexed directly by the enum
 * value via designated initializers, so entry order need not match but every
 * enumerator MUST appear - a missing one leaves a {NULL,0} hole. Each array is
 * sized [GPIO_*_COUNT], so adding an enumerator without a table entry is caught
 * as a zeroed slot at that index.
 * ---------------------------------------------------------------------- */

#include "gpio_ctrl.h"

typedef struct
{
	GPIO_TypeDef *port;
	uint16_t      pin;
} gpio_map_t;

/* Output pin table - one entry per gpio_out_t enumerator. */
static const gpio_map_t out_map[GPIO_OUT_COUNT] =
{
	[GPIO_OUT_WATER_ON]        = { o_WATER_ON_GPIO_Port,        o_WATER_ON_Pin        },
	[GPIO_OUT_EN_DOOR_TRASH]   = { o_EN_DOOR_TRASH_GPIO_Port,   o_EN_DOOR_TRASH_Pin   },
	[GPIO_OUT_EN_DOOR_WATER]   = { o_EN_DOOR_WATER_GPIO_Port,   o_EN_DOOR_WATER_Pin   },
	[GPIO_OUT_EN_SPK]          = { o_EN_SPK_GPIO_Port,          o_EN_SPK_Pin          },
	[GPIO_OUT_HT_POWER]        = { o_HT_POWER_GPIO_Port,        o_HT_POWER_Pin        },
	[GPIO_OUT_SPI1_EEPROM_CS]  = { o_SPI1_EEPROM_CS_GPIO_Port,  o_SPI1_EEPROM_CS_Pin  },
	[GPIO_OUT_M1_ENABLE]       = { o_M1_ENABLE_GPIO_Port,       o_M1_ENABLE_Pin       },
	[GPIO_OUT_M2_ENABLE]       = { o_M2_ENABLE_GPIO_Port,       o_M2_ENABLE_Pin       },
	[GPIO_OUT_M1_DIR]          = { o_M1_DIR_GPIO_Port,          o_M1_DIR_Pin          },
	[GPIO_OUT_M2_DIR]          = { o_M2_DIR_GPIO_Port,          o_M2_DIR_Pin          },
	[GPIO_OUT_M1_nBRAKE]       = { o_M1_nBRAKE_GPIO_Port,       o_M1_nBRAKE_Pin       },
	[GPIO_OUT_M2_nBRAKE]       = { o_M2_nBRAKE_GPIO_Port,       o_M2_nBRAKE_Pin       },
	[GPIO_OUT_FRAME_WP]        = { o_FRAME_WP_GPIO_Port,        o_FRAME_WP_Pin        },
	[GPIO_OUT_MTR_DC_LIFT]     = { o_MTR_DC_LIFT_GPIO_Port,     o_MTR_DC_LIFT_Pin     },
	[GPIO_OUT_VALVE_DRY_IN]    = { o_VALVE_DRY_IN_GPIO_Port,    o_VALVE_DRY_IN_Pin    },
	[GPIO_OUT_VALVE_DRAIN_CLN] = { o_VALVE_DRAIN_CLN_GPIO_Port, o_VALVE_DRAIN_CLN_Pin },
	[GPIO_OUT_FAN_VAPOR]       = { o_FAN_VAPOR_GPIO_Port,       o_FAN_VAPOR_Pin       },
	[GPIO_OUT_STEP1_M1]        = { o_STEP1_M1_GPIO_Port,        o_STEP1_M1_Pin        },
	[GPIO_OUT_STEP1_M2]        = { o_STEP1_M2_GPIO_Port,        o_STEP1_M2_Pin        },
	[GPIO_OUT_STEP1_M3]        = { o_STEP1_M3_GPIO_Port,        o_STEP1_M3_Pin        },
	[GPIO_OUT_STEP1_M4]        = { o_STEP1_M4_GPIO_Port,        o_STEP1_M4_Pin        },
	[GPIO_OUT_STEP2_M1]        = { o_STEP2_M1_GPIO_Port,        o_STEP2_M1_Pin        },
	[GPIO_OUT_STEP2_M2]        = { o_STEP2_M2_GPIO_Port,        o_STEP2_M2_Pin        },
	[GPIO_OUT_STEP2_M3]        = { o_STEP2_M3_GPIO_Port,        o_STEP2_M3_Pin        },
	[GPIO_OUT_STEP2_M4]        = { o_STEP2_M4_GPIO_Port,        o_STEP2_M4_Pin        },
	[GPIO_OUT_BLE_MODE]        = { o_BLE_MODE_GPIO_Port,        o_BLE_MODE_Pin        },
	[GPIO_OUT_FAN_EXHAUST]     = { o_FAN_EXHAUST_GPIO_Port,     o_FAN_EXHAUST_Pin     },
	[GPIO_OUT_LIFT_IN1]        = { o_LIFT_IN1_GPIO_Port,        o_LIFT_IN1_Pin        },
	[GPIO_OUT_LIFT_IN2]        = { o_LIFT_IN2_GPIO_Port,        o_LIFT_IN2_Pin        },
};

/* Input pin table - one entry per gpio_in_t enumerator. */
static const gpio_map_t in_map[GPIO_IN_COUNT] =
{
	[GPIO_IN_BLE_STATUS] = { i_BLE_STATUS_GPIO_Port, i_BLE_STATUS_Pin },
};

/* EXTI pin table - one entry per gpio_exti_t enumerator. */
static const gpio_map_t exti_map[GPIO_EXTI_COUNT] =
{
	[GPIO_EXTI_BIMETAL1]   = { exti0_BIMETAL1_GPIO_Port,  exti0_BIMETAL1_Pin  },
	[GPIO_EXTI_BIMETAL2]   = { exti1_BIMETAL2_GPIO_Port,  exti1_BIMETAL2_Pin  },
	[GPIO_EXTI_BIMETAL3]   = { exti2_BIMETAL3_GPIO_Port,  exti2_BIMETAL3_Pin  },
	[GPIO_EXTI_BIMETAL4]   = { exti3_BIMETAL4_GPIO_Port,  exti3_BIMETAL4_Pin  },
	[GPIO_EXTI_BIMETAL5]   = { exti4_BIMETAL5_GPIO_Port,  exti4_BIMETAL5_Pin  },
	[GPIO_EXTI_LEAD_SW]    = { exti5_LEAD_SW_GPIO_Port,   exti5_LEAD_SW_Pin   },
	[GPIO_EXTI_WATER_SEN1] = { exti6_WATER_SEN1_GPIO_Port,exti6_WATER_SEN1_Pin},
	[GPIO_EXTI_WATER_SEN2] = { exti7_WATER_SEN2_GPIO_Port,exti7_WATER_SEN2_Pin},
	[GPIO_EXTI_TIMER_OUT]  = { exti11_TIMER_OUT_GPIO_Port,exti11_TIMER_OUT_Pin},
	[GPIO_EXTI_M2_FGOT]    = { exti12_M2_FGOT_GPIO_Port,  exti12_M2_FGOT_Pin  },
	[GPIO_EXTI_M2_nFAULT]  = { exti13_M2_nFAULT_GPIO_Port,exti13_M2_nFAULT_Pin},
	[GPIO_EXTI_M1_nFAULT]  = { exti14_M1_nFAULT_GPIO_Port,exti14_M1_nFAULT_Pin},
	[GPIO_EXTI_M1_FGOT]    = { exti15_M1_FGOT_GPIO_Port,  exti15_M1_FGOT_Pin  },
};

/* Per-EXTI-pin interrupt flags. Set in ISR context by gpio_ctrl_exti_dispatch()
 * on each falling edge, cleared by application code. `volatile` because they
 * are written from an interrupt and read from thread context. Not consumed
 * anywhere yet - reserved for future edge handling. */
static volatile uint8_t exti_flag[GPIO_EXTI_COUNT];

/* ---- Output control -------------------------------------------------- */

void gpio_ctrl_on(gpio_out_t out)
{
	if (out >= GPIO_OUT_COUNT) return;
	HAL_GPIO_WritePin(out_map[out].port, out_map[out].pin, GPIO_PIN_SET);
}

void gpio_ctrl_off(gpio_out_t out)
{
	if (out >= GPIO_OUT_COUNT) return;
	HAL_GPIO_WritePin(out_map[out].port, out_map[out].pin, GPIO_PIN_RESET);
}

void gpio_ctrl_enable(gpio_out_t out)
{
	gpio_ctrl_on(out);
}

void gpio_ctrl_disable(gpio_out_t out)
{
	gpio_ctrl_off(out);
}

void gpio_ctrl_set(gpio_out_t out, uint8_t on)
{
	if (out >= GPIO_OUT_COUNT) return;
	HAL_GPIO_WritePin(out_map[out].port, out_map[out].pin,
	                  on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void gpio_ctrl_toggle(gpio_out_t out)
{
	if (out >= GPIO_OUT_COUNT) return;
	HAL_GPIO_TogglePin(out_map[out].port, out_map[out].pin);
}

uint8_t gpio_ctrl_is_on(gpio_out_t out)
{
	if (out >= GPIO_OUT_COUNT) return 0;
	/* Read-back the output data latch, not the input register. */
	return (out_map[out].port->ODR & out_map[out].pin) ? 1U : 0U;
}

/* ---- Input read ------------------------------------------------------- */

uint8_t gpio_ctrl_read(gpio_in_t in)
{
	if (in >= GPIO_IN_COUNT) return 0;
	return (HAL_GPIO_ReadPin(in_map[in].port, in_map[in].pin) == GPIO_PIN_SET)
	       ? 1U : 0U;
}

/* ---- EXTI: level read + interrupt flags ------------------------------ */

uint8_t gpio_ctrl_exti_read(gpio_exti_t e)
{
	if (e >= GPIO_EXTI_COUNT) return 0;
	return (HAL_GPIO_ReadPin(exti_map[e].port, exti_map[e].pin) == GPIO_PIN_SET)
	       ? 1U : 0U;
}

void gpio_ctrl_exti_dispatch(uint16_t gpio_pin)
{
	/* Runs in EXTI ISR context. Each EXTI line is driven by exactly one pin
	 * number on this board, so the pin mask alone identifies the source. */
	for (gpio_exti_t e = 0; e < GPIO_EXTI_COUNT; e++)
	{
		if (exti_map[e].pin == gpio_pin)
		{
			exti_flag[e] = 1U;
			return;
		}
	}
}

uint8_t gpio_ctrl_exti_flag_get(gpio_exti_t e)
{
	if (e >= GPIO_EXTI_COUNT) return 0;
	return exti_flag[e];
}

void gpio_ctrl_exti_flag_clear(gpio_exti_t e)
{
	if (e >= GPIO_EXTI_COUNT) return;
	exti_flag[e] = 0U;
}

void gpio_ctrl_exti_flag_clear_all(void)
{
	for (gpio_exti_t e = 0; e < GPIO_EXTI_COUNT; e++)
	{
		exti_flag[e] = 0U;
	}
}
