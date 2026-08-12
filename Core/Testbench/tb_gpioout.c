#include "tb_gpioout.h"

#include "gpio_ctrl.h"

/* Testbed for the four plain ON/OFF GPIO loads (drain valve + 3 fans). See
 * tb_gpioout.h for the contract. Each flag is a level: Poll() writes the pin to
 * match, so setting a flag from the debugger turns the load on/off live. */

/* Runtime switches -- set from the debugger while running. Default off/safe. */
volatile uint8_t tb_valve_drain_en = 0;
volatile uint8_t tb_valve_dry_en   = 0;
volatile uint8_t tb_fan_vapor_en   = 0;
volatile uint8_t tb_fan_exhaust_en = 0;
volatile uint8_t tb_bldc_fan_en    = 0;

void TB_GpioOut_Init(void)
{
	tb_valve_drain_en = 0;
	tb_valve_dry_en   = 0;
	tb_fan_vapor_en   = 0;
	tb_fan_exhaust_en = 0;
	tb_bldc_fan_en    = 0;

	/* Start from a known-off state so nothing is left energised by a prior run. */
	gpio_ctrl_off(GPIO_OUT_VALVE_DRAIN_CLN);
	gpio_ctrl_off(GPIO_OUT_VALVE_DRY_IN);
	gpio_ctrl_off(GPIO_OUT_FAN_VAPOR);
	gpio_ctrl_off(GPIO_OUT_FAN_EXHAUST);
	gpio_ctrl_off(GPIO_OUT_BLDC_FAN);
}

void TB_GpioOut_Poll(void)
{
	/* Mirror each flag onto its pin. gpio_ctrl_set is idempotent, so rewriting
	 * the same level every poll is harmless and keeps the code stateless. */
	gpio_ctrl_set(GPIO_OUT_VALVE_DRAIN_CLN, tb_valve_drain_en);
	gpio_ctrl_set(GPIO_OUT_VALVE_DRY_IN,    tb_valve_dry_en);
	gpio_ctrl_set(GPIO_OUT_FAN_VAPOR,       tb_fan_vapor_en);
	gpio_ctrl_set(GPIO_OUT_FAN_EXHAUST,     tb_fan_exhaust_en);
	gpio_ctrl_set(GPIO_OUT_BLDC_FAN,        tb_bldc_fan_en);
}
