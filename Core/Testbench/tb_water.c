#include "tb_water.h"

#include "gpio_ctrl.h"

/* Testbed for the clean-water fill path (WATER-ON supply + WATER-SEN1/SEN2
 * level sensors). See tb_water.h for the pin map, polarity note and the
 * single-owner (StartDefaultTask) constraint.
 *
 * Poll() mirrors tb_water_enable onto WATER-ON: enabled -> supply on and the
 * two sensors are decoded (raw level, polarity-applied "present", and a
 * falling-edge count taken from the shared gpio_ctrl EXTI flags); disabled ->
 * supply forced off and the last snapshot is left untouched for inspection. */

/* Runtime switch -- set from the debugger while running. Default off/safe. */
volatile uint8_t tb_water_enable = 0;

/* Latest snapshot. */
volatile uint8_t tb_water_on_state = 0;

volatile uint8_t tb_water_sen1_level = 0;
volatile uint8_t tb_water_sen2_level = 0;

volatile uint8_t tb_water_sen1_present = 0;
volatile uint8_t tb_water_sen2_present = 0;
volatile uint8_t tb_water_present      = 0;

volatile uint32_t tb_water_sen1_events = 0;
volatile uint32_t tb_water_sen2_events = 0;

volatile uint32_t tb_water_samples = 0;

/* 0 until the enable edge arms monitoring; used to clear stale EXTI flags once
 * when the testbed turns on so a pre-existing level is not counted as an edge. */
static uint8_t s_armed;

/* Map a raw pin level to logical "water present" using the configured polarity. */
static uint8_t level_to_present(uint8_t level)
{
#if TB_WATER_ACTIVE_LOW
	return (uint8_t)(level == 0U);
#else
	return (uint8_t)(level != 0U);
#endif
}

void TB_Water_Init(void)
{
	tb_water_enable = 0;

	tb_water_on_state = 0;

	tb_water_sen1_level = 0;
	tb_water_sen2_level = 0;

	tb_water_sen1_present = 0;
	tb_water_sen2_present = 0;
	tb_water_present      = 0;

	tb_water_sen1_events = 0;
	tb_water_sen2_events = 0;

	tb_water_samples = 0;

	s_armed = 0;

	/* Start from a known-off state so the supply is never left energised. */
	gpio_ctrl_off(GPIO_OUT_WATER_ON);
}

void TB_Water_Poll(void)
{
	uint8_t l1;
	uint8_t l2;

	/* Disabled: shut the supply off and stop monitoring. Keep the last snapshot
	 * so it can still be inspected, and drop the armed state so re-enabling
	 * seeds a fresh baseline instead of firing spurious edges. */
	if (!tb_water_enable)
	{
		gpio_ctrl_off(GPIO_OUT_WATER_ON);
		tb_water_on_state = 0;
		s_armed = 0;
		return;
	}

	/* First enabled poll only: clear any edge latched before the testbed took
	 * over so a level already asserted at enable time is not miscounted. */
	if (!s_armed)
	{
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WATER_SEN1);
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WATER_SEN2);
		s_armed = 1;
	}

	/* 1) Enable the water supply and read back the output latch. */
	gpio_ctrl_on(GPIO_OUT_WATER_ON);
	tb_water_on_state = gpio_ctrl_is_on(GPIO_OUT_WATER_ON);

	/* 2) Recognise the two level sensors: raw level + polarity-applied present. */
	l1 = gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN1);
	l2 = gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN2);

	tb_water_sen1_level = l1;
	tb_water_sen2_level = l2;

	tb_water_sen1_present = level_to_present(l1);
	tb_water_sen2_present = level_to_present(l2);
	tb_water_present = (uint8_t)(tb_water_sen1_present || tb_water_sen2_present);

	/* 3) Count falling edges latched by the EXTI ISR, then clear each flag so
	 * the next edge is counted exactly once. */
	if (gpio_ctrl_exti_flag_get(GPIO_EXTI_WATER_SEN1))
	{
		tb_water_sen1_events++;
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WATER_SEN1);
	}
	if (gpio_ctrl_exti_flag_get(GPIO_EXTI_WATER_SEN2))
	{
		tb_water_sen2_events++;
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WATER_SEN2);
	}

	tb_water_samples++;
}
