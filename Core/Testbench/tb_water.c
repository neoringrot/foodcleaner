#include "tb_water.h"

#include "gpio_ctrl.h"

/* Testbed for the clean-water fill path (WATER-ON supply + the WATER-SEN1
 * level sensor). See tb_water.h for the pin map, polarity note and the
 * single-owner (StartDefaultTask) constraint.
 *
 * Poll() mirrors tb_water_enable onto WATER-ON: enabled -> supply on and the
 * sensor is decoded (raw level, polarity-applied "present", and a
 * falling-edge count taken from the shared gpio_ctrl EXTI flags); disabled ->
 * supply forced off and the last snapshot is left untouched for inspection. */

/* Runtime switch -- set from the debugger while running. Default off/safe. */
volatile uint8_t tb_water_enable = 0;

/* Latest snapshot. */
volatile uint8_t tb_water_on_state = 0;

volatile uint8_t tb_water_sen1_level = 0;

volatile uint8_t tb_water_sen1_present = 0;
volatile uint8_t tb_water_present      = 0;

volatile uint32_t tb_water_sen1_events = 0;

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

	tb_water_sen1_present = 0;
	tb_water_present      = 0;

	tb_water_sen1_events = 0;

	tb_water_samples = 0;

	s_armed = 0;

	/* Start from a known-off state so the supply is never left energised. */
	gpio_ctrl_off(GPIO_OUT_WATER_ON);
}

void TB_Water_Poll(uint8_t tb_active)
{
	uint8_t l1;

	/* ★소유권 게이트(tb_heat 의 HT-POWER 와 같은 문제·같은 해법).
	 * WATER-ON(PE2)은 모음/동작 시나리오도 쓰는 핀이다. 시나리오는 MotorTask 에서
	 * 켜는데 이 함수는 defaultTask 에서 100ms 마다 돈다. 예전에는 게이트 없이
	 * 호출되어, tb_water_enable=0(기본값)일 때 아래 분기가 매 틱 PE2 를 꺼 버렸다.
	 * 실측: 모음 급수 단계에서 급수솔(PB13)은 ON 인데 급수메인(PE2)만 OFF 로
	 * 보였다 - 앱이 잘못 읽은 게 아니라 실제로 꺼져 있었다.
	 * 시나리오가 도는 동안에는 핀을 건드리지 않고 물러난다. */
	if (!tb_active)
	{
		s_armed = 0;          /* 벤치로 돌아오면 새 기준선부터 잡는다 */
		return;
	}

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
		s_armed = 1;
	}

	/* 1) Enable the water supply and read back the output latch. */
	gpio_ctrl_on(GPIO_OUT_WATER_ON);
	tb_water_on_state = gpio_ctrl_is_on(GPIO_OUT_WATER_ON);

	/* 2) Recognise the level sensor: raw level + polarity-applied present. */
	l1 = gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN1);

	tb_water_sen1_level = l1;

	tb_water_sen1_present = level_to_present(l1);
	tb_water_present = tb_water_sen1_present;

	/* 3) Count falling edges latched by the EXTI ISR, then clear each flag so
	 * the next edge is counted exactly once. */
	if (gpio_ctrl_exti_flag_get(GPIO_EXTI_WATER_SEN1))
	{
		tb_water_sen1_events++;
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WATER_SEN1);
	}

	tb_water_samples++;
}
