#include "tb_doorhall.h"

#include "gpio_ctrl.h"
#include "wdoor.h"   /* WDoor_AtOpen/AtClose - reuse the production decode */
#include "tdoor.h"   /* TDoor_AtOpen/AtClose */

/* Testbed for the four door-limit Hall sensors (WHALL/THALL open/close). See
 * tb_doorhall.h for the pin map and the single-owner (StartDefaultTask)
 * constraint.
 *
 * Split by nature of the signal:
 *   Poll() (~100 ms) reads raw levels for diagnostics and decodes at-limit via
 *   the same driver functions the scenario uses (so the tb never disagrees with
 *   the firmware). Levels stay polled on purpose - they are what you watch at
 *   runtime. Disabled -> the last snapshot is left untouched.
 *
 *   OnEXTI() runs in EXTI ISR context on every falling edge and counts/stamps it
 *   there, so bursts faster than the poll period are not merged. Edges arriving
 *   while disabled are dropped outright.
 *
 * Never drives an actuator. */

/* Runtime switch -- set from the debugger while running. Default off. */
volatile uint8_t tb_doorhall_enable = 0;

volatile uint8_t tb_whall_close_level = 0;
volatile uint8_t tb_whall_open_level  = 0;
volatile uint8_t tb_thall_close_level = 0;
volatile uint8_t tb_thall_open_level  = 0;

volatile uint8_t tb_wdoor_at_close = 0;
volatile uint8_t tb_wdoor_at_open  = 0;
volatile uint8_t tb_tdoor_at_close = 0;
volatile uint8_t tb_tdoor_at_open  = 0;

volatile uint32_t tb_whall_close_events = 0;
volatile uint32_t tb_whall_open_events  = 0;
volatile uint32_t tb_thall_close_events = 0;
volatile uint32_t tb_thall_open_events  = 0;

volatile uint32_t tb_whall_close_tick = 0;
volatile uint32_t tb_whall_open_tick  = 0;
volatile uint32_t tb_thall_close_tick = 0;
volatile uint32_t tb_thall_open_tick  = 0;

volatile uint32_t tb_doorhall_isr_count = 0;
volatile uint32_t tb_doorhall_samples   = 0;

/* Count one falling edge, dropping chatter within TB_DOORHALL_DEBOUNCE_MS of the
 * previous accepted edge on the same line. ISR context. */
static void count_edge(volatile uint32_t *ctr, volatile uint32_t *last_tick)
{
	uint32_t now = HAL_GetTick();

#if (TB_DOORHALL_DEBOUNCE_MS > 0U)
	/* *last_tick == 0 means "no edge yet" -> always accept the first one. */
	if ((*last_tick != 0U) &&
	    ((uint32_t)(now - *last_tick) < TB_DOORHALL_DEBOUNCE_MS))
	{
		return;
	}
#endif

	*last_tick = (now != 0U) ? now : 1U;
	(*ctr)++;
	tb_doorhall_isr_count++;
}

void TB_DoorHall_Init(void)
{
	tb_doorhall_enable = 0;

	tb_whall_close_level = 0;
	tb_whall_open_level  = 0;
	tb_thall_close_level = 0;
	tb_thall_open_level  = 0;

	tb_wdoor_at_close = 0;
	tb_wdoor_at_open  = 0;
	tb_tdoor_at_close = 0;
	tb_tdoor_at_open  = 0;

	tb_whall_close_events = 0;
	tb_whall_open_events  = 0;
	tb_thall_close_events = 0;
	tb_thall_open_events  = 0;

	tb_whall_close_tick = 0;
	tb_whall_open_tick  = 0;
	tb_thall_close_tick = 0;
	tb_thall_open_tick  = 0;

	tb_doorhall_isr_count = 0;
	tb_doorhall_samples   = 0;
}

void TB_DoorHall_Poll(void)
{
	/* Disabled: freeze the last snapshot. */
	if (!tb_doorhall_enable)
	{
		return;
	}

	/* 1) Raw levels (diagnostic, polled on purpose - runtime observation). */
	tb_whall_close_level = gpio_ctrl_exti_read(GPIO_EXTI_WHALL_CLOSE);
	tb_whall_open_level  = gpio_ctrl_exti_read(GPIO_EXTI_WHALL_OPEN);
	tb_thall_close_level = gpio_ctrl_exti_read(GPIO_EXTI_THALL_CLOSE);
	tb_thall_open_level  = gpio_ctrl_exti_read(GPIO_EXTI_THALL_OPEN);

	/* 2) at-limit decode via the production drivers (matches the scenario). */
	tb_wdoor_at_close = WDoor_AtClose();
	tb_wdoor_at_open  = WDoor_AtOpen();
	tb_tdoor_at_close = TDoor_AtClose();
	tb_tdoor_at_open  = TDoor_AtOpen();

	/* 3) Edge counts are maintained by TB_DoorHall_OnEXTI() in ISR context. */

	tb_doorhall_samples++;
}

void TB_DoorHall_OnEXTI(uint16_t gpio_pin)
{
	/* Ignore every edge while the bench is off - this replaces the old
	 * "clear stale flags once at enable" arming step. */
	if (!tb_doorhall_enable)
	{
		return;
	}

	switch (gpio_pin)
	{
	case exti3_WHALL_CLOSE_Pin:  /* PF3 - 배수문 닫힘 리밋 */
		count_edge(&tb_whall_close_events, &tb_whall_close_tick);
		break;
	case exti4_WHALL_OPEN_Pin:   /* PF4 - 배수문 열림 리밋 */
		count_edge(&tb_whall_open_events, &tb_whall_open_tick);
		break;
	case exti2_THALL_CLOSE_Pin:  /* PF2 - 배출문 닫힘 리밋 */
		count_edge(&tb_thall_close_events, &tb_thall_close_tick);
		break;
	case exti5_THALL_OPEN_Pin:   /* PF5 - 배출문 열림 리밋 */
		count_edge(&tb_thall_open_events, &tb_thall_open_tick);
		break;
	default:
		break;
	}
}
