#include "tb_doorhall.h"

#include "gpio_ctrl.h"
#include "wdoor.h"   /* WDoor_AtOpen/AtClose - reuse the production decode */
#include "tdoor.h"   /* TDoor_AtOpen/AtClose */

/* Testbed for the four door-limit Hall sensors (WHALL/THALL open/close). See
 * tb_doorhall.h for the pin map and the single-owner (StartDefaultTask)
 * constraint.
 *
 * Poll() reads raw levels for diagnostics, decodes at-limit via the same driver
 * functions the scenario uses (so the tb never disagrees with the firmware), and
 * counts falling edges off the shared gpio_ctrl EXTI flags. Disabled -> the last
 * snapshot is left untouched and the armed baseline is dropped so re-enabling
 * seeds a fresh one instead of firing a spurious edge. Never drives an
 * actuator. */

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

volatile uint32_t tb_doorhall_samples = 0;

/* 0 until the enable edge arms monitoring; used to clear stale EXTI flags once
 * so a level already asserted at enable time is not counted as an edge. */
static uint8_t s_armed;

/* Count and clear one falling-edge flag. */
static void count_edge(gpio_exti_t e, volatile uint32_t *ctr)
{
	if (gpio_ctrl_exti_flag_get(e))
	{
		(*ctr)++;
		gpio_ctrl_exti_flag_clear(e);
	}
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

	tb_doorhall_samples = 0;

	s_armed = 0;
}

void TB_DoorHall_Poll(void)
{
	/* Disabled: freeze the last snapshot and drop the armed baseline. */
	if (!tb_doorhall_enable)
	{
		s_armed = 0;
		return;
	}

	/* First enabled poll only: clear any edge latched before the tb took over. */
	if (!s_armed)
	{
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WHALL_CLOSE);
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WHALL_OPEN);
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_THALL_CLOSE);
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_THALL_OPEN);
		s_armed = 1;
	}

	/* 1) Raw levels (diagnostic). */
	tb_whall_close_level = gpio_ctrl_exti_read(GPIO_EXTI_WHALL_CLOSE);
	tb_whall_open_level  = gpio_ctrl_exti_read(GPIO_EXTI_WHALL_OPEN);
	tb_thall_close_level = gpio_ctrl_exti_read(GPIO_EXTI_THALL_CLOSE);
	tb_thall_open_level  = gpio_ctrl_exti_read(GPIO_EXTI_THALL_OPEN);

	/* 2) at-limit decode via the production drivers (matches the scenario). */
	tb_wdoor_at_close = WDoor_AtClose();
	tb_wdoor_at_open  = WDoor_AtOpen();
	tb_tdoor_at_close = TDoor_AtClose();
	tb_tdoor_at_open  = TDoor_AtOpen();

	/* 3) Falling-edge counts. */
	count_edge(GPIO_EXTI_WHALL_CLOSE, &tb_whall_close_events);
	count_edge(GPIO_EXTI_WHALL_OPEN,  &tb_whall_open_events);
	count_edge(GPIO_EXTI_THALL_CLOSE, &tb_thall_close_events);
	count_edge(GPIO_EXTI_THALL_OPEN,  &tb_thall_open_events);

	tb_doorhall_samples++;
}
