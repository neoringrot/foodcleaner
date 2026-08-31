#include "tb_drv8871.h"

#include "cmsis_os.h"
#include "wdoor.h"
#include "tdoor.h"

/* Runtime switches -- set from the debugger while running. Default off/safe. */
volatile uint8_t tb_wdoor_enable  = 0;
volatile uint8_t tb_wdoor_duty    = 50;
volatile uint8_t tb_wdoor_reverse = 0;

volatile uint8_t tb_tdoor_enable  = 0;
volatile uint8_t tb_tdoor_duty    = 50;
volatile uint8_t tb_tdoor_reverse = 0;

/* Limit auto-stop: 1 = stop at the Hall limit for the direction being driven,
 * 0 = free-run (old behaviour, motor stalls on the end stop). */
volatile uint8_t tb_wdoor_limit_stop = 1;
volatile uint8_t tb_tdoor_limit_stop = 1;

/* Status (read-only in the debugger): 1 = this door was auto-stopped by its
 * limit. Cleared when the motor is next started off the limit. */
volatile uint8_t tb_wdoor_limit_hit = 0;
volatile uint8_t tb_tdoor_limit_hit = 0;

/* WDoor direction profile. tb_wdoor_reverse: **0 = 닫힘, 1 = 열림** -- confirmed
 * on the bench, and the opposite way round from TDoor. Values default to the
 * WDOOR_* profile in wdoor.h, which the 모음/동작 scenarios drive from too, so
 * the bench and the product move the door the same way. Open kick-starts at
 * 80 % for 2 s then holds 65 % and normally ends on WHALL-OPEN with the 6 s cap
 * as a backstop; Close holds 80 % flat and ends on its 4.2 s timer (WHALL-CLOSE
 * stops it earlier if it fires).
 * While the profile is on, Poll() writes the duty it applies into
 * tb_wdoor_duty, so that variable is the live "current duty" readout. */
volatile uint8_t  tb_wdoor_profile         = 1;
volatile uint8_t  tb_wdoor_open_kick_duty  = WDOOR_OPEN_KICK_DUTY;
volatile uint16_t tb_wdoor_open_kick_ms    = WDOOR_OPEN_KICK_MS;
volatile uint8_t  tb_wdoor_open_run_duty   = WDOOR_OPEN_RUN_DUTY;
volatile uint16_t tb_wdoor_open_ms         = WDOOR_OPEN_MAX_MS;
volatile uint8_t  tb_wdoor_close_duty      = WDOOR_CLOSE_DUTY;
volatile uint16_t tb_wdoor_close_ms        = WDOOR_CLOSE_MS;

/* Status (read-only): 1 = the run ended on its time limit rather than on a
 * Hall limit. Expected for Close; on Open it means WHALL-OPEN never fired. */
volatile uint8_t tb_wdoor_time_hit = 0;

/* TDoor profile: 80 % flat, and the limit does not stop the motor dead -- the
 * door keeps turning for tb_tdoor_overrun_ms (1 s) past the Hall so it travels
 * clear of the sensor before coasting. */
volatile uint8_t  tb_tdoor_profile      = 1;
volatile uint8_t  tb_tdoor_run_duty     = TDOOR_DUTY;
volatile uint16_t tb_tdoor_overrun_ms   = TDOOR_OVERRUN_MS;
volatile uint16_t tb_tdoor_open_max_ms  = TDOOR_OPEN_MAX_MS;
volatile uint16_t tb_tdoor_close_max_ms = TDOOR_CLOSE_MAX_MS;

/* Status (read-only): 1 = the Hall has been seen and the door is inside the
 * extra-run window. */
volatile uint8_t tb_tdoor_overrun = 0;

/* Status (read-only): 1 = the run ended on its per-direction time cap, i.e.
 * the Hall never fired (or fired too late for the full overrun). */
volatile uint8_t tb_tdoor_time_hit = 0;

/* Track previous enable state so VM is switched only on transitions (avoids
 * re-toggling the enable GPIO every poll). */
static uint8_t wdoor_running = 0;
static uint8_t tdoor_running = 0;

/* Consecutive at-limit polls seen so far (debounce accumulator). */
static uint8_t wdoor_confirm = 0;
static uint8_t tdoor_confirm = 0;

/* Start of the current WDoor run (HAL tick), and the direction it started in.
 * Both the Open kick window and the Close 4.2 s timer are measured from here;
 * a mid-run direction flip starts a new run (the door has to break away from a
 * standstill again), so it restarts the stamp too. */
static uint32_t wdoor_start_ms = 0;
static uint8_t  wdoor_dir      = 0;

/* Start of the current TDoor run and of its overrun window (HAL ticks), and
 * the direction the run started in. */
static uint32_t tdoor_start_ms = 0;
static uint32_t tdoor_over_ms  = 0;
static uint8_t  tdoor_dir      = 0;

/* Poll() runs on MotorTask at 1 ms, so this is a ~5 ms confirm window: long
 * enough that a glitch on the open-drain Hall line cannot latch a false stop,
 * short enough that the door barely moves past the magnet. */
#ifndef TB_DOOR_LIMIT_CONFIRM
#define TB_DOOR_LIMIT_CONFIRM 5U
#endif

/* Debounced at-limit decision. at_limit is the production decode for the
 * direction currently being driven; confirm is that door's accumulator. */
static uint8_t limit_hit(uint8_t at_limit, uint8_t *confirm)
{
	if (!at_limit)
	{
		*confirm = 0;
		return 0U;
	}
	if (*confirm < (uint8_t)TB_DOOR_LIMIT_CONFIRM)
	{
		(*confirm)++;
	}
	return (*confirm >= (uint8_t)TB_DOOR_LIMIT_CONFIRM) ? 1U : 0U;
}

void TB_DRV8871_Init(void)
{
	WDoor_Init();
	TDoor_Init();
	wdoor_running = 0;
	tdoor_running = 0;
	wdoor_confirm = 0;
	tdoor_confirm = 0;
	wdoor_start_ms = 0;
	wdoor_dir = 0;
	tdoor_start_ms = 0;
	tdoor_over_ms = 0;
	tdoor_dir = 0;
	tb_wdoor_limit_hit = 0;
	tb_tdoor_limit_hit = 0;
	tb_wdoor_time_hit = 0;
	tb_tdoor_overrun = 0;
	tb_tdoor_time_hit = 0;
}

void TB_DRV8871_Poll(void)
{
	/* ---- U5 Water Door ---- */
	if (tb_wdoor_enable)
	{
		/* tb_wdoor_reverse: 0 = 닫힘, 1 = 열림. Only the limit in the travel
		 * direction blocks, so sitting on one end stop still allows the
		 * opposite command to back the door off it. Reached*() = latched EXTI
		 * falling edge since LimitArm() OR the level now, the same arrival test
		 * the scenarios use. */
		uint8_t at = tb_wdoor_reverse ? WDoor_ReachedOpen() : WDoor_ReachedClose();

		if (tb_wdoor_limit_stop && limit_hit(at, &wdoor_confirm))
		{
			WDoor_Stop();            /* coast, same as the scenario does */
			WDoor_Disable();         /* VM off */
			wdoor_running = 0;
			tb_wdoor_enable = 0;     /* auto-stop: the switch drops itself */
			tb_wdoor_limit_hit = 1;
		}
		else
		{
			uint32_t el;
			uint16_t run_ms;

			if (!wdoor_running)
			{
				WDoor_LimitArm();    /* drop the stale edge of the last move */
				WDoor_Enable();      /* VM on before driving */
				wdoor_running = 1;
				wdoor_start_ms = HAL_GetTick();   /* run clock starts */
				wdoor_dir = tb_wdoor_reverse;
				tb_wdoor_limit_hit = 0;
				tb_wdoor_time_hit = 0;
			}
			else if (wdoor_dir != tb_wdoor_reverse)
			{
				/* Direction flipped mid-run: this is a new run -- re-kick and
				 * restart the clock, the door has to break away again. */
				WDoor_LimitArm();
				wdoor_start_ms = HAL_GetTick();
				wdoor_dir = tb_wdoor_reverse;
				tb_wdoor_time_hit = 0;
			}

			/* Unsigned subtraction, so the tick rollover at ~49 days is
			 * harmless. */
			el = HAL_GetTick() - wdoor_start_ms;

			/* Per-direction run length. Close (reverse=0): the timer IS the
			 * stop condition (4.2 s). Open (reverse=1): the Hall normally
			 * stops it first, so this is the backstop that keeps the motor off
			 * the end stop if WHALL-OPEN never fires (6 s). */
			run_ms = tb_wdoor_reverse ? tb_wdoor_open_ms : tb_wdoor_close_ms;

			if (tb_wdoor_profile && el >= (uint32_t)run_ms)
			{
				WDoor_Stop();
				WDoor_Disable();
				wdoor_running = 0;
				tb_wdoor_enable = 0;
				tb_wdoor_time_hit = 1;
			}
			else
			{
				if (tb_wdoor_profile)
				{
					/* Open (reverse=1): kick duty for tb_wdoor_open_kick_ms,
					 * then the hold duty. Close (reverse=0): flat. */
					tb_wdoor_duty =
					    tb_wdoor_reverse
					        ? ((el < (uint32_t)tb_wdoor_open_kick_ms)
					               ? tb_wdoor_open_kick_duty
					               : tb_wdoor_open_run_duty)
					        : tb_wdoor_close_duty;
				}

				if (tb_wdoor_reverse)
				{
					WDoor_Open(tb_wdoor_duty);    /* reverse=1 -> 열림 */
				}
				else
				{
					WDoor_Close(tb_wdoor_duty);   /* reverse=0 -> 닫힘 */
				}
			}
		}
	}
	else if (wdoor_running)
	{
		WDoor_Stop();            /* coast */
		WDoor_Disable();         /* VM off */
		wdoor_running = 0;
		wdoor_confirm = 0;
	}

	/* ---- U7 Trash Door ---- */
	if (tb_tdoor_enable)
	{
		uint8_t  at      = tb_tdoor_reverse ? TDoor_ReachedClose()
		                                   : TDoor_ReachedOpen();
		uint16_t over_ms = tb_tdoor_profile ? tb_tdoor_overrun_ms : 0;
		uint16_t max_ms  = !tb_tdoor_profile ? 0                    /* 0 = no cap */
		                   : (tb_tdoor_reverse ? tb_tdoor_close_max_ms
		                                       : tb_tdoor_open_max_ms);

		/* Direction flipped mid-run: this is a new run toward the other limit,
		 * so restart the run clock and drop a pending overrun + the debounce. */
		if (tdoor_running && (tdoor_dir != tb_tdoor_reverse))
		{
			TDoor_LimitArm();
			tdoor_dir = tb_tdoor_reverse;
			tdoor_start_ms = HAL_GetTick();
			tdoor_confirm = 0;
			tb_tdoor_overrun = 0;
			tb_tdoor_time_hit = 0;
		}

		/* Unlike WDoor, the limit does not stop this door dead: the first
		 * confirmed hit only LATCHES the overrun window and the motor keeps
		 * driving for over_ms. Latched on purpose -- once the door travels past
		 * the magnet the level goes away again, and that must not cut the
		 * overrun short. */
		if (!tb_tdoor_overrun && tb_tdoor_limit_stop &&
		    limit_hit(at, &tdoor_confirm))
		{
			tb_tdoor_overrun = 1;
			tdoor_over_ms = HAL_GetTick();
		}

		if (tdoor_running && max_ms &&
		    ((HAL_GetTick() - tdoor_start_ms) >= (uint32_t)max_ms))
		{
			/* Hard ceiling on the whole run, both directions -- the backstop
			 * for a Hall that never fires. It also truncates an overrun that
			 * was latched too close to the cap. */
			TDoor_Stop();
			TDoor_Disable();
			tdoor_running = 0;
			tdoor_confirm = 0;
			tb_tdoor_enable = 0;
			tb_tdoor_overrun = 0;
			tb_tdoor_time_hit = 1;
		}
		else if (tb_tdoor_overrun &&
		         ((HAL_GetTick() - tdoor_over_ms) >= (uint32_t)over_ms))
		{
			TDoor_Stop();            /* coast */
			TDoor_Disable();         /* VM off */
			tdoor_running = 0;
			tdoor_confirm = 0;
			tb_tdoor_enable = 0;     /* auto-stop: the switch drops itself */
			tb_tdoor_overrun = 0;
			tb_tdoor_limit_hit = 1;
		}
		else
		{
			if (!tdoor_running)
			{
				TDoor_LimitArm();    /* drop the stale edge of the last move */
				TDoor_Enable();      /* VM on before driving */
				tdoor_running = 1;
				tdoor_start_ms = HAL_GetTick();   /* run clock starts */
				tdoor_dir = tb_tdoor_reverse;
				tb_tdoor_limit_hit = 0;
				tb_tdoor_time_hit = 0;
			}

			if (tb_tdoor_profile)
			{
				tb_tdoor_duty = tb_tdoor_run_duty;
			}

			if (tb_tdoor_reverse)
			{
				TDoor_Close(tb_tdoor_duty);
			}
			else
			{
				TDoor_Open(tb_tdoor_duty);
			}
		}
	}
	else if (tdoor_running)
	{
		TDoor_Stop();
		TDoor_Disable();
		tdoor_running = 0;
		tdoor_confirm = 0;
		tb_tdoor_overrun = 0;
	}
}

void TB_DRV8871_Loop(void)
{
	for (;;)
	{
		TB_DRV8871_Poll();
		osDelay(10);
	}
}
