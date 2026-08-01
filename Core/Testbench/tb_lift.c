#include "tb_lift.h"

#include "cmsis_os.h"
#include "lift_motor.h"

/* Runtime switches -- set from the debugger while running. Default off/safe.
 *
 * SOFTWARE PWM: the lift's IN1/IN2 are on PG3/PG4, which have no timer channel
 * on the STM32F103ZET6, so hardware PWM (as the TIM3 doors use) is impossible.
 * Speed is therefore produced in software here: within each PWM window of
 * tb_lift_pwm_period_ms, the active input is driven for (duty %) of the window
 * and coasted (both inputs low) for the remainder -- the coast-decay speed
 * control of the DRV8871 (datasheet 7.4.2), exactly like the doors, but toggled
 * by this poll loop instead of the timer. The VM gate (PB12) stays ON for the
 * whole motion; only the IN pin is pulsed.
 *
 * Resolution/frequency are a trade-off set by tb_lift_pwm_period_ms because the
 * pulse is quantised to the ~1 ms poll cadence of StartMotorTask: e.g. a 10 ms
 * period is 100 Hz with ~10 % duty steps; a 20 ms period is 50 Hz with ~5 %
 * steps. Keep the period a small multiple of the poll interval. */
volatile uint8_t  tb_lift_enable        = 0;
volatile uint8_t  tb_lift_reverse       = 0;
volatile uint8_t  tb_lift_duty          = 50;  /* 0..100 % (0 = coast, 100 = full) */
volatile uint16_t tb_lift_pwm_period_ms = 10;  /* software-PWM period (>=1 ms)      */

/* Track previous enable state so VM is switched only on transitions (avoids
 * re-toggling the enable GPIO every poll). */
static uint8_t lift_running = 0;

/* Software-PWM state. lift_applied is the output currently driven onto IN1/IN2
 * (0 = coast, 1 = Up, 2 = Down) so the pins are only rewritten on a change.
 * lift_pwm_start marks the tick at which the current PWM window began. */
static uint8_t  lift_applied = 0;
static uint32_t lift_pwm_start = 0;

void TB_Lift_Init(void)
{
	/* Lift_Init() (VM off, coast) is performed in main.c USER CODE BEGIN 2,
	 * which always runs before the scheduler; here we only reset run-state. */
	lift_running   = 0;
	lift_applied   = 0;
	lift_pwm_start = 0;
}

void TB_Lift_Poll(void)
{
	/* ---- U6 Lift (software PWM) ---- */
	if (tb_lift_enable)
	{
		uint32_t now    = HAL_GetTick();
		uint16_t period = tb_lift_pwm_period_ms ? tb_lift_pwm_period_ms : 1U;
		uint8_t  duty   = (tb_lift_duty > 100U) ? 100U : tb_lift_duty;
		uint32_t elapsed;
		uint32_t on_ticks;
		uint8_t  want_on;
		uint8_t  desired;

		if (!lift_running)
		{
			Lift_Enable();          /* VM on before driving */
			lift_pwm_start = now;
			lift_applied   = 0;     /* nothing driven yet; force first write */
			lift_running   = 1;
		}

		/* Advance the PWM window; wrap once its period has elapsed. */
		elapsed = (uint32_t)(now - lift_pwm_start);
		if (elapsed >= period)
		{
			lift_pwm_start = now;
			elapsed = 0U;
		}

		/* On-time = duty% of the window. Handle the extremes exactly so 0 % is
		 * always coast and 100 % is always driven (no 1-tick glitch). */
		on_ticks = (uint32_t)period * duty / 100U;
		if (duty == 0U)        { want_on = 0U; }
		else if (duty >= 100U) { want_on = 1U; }
		else                   { want_on = (elapsed < on_ticks) ? 1U : 0U; }

		/* Resolve to a concrete output and apply only on change (also re-issues
		 * the correct direction if tb_lift_reverse is flipped live). */
		desired = want_on ? (tb_lift_reverse ? 2U : 1U) : 0U;
		if (desired != lift_applied)
		{
			switch (desired)
			{
			case 1U:  Lift_Up();   break;   /* drive Up   */
			case 2U:  Lift_Down(); break;   /* drive Down */
			default:  Lift_Stop(); break;   /* coast      */
			}
			lift_applied = desired;
		}
	}
	else if (lift_running)
	{
		Lift_Stop();             /* coast */
		Lift_Disable();          /* VM off */
		lift_running = 0;
		lift_applied = 0;
	}
}

void TB_Lift_Loop(void)
{
	for (;;)
	{
		TB_Lift_Poll();
		osDelay(1);   /* 1 ms: software PWM is quantised to this cadence */
	}
}
