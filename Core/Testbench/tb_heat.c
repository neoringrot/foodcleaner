#include "tb_heat.h"

#include "gpio_ctrl.h"
#include "thermistor.h"
#include "heater_hyst.h"   /* shared hysteresis/over-temp decision (also used by dongjak) */

/* Testbed for the 220V heater on HT-POWER (PA12). See tb_heat.h for the
 * contract, the defaultTask/ADC placement rule and the safety model.
 *
 * The temperature is the thermistor.c EMA-smoothed value (Thermistor_Get*),
 * the same source the 동작 scenario's dj_heater_tick() uses, so the loop here
 * matches the scenario's. This file does NOT read the ADC -- it only consumes
 * the value Thermistor_Tick() already cached on this same task. */

/* ---- Live inputs (debugger). Defaults = safe/off, band = 동작 scenario. --- */
volatile uint8_t  tb_heat_enable      = 0u;
volatile uint8_t  tb_heat_mode        = TB_HEAT_MODE_HYST;
volatile uint8_t  tb_heat_ch          = 0u;      /* DJ_TEMP_CH (처리통 probe)   */
volatile uint8_t  tb_heat_manual_on   = 0u;
volatile uint8_t  tb_heat_clear_fault = 0u;

volatile int16_t  tb_heat_on_d10      = 1900;    /* 190.0C, = DJ_TEMP_HEATER_ON  */
volatile int16_t  tb_heat_off_d10     = 1950;    /* 195.0C, = DJ_TEMP_HEATER_OFF */
volatile int16_t  tb_heat_safety_d10  = 2100;    /* 210.0C, = DJ_TEMP_SAFETY     */
volatile uint32_t tb_heat_max_on_ms   = 0u;      /* continuous-ON watchdog; 0 = OFF
                                                  * (opt-in bench idle-guard, e.g.
                                                  * set 600000 = 10 min). A real run
                                                  * outlasts any short cap, so it is
                                                  * disabled by default. */

/* ---- Live status (debugger) -------------------------------------------- */
volatile uint8_t  tb_heat_out       = 0u;
volatile int16_t  tb_heat_temp_d10  = THERMISTOR_ERR_D10;
volatile uint8_t  tb_heat_fault     = TB_HEAT_FAULT_NONE;
volatile uint32_t tb_heat_on_ms     = 0u;
volatile uint32_t tb_heat_samples   = 0u;

/* ---- Timed logging session (5 min, one sample / 30 s) ------------------- */
volatile TbHeatLog_t tb_heat_log[TB_HEAT_LOG_COUNT] = {{0}};
volatile uint8_t     tb_heat_log_count      = 0u;
volatile uint8_t     tb_heat_session_active = 0u;
volatile uint32_t    tb_heat_session_ms     = 0u;

/* ---- Private state ------------------------------------------------------ */
static uint8_t  s_prev_enable  = 0u;  /* for enable rising-edge detection      */
static uint8_t  s_out          = 0u;  /* last level we commanded (hyst hold)   */
static uint32_t s_on_since     = 0u;  /* HAL tick when the heater last went ON */
static uint32_t s_session_start = 0u; /* HAL tick when the session began       */

/* Drive HT-POWER and keep the mirror/timer consistent. */
static void tb_heat_apply(uint8_t on, uint32_t now)
{
	if (on && !s_out)
	{
		s_on_since = now;             /* rising edge: start the ON-time clock   */
	}
	s_out = on ? 1u : 0u;
	gpio_ctrl_set(GPIO_OUT_HT_POWER, s_out);
	tb_heat_out = s_out;
	tb_heat_on_ms = s_out ? (now - s_on_since) : 0u;
}

/* Clear the log and (re)start a fresh 5-minute logging session. */
static void tb_heat_session_begin(uint32_t now)
{
	uint8_t i;
	for (i = 0u; i < TB_HEAT_LOG_COUNT; i++)
	{
		tb_heat_log[i].t_ms     = 0u;
		tb_heat_log[i].temp_d10 = THERMISTOR_ERR_D10;
		tb_heat_log[i].out      = 0u;
		tb_heat_log[i].fault    = TB_HEAT_FAULT_NONE;
	}
	tb_heat_log_count      = 0u;
	tb_heat_session_ms     = 0u;
	tb_heat_session_active = 1u;
	s_session_start        = now;
}

static void tb_heat_session_end(void)
{
	tb_heat_session_active = 0u;
}

void TB_Heat_Init(void)
{
	tb_heat_enable      = 0u;
	tb_heat_manual_on   = 0u;
	tb_heat_clear_fault = 0u;
	tb_heat_fault       = TB_HEAT_FAULT_NONE;
	tb_heat_temp_d10    = THERMISTOR_ERR_D10;
	tb_heat_on_ms       = 0u;
	tb_heat_samples     = 0u;

	s_prev_enable   = 0u;
	s_out           = 0u;
	s_on_since      = 0u;
	s_session_start = 0u;

	tb_heat_session_end();
	tb_heat_log_count  = 0u;
	tb_heat_session_ms = 0u;

	/* Known-off: never leave the heater energised by a prior run. */
	gpio_ctrl_off(GPIO_OUT_HT_POWER);
	tb_heat_out = 0u;
}

void TB_Heat_Poll(uint8_t testbench_active)
{
	uint32_t now = HAL_GetTick();
	uint8_t  ch  = (tb_heat_ch < THERMISTOR_COUNT) ? tb_heat_ch : 0u;
	int16_t  t;
	uint8_t  temp_valid;
	uint8_t  desired;

	/* --- Scenario owns HT-POWER: keep hands off, reset our timers. --------
	 * When not in testbench mode the 동작 MotorTask drives HT-POWER via
	 * dj_heater_tick(); writing it here too would fight that. Do NOT touch the
	 * pin -- just clear our mirror so a later activation starts from OFF. */
	if (!testbench_active)
	{
		s_out         = 0u;
		s_on_since    = now;
		s_prev_enable = 0u;
		tb_heat_out   = gpio_ctrl_is_on(GPIO_OUT_HT_POWER); /* reflect true pin */
		tb_heat_on_ms = 0u;
		tb_heat_session_end();            /* scenario owns the pin: no session   */
		return;
	}

	/* --- Enable rising edge (0->1): clear fault latch + arm a fresh 5-min
	 * logging session. Also an explicit fault clear request. --------------- */
	if ((tb_heat_enable && !s_prev_enable) || tb_heat_clear_fault)
	{
		tb_heat_fault       = TB_HEAT_FAULT_NONE;
		tb_heat_clear_fault = 0u;
	}
	if (tb_heat_enable && !s_prev_enable)
	{
		tb_heat_session_begin(now);
	}
	s_prev_enable = tb_heat_enable ? 1u : 0u;

	/* --- Disabled: force OFF, close any session, hold the fault for inspect. */
	if (!tb_heat_enable)
	{
		tb_heat_apply(0u, now);
		tb_heat_session_end();            /* operator stop (enable -> 0)         */
		return;
	}

	/* --- Read the shared (EMA-smoothed) probe temperature. No ADC here. ---- */
	{
		HeaterHyst_t cfg;
		cfg.on_d10     = tb_heat_on_d10;
		cfg.off_d10    = tb_heat_off_d10;
		cfg.safety_d10 = tb_heat_safety_d10;

		t = Thermistor_GetCelsius_d10(ch);
		temp_valid = (uint8_t)(t != THERMISTOR_ERR_D10);
		tb_heat_temp_d10 = t;

		/* --- Latched safety: over-temp (valid && >= safety). Real safety is the
		 * HW bimetal; this FW backstop mirrors DJ_TEMP_SAFETY and applies in BOTH
		 * modes -- MANUAL cannot override a real over-temp. Uses the SAME predicate
		 * the 동작 scenario decision uses. */
		if (Heater_IsOverTemp(&cfg, t, temp_valid))
		{
			tb_heat_fault |= TB_HEAT_FAULT_OVERTEMP;
		}

		/* --- Latched idle-guard watchdog (TESTBENCH-ONLY, opt-in). Disabled by
		 * default (tb_heat_max_on_ms == 0); the 동작 scenario has no such cap. */
		if ((tb_heat_max_on_ms != 0u) && s_out &&
		    ((now - s_on_since) >= tb_heat_max_on_ms))
		{
			tb_heat_fault |= TB_HEAT_FAULT_WATCHDOG;
		}

		/* --- Sensor status bit (informational). NOTE: this no longer force-cuts
		 * MANUAL -- a missing/ERR probe only blocks the HYST loop (which cannot
		 * decide blind), so an electrical MANUAL test works with no probe. */
		if (!temp_valid)
		{
			tb_heat_fault |= TB_HEAT_FAULT_SENSOR;
		}
		else
		{
			tb_heat_fault &= (uint8_t)~TB_HEAT_FAULT_SENSOR;  /* recovers */
		}

		/* --- Base request from the selected mode. ------------------------- */
		if (tb_heat_mode == TB_HEAT_MODE_MANUAL)
		{
			/* Raw pin force for electrical checks -- no sensor gate. */
			desired = tb_heat_manual_on ? 1u : 0u;
		}
		else /* TB_HEAT_MODE_HYST: shared decision with the 동작 scenario. */
		{
			switch (Heater_HystDecide(&cfg, t, temp_valid))
			{
			case HEATER_CMD_ON:   desired = 1u; break;
			case HEATER_CMD_OFF:  desired = 0u; break;
			case HEATER_CMD_HOLD:
			default:
				/* Inside the band -> hold last state. But we cannot run the loop
				 * blind, so a HOLD caused by an invalid reading means OFF. */
				desired = temp_valid ? s_out : 0u;
				break;
			}
		}

		/* --- Latched-fault override beats any request (both modes). -------- */
		if ((tb_heat_fault & (TB_HEAT_FAULT_OVERTEMP | TB_HEAT_FAULT_WATCHDOG)) != 0u)
		{
			desired = 0u;                /* stays off until the latch is cleared */
		}
	}

	tb_heat_apply(desired, now);
	tb_heat_samples++;

	/* --- Timed logging session: snapshot every 30 s, auto-terminate at 5 min.
	 * Runs after the heater decision so each sample reflects THIS cycle's temp
	 * and output. Samples land at elapsed = 0, 30, 60, ... 300 s. */
	if (tb_heat_session_active)
	{
		uint32_t elapsed = now - s_session_start;
		tb_heat_session_ms = elapsed;

		/* Due for the next slot? (count-th sample at count*30 s.) */
		if ((tb_heat_log_count < TB_HEAT_LOG_COUNT) &&
		    (elapsed >= ((uint32_t)tb_heat_log_count * TB_HEAT_LOG_PERIOD_MS)))
		{
			uint8_t i = tb_heat_log_count;
			tb_heat_log[i].t_ms     = elapsed;
			tb_heat_log[i].temp_d10 = tb_heat_temp_d10;
			tb_heat_log[i].out      = tb_heat_out;
			tb_heat_log[i].fault    = tb_heat_fault;
			tb_heat_log_count       = (uint8_t)(i + 1u);
		}

		/* 5 minutes up -> auto-terminate: clear enable + force heater OFF. */
		if (elapsed >= (uint32_t)TB_HEAT_SESSION_MS)
		{
			tb_heat_apply(0u, now);
			tb_heat_enable = 0u;
			s_prev_enable  = 0u;          /* re-arm cleanly on the next 0->1     */
			tb_heat_session_end();
		}
	}
}
