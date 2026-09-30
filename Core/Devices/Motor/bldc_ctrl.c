#include "bldc_ctrl.h"

/* Generic BLDC closed-loop control (PI on FGOUT + soft-lock). See bldc_ctrl.h
 * for the model; the per-motor tuning lives in the two configs below. */

/* ---- Speed ladders -------------------------------------------------------
 * M1 (grinder) is direct drive, so its ladder is in MOTOR RPM. M2 (stirrer)
 * runs behind a 1:82 gearbox + x1.2 gear, so its ladder is in BLADE RPM (§0.22, the DRV8306
 * layer converts to motor RPM via gear_ratio). Each SW7/SW8 press advances one
 * rung and wraps; every start begins at rung 0. */
static const uint16_t GRIND_LADDER[] = { 800U, 1200U, 1600U, 2000U, 2500U }; /* motor RPM  */
static const uint16_t STIR_LADDER[]  = {  20U,   25U,   30U,   35U,   40U }; /* BLADE RPM (§0.22) */

/* ---- Per-motor configuration ---------------------------------------------
 * PI gains are conservative starting points -- TRIM ON THE BENCH against
 * c->meas_out_rpm. The gains differ between motors because the error unit
 * differs (see bldc_ctrl.h "UNITS GOTCHA"):
 *   M1 err is MOTOR RPM (0..2500): a small per-mille-per-RPM gain (0.25) keeps
 *       a large error from slamming the output to a rail.
 *   M2 err is OUTPUT RPM (0..40): ~10 per-mille of duty is needed per output
 *       RPM at no load, so Kp = 5 leans on the feed-forward and lets the
 *       integrator close the rest.
 * The feed-forward (DRV8306_FeedForwardPerMille) supplies the bulk of the duty
 * in both cases, so the PI only trims the residual. */
static const BldcCtrl_Cfg_t grind_cfg =
{
	.h           = &drv8306_m1,
	.ladder      = GRIND_LADDER,
	.ladder_len  = (uint8_t)(sizeof(GRIND_LADDER) / sizeof(GRIND_LADDER[0])),
	.kp_num      = 1, .kp_den = 4,        /* 0.25 per-mille / (motor RPM)      */
	.ki_num      = 1, .ki_den = 8000,     /* small: integrate residual slowly  */
	.duty_min_pm = 30, .duty_max_pm = 1000,
	/* Grinder: crushing load legitimately sags RPM with duty high, so use a
	 * LONGER stall window than the stirrer to avoid tripping on a hard-but-
	 * normal crush; a true jam still trips and reverse-crushes (scenario's
	 * load-based direction reversal). */
	.stall_pct   = 60U, .duty_sat_pm = 980, .stall_ms = 1500U,
	.unjam_ms    = 700U, .unjam_pm = 700, .retry_max = 4U,
	.slew_rpm_per_s = 1000U,               /* 0->2500 rpm in ~2.5 s (motor rpm) */
	.period_ms   = 100U, .wake_mask_ms = 50U,
};

static const BldcCtrl_Cfg_t stir_cfg =
{
	.h           = &drv8306_m2,
	.ladder      = STIR_LADDER,
	.ladder_len  = (uint8_t)(sizeof(STIR_LADDER) / sizeof(STIR_LADDER[0])),
	.kp_num      = 5, .kp_den = 1,        /* 5 per-mille / (output RPM)        */
	.ki_num      = 2, .ki_den = 1000,
	.duty_min_pm = 30, .duty_max_pm = 1000,
	.stall_pct   = 60U, .duty_sat_pm = 980, .stall_ms = 800U,
	.unjam_ms    = 700U, .unjam_pm = 600, .retry_max = 4U,
	.slew_rpm_per_s = 20U,                 /* 0->40 rpm in ~2 s (blade rpm)    */
	.period_ms   = 100U, .wake_mask_ms = 50U,
};

BldcCtrl_t g_grind_ctrl = { .cfg = &grind_cfg };
BldcCtrl_t g_stir_ctrl  = { .cfg = &stir_cfg  };

/* ---- Helpers ------------------------------------------------------------- */

static void bldc_set_dir(BldcCtrl_t *c, uint8_t reverse)
{
	DRV8306_SetDirection(c->cfg->h,
	                     reverse ? DRV8306_DIR_CCW : DRV8306_DIR_CW);
	c->dir_ccw = reverse ? 1U : 0U;   /* ★C088: position 부호 = 실제로 건 방향 */
}

/* ★R3 개정4 C088·C089 — position 누적 + 마지막 움직임 시각. 매 Tick 호출. */
static void bldc_track_motion(BldcCtrl_t *c, uint32_t now_ms)
{
	uint32_t fg = c->cfg->h->fg_edges;          /* ISR 증가, 32bit 원자 읽기 */
	uint32_t d  = fg - c->pos_fg_last;           /* uint32 모듈러             */
	if (d != 0U)
	{
		c->pos_fg_last = fg;
		c->fg_move_ms  = now_ms;
		if (c->dir_ccw) { c->position -= d; } else { c->position += d; }
	}
}

static void bldc_pi_reset(BldcCtrl_t *c)
{
	const BldcCtrl_Cfg_t *k = c->cfg;
	DRV8306_PI_Init(&c->pi, k->kp_num, k->kp_den, k->ki_num, k->ki_den,
	                k->duty_min_pm, k->duty_max_pm);
	c->stall_active = 0U;
}

/* ---- Lifecycle ----------------------------------------------------------- */
void BldcCtrl_Init(BldcCtrl_t *c)
{
	const BldcCtrl_Cfg_t *k = c->cfg;

	c->target_out_rpm = k->ladder[0];
	c->sp_out_rpm     = 0U;
	c->meas_out_rpm   = 0U;
	c->duty_pm        = 0;
	c->running        = 0U;
	c->reverse        = 0U;
	c->state          = (uint8_t)BLDC_IDLE;
	c->fault          = 0U;
	c->retry_cnt      = 0U;

	c->spd_idx     = 0U;
	c->wake_tick   = 0U;
	c->unjam_until = 0U;
	c->stall_since = 0U;
	c->last_tick   = HAL_GetTick();
	bldc_pi_reset(c);

	/* ★C088·C089: 위치·정지 관측 초기화 */
	c->position    = 0U;
	c->rest_ms     = (uint16_t)BLDC_REST_MS;
	c->pos_fg_last = k->h->fg_edges;
	c->fg_move_ms  = c->last_tick;
	c->dir_ccw     = 0U;

	/* The motor itself is left parked/disabled by DRV8306_InitAll(). */
}

/* ---- Panel actions ------------------------------------------------------- */
void BldcCtrl_Start(BldcCtrl_t *c, uint8_t reverse)
{
	const BldcCtrl_Cfg_t *k = c->cfg;

	if (c->running)
	{
		return;                              /* start only while stopped       */
	}
	if (c->state == (uint8_t)BLDC_LOCKED)
	{
		return;                              /* soft-locked: Stop() to re-arm  */
	}

	/* Resume the LAST speed rung chosen with SW7/SW8: spd_idx persists across
	 * Stop, so a restart returns to the previous RPM instead of the bottom rung.
	 * The power-on default is rung 0 (set once in BldcCtrl_Init). */
	c->target_out_rpm = k->ladder[c->spd_idx];
	c->sp_out_rpm     = 0U;                  /* soft-start: ramp up from rest  */
	c->reverse        = reverse ? 1U : 0U;
	c->retry_cnt      = 0U;
	bldc_pi_reset(c);

	DRV8306_ClearFault(k->h);                /* clear stale faults on (re)start */
	DRV8306_ReleaseBrake(k->h);
	bldc_set_dir(c, c->reverse);
	DRV8306_Enable(k->h);                    /* wake                            */

	/* Zero the FGOUT window so the first measured-RPM sample is not skewed by
	 * edges counted while idle. */
	k->h->fg_last = k->h->fg_edges;

	/* Seed duty from the (still-zero) setpoint feed-forward: the setpoint slews
	 * up from 0 over the first control ticks, so the drive rises gradually
	 * rather than slamming the full target duty at t=0 (soft start). */
	c->duty_pm = DRV8306_FeedForwardPerMille(k->h, c->sp_out_rpm);
	DRV8306_SetDutyPerMille(k->h, (uint16_t)c->duty_pm);

	c->wake_tick = HAL_GetTick();            /* open the wake-fault mask window */
	c->last_tick = c->wake_tick;             /* first PI window is a full period*/
	c->running   = 1U;
	c->state     = (uint8_t)BLDC_RUN;
}

/* 폴트 래치 해제. 근거/제약은 헤더 주석 참조. */
void BldcCtrl_ClearFault(BldcCtrl_t *c)
{
	if (c->running != 0U)
		return;                              /* 구동 중에는 건드리지 않는다 */

	DRV8306_ClearFault(c->cfg->h);           /* ENABLE 펄스 -> 래치 리셋     */
	c->fault     = 0U;
	c->retry_cnt = 0U;
	if (c->state == (uint8_t)BLDC_LOCKED)
		c->state = (uint8_t)BLDC_IDLE;       /* 소프트락도 함께 해제         */
}

void BldcCtrl_Stop(BldcCtrl_t *c)
{
	const BldcCtrl_Cfg_t *k = c->cfg;

	DRV8306_Stop(k->h);                      /* coast (duty 0) + sleep          */
	c->running        = 0U;
	/* Keep c->spd_idx so the next Start resumes the last-selected RPM rung. */
	c->target_out_rpm = k->ladder[c->spd_idx];
	c->sp_out_rpm     = 0U;
	c->retry_cnt      = 0U;
	c->duty_pm        = 0;
	c->state          = (uint8_t)BLDC_IDLE;  /* Stop clears any soft-lock       */
	bldc_pi_reset(c);
}

/* Emergency short-brake. Unlike Stop() the driver is left AWAKE (ENABLE HIGH):
 * nBRAKE only shorts the phases while the gate driver is powered, so sleeping
 * here would silently turn the brake into a coast. The controller is marked
 * IDLE so BldcCtrl_Tick() stops driving duty; target is forced to 0 (not the
 * ladder rung) so nothing restarts it implicitly. Must be paired with
 * BldcCtrl_BrakeRelease() -- see the header. */
void BldcCtrl_BrakeStop(BldcCtrl_t *c)
{
	const BldcCtrl_Cfg_t *k = c->cfg;

	DRV8306_Enable(k->h);                    /* brake needs the driver awake    */
	DRV8306_Brake(k->h);                     /* duty 0 + nBRAKE LOW (short)     */
	c->running        = 0U;
	c->target_out_rpm = 0U;
	c->sp_out_rpm     = 0U;
	c->duty_pm        = 0;
	c->retry_cnt      = 0U;
	c->stall_active   = 0U;
	c->state          = (uint8_t)BLDC_IDLE;  /* also clears any soft-lock       */
	bldc_pi_reset(c);
}

/* Release the short brake and settle into the normal stopped state (duty 0 +
 * sleep, ladder rung restored, PI reset). */
void BldcCtrl_BrakeRelease(BldcCtrl_t *c)
{
	DRV8306_ReleaseBrake(c->cfg->h);
	BldcCtrl_Stop(c);
}

void BldcCtrl_SpeedStep(BldcCtrl_t *c)
{
	const BldcCtrl_Cfg_t *k = c->cfg;

	c->spd_idx = (uint8_t)((c->spd_idx + 1U) % k->ladder_len);
	c->target_out_rpm = k->ladder[c->spd_idx];
	/* The PI tracks the new target live on the next tick; no integ reset. */
}

uint8_t BldcCtrl_IsRunning(const BldcCtrl_t *c)
{
	return c->running;
}

uint32_t BldcCtrl_Position(const BldcCtrl_t *c)
{
	return c->position;
}

uint8_t BldcCtrl_IsAtRest(const BldcCtrl_t *c, uint32_t now_ms)
{
	return (uint8_t)(!c->running && ((now_ms - c->fg_move_ms) >= (uint32_t)c->rest_ms));
}

void BldcCtrl_CoastAwake(BldcCtrl_t *c)
{
	const BldcCtrl_Cfg_t *k = c->cfg;

	/* ENABLE·nBRAKE 는 건드리지 않는다 — 구동 중이었다면 이미 HIGH 다. duty 만 0. */
	DRV8306_SetDutyPerMille(k->h, 0U);
	c->running        = 0U;
	c->target_out_rpm = k->ladder[c->spd_idx];   /* Stop() 과 같은 규칙 */
	c->sp_out_rpm     = 0U;
	c->retry_cnt      = 0U;
	c->duty_pm        = 0;
	c->stall_active   = 0U;
	c->state          = (uint8_t)BLDC_IDLE;
	bldc_pi_reset(c);
}

/* ---- Control tick -------------------------------------------------------- */
void BldcCtrl_Tick(BldcCtrl_t *c, uint32_t now_ms)
{
	const BldcCtrl_Cfg_t  *k = c->cfg;
	DRV8306_HandleTypeDef *h = k->h;
	uint32_t dt;
	uint8_t  faulted;

	bldc_track_motion(c, now_ms);   /* ★C088·C089: 주기 게이트 앞, 매 호출 */

	/* --- nFAULT handling runs EVERY call, BEFORE the control-period gate. Two
	 * reasons it must not be gated behind the 100 ms period: (1) the wake window
	 * below is shorter than a period, so gating would let it expire unseen and
	 * latch a false soft-lock; (2) a genuine fault then stops the motor within a
	 * poll, not a period.
	 *
	 * WAKE WINDOW: for wake_mask_ms after a start the DRV8306 legitimately holds
	 * nFAULT LOW until its charge pump / DVDD come up (tWAKE) -- this is NOT a
	 * fault. So within that window IGNORE nFAULT entirely (clear the latch, do
	 * not trip), regardless of pin level. The device's own hardware OCP still
	 * protects the power stage during this time, and a real fault persists, so
	 * it is caught the instant the window closes. (Reading "pin already HIGH"
	 * is unreliable here: at ~1 ms polling the first check can fall inside the
	 * legitimate tWAKE LOW and would otherwise trip immediately.) */
	faulted = DRV8306_IsFault(h);
	if (c->running &&
	    (uint32_t)(now_ms - c->wake_tick) < k->wake_mask_ms)
	{
		h->fault = 0U;                       /* wake settle: nFAULT not valid  */
		faulted  = 0U;
	}
	c->fault = faulted;

	if (faulted && c->running)
	{
		DRV8306_Stop(h);                     /* real fault: soft-lock, re-arm  */
		c->running = 0U;
		c->state   = (uint8_t)BLDC_LOCKED;
	}

	/* Control law self-times to the cadence; nFAULT above is handled every poll. */
	dt = now_ms - c->last_tick;
	if (dt < k->period_ms)
	{
		return;
	}
	c->last_tick = now_ms;

	/* Measured RPM (advances the FGOUT window by dt); kept fresh while stopped. */
	c->meas_out_rpm = DRV8306_MeasuredOutputRPM(h, dt);

	switch ((bldc_state_t)c->state)
	{
	case BLDC_RUN:
	{
		int16_t ff;
		int32_t err;
		int16_t duty;

		/* Slew the setpoint toward the commanded target at slew_rpm_per_s so a
		 * button/target change (or a soft start from 0) accelerates/decelerates
		 * gradually. The PI tracks this slewed setpoint, not the raw target, so
		 * there is no error step and hence no duty/current spike. */
		{
			uint32_t step = ((uint32_t)k->slew_rpm_per_s * dt) / 1000U;
			if (step == 0U)
			{
				step = 1U;                   /* guarantee progress each tick   */
			}
			if (c->sp_out_rpm < c->target_out_rpm)
			{
				uint16_t gap = (uint16_t)(c->target_out_rpm - c->sp_out_rpm);
				c->sp_out_rpm += (uint16_t)((gap < step) ? gap : step);
			}
			else if (c->sp_out_rpm > c->target_out_rpm)
			{
				uint16_t gap = (uint16_t)(c->sp_out_rpm - c->target_out_rpm);
				c->sp_out_rpm -= (uint16_t)((gap < step) ? gap : step);
			}
		}

		ff   = DRV8306_FeedForwardPerMille(h, c->sp_out_rpm);
		err  = (int32_t)c->sp_out_rpm - (int32_t)c->meas_out_rpm;
		duty = DRV8306_PI_Compute(&c->pi, err, ff, dt);

		DRV8306_SetDutyPerMille(h, (uint16_t)duty);
		c->duty_pm = duty;

		/* Stall detect against the SETPOINT (not the final target): during a
		 * ramp the motor legitimately lags the target, so comparing to the
		 * live setpoint avoids false trips while still catching a real jam. */
		{
			uint16_t thresh = (uint16_t)(((uint32_t)c->sp_out_rpm
			                              * k->stall_pct) / 100U);
			uint8_t stalling = (uint8_t)((c->meas_out_rpm < thresh) &&
			                             (duty >= k->duty_sat_pm));

			if (stalling)
			{
				if (!c->stall_active)
				{
					c->stall_active = 1U;
					c->stall_since  = now_ms;
				}
				else if ((uint32_t)(now_ms - c->stall_since) >= k->stall_ms)
				{
					/* Jam confirmed. */
					c->stall_active = 0U;
					c->retry_cnt++;

					if (c->retry_cnt > k->retry_max)
					{
						DRV8306_Stop(h);              /* give up: soft-lock */
						c->running = 0U;
						c->state   = (uint8_t)BLDC_LOCKED;
					}
					else
					{
						/* Reverse-push to break the jam at a fixed,
						 * HW-current-limited duty. */
						bldc_set_dir(c, (uint8_t)(!c->reverse));
						DRV8306_SetDutyPerMille(h, (uint16_t)k->unjam_pm);
						c->duty_pm     = k->unjam_pm;
						c->unjam_until = now_ms + k->unjam_ms;
						c->state       = (uint8_t)BLDC_UNJAM;
					}
				}
			}
			else
			{
				c->stall_active = 0U;        /* condition broke: reset timer   */
			}
		}
		break;
	}

	case BLDC_UNJAM:
		if ((int32_t)(now_ms - c->unjam_until) >= 0)
		{
			/* Push done: resume normal run in the base direction, fresh PI, and
			 * re-ramp the setpoint from 0 so it re-accelerates gently. */
			bldc_set_dir(c, c->reverse);
			bldc_pi_reset(c);
			c->sp_out_rpm = 0U;
			c->state = (uint8_t)BLDC_RUN;
		}
		break;

	case BLDC_LOCKED:
	case BLDC_IDLE:
	default:
		break;                               /* nothing to drive */
	}
}
