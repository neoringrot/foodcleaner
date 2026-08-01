#include "drv8306.h"

/* Board wiring, taken from the netlist (see header for the pin map).
 * Reset defaults (gpio.c): ENABLE = LOW (sleep, safe), DIR = LOW,
 * nBRAKE = HIGH (not braking). This driver keeps that safe state until a run
 * is explicitly commanded. */
DRV8306_HandleTypeDef drv8306_m1 =
{
	.htim        = &htim1,
	.pwm_ch      = TIM_CHANNEL_1,          /* PE9  tim1_M1_PWM  */
	.en_port     = o_M1_ENABLE_GPIO_Port,  .en_pin     = o_M1_ENABLE_Pin,      /* PE7  */
	.dir_port    = o_M1_DIR_GPIO_Port,     .dir_pin    = o_M1_DIR_Pin,         /* PE10 */
	.nbrake_port = o_M1_nBRAKE_GPIO_Port,  .nbrake_pin = o_M1_nBRAKE_Pin,      /* PE13 */
	.nfault_port = exti14_M1_nFAULT_GPIO_Port, .nfault_pin = exti14_M1_nFAULT_Pin, /* PF14 */
	.fgout_port  = exti15_M1_FGOT_GPIO_Port,   .fgout_pin  = exti15_M1_FGOT_Pin,   /* PF15 */
	.pole_pairs  = DRV8306_DEFAULT_POLE_PAIRS,
	.gear_ratio  = DRV8306_M1_GEAR_RATIO,      /* direct drive           */
	.noload_rpm  = DRV8306_MOTOR_NOLOAD_RPM,   /* JK60BLS03: 4000 RPM    */
};

DRV8306_HandleTypeDef drv8306_m2 =
{
	.htim        = &htim1,
	.pwm_ch      = TIM_CHANNEL_2,          /* PE11 tim1_M2_PWM  */
	.en_port     = o_M2_ENABLE_GPIO_Port,  .en_pin     = o_M2_ENABLE_Pin,      /* PE8  */
	.dir_port    = o_M2_DIR_GPIO_Port,     .dir_pin    = o_M2_DIR_Pin,         /* PE12 */
	.nbrake_port = o_M2_nBRAKE_GPIO_Port,  .nbrake_pin = o_M2_nBRAKE_Pin,      /* PE14 */
	.nfault_port = exti13_M2_nFAULT_GPIO_Port, .nfault_pin = exti13_M2_nFAULT_Pin, /* PF13 */
	.fgout_port  = exti12_M2_FGOT_GPIO_Port,   .fgout_pin  = exti12_M2_FGOT_Pin,   /* PF12 */
	.pole_pairs  = DRV8306_DEFAULT_POLE_PAIRS,
	.gear_ratio  = DRV8306_M2_GEAR_RATIO,      /* 1:49 planetary gearbox */
	.noload_rpm  = DRV8306_M2_NOLOAD_RPM,      /* JK42BLS02: 5000 RPM    */
};

static uint32_t DRV8306_DutyTicks(const DRV8306_HandleTypeDef *h, uint8_t duty_pct)
{
	if (duty_pct > 100U)
	{
		duty_pct = 100U;
	}
	/* Round to nearest tick so 100 % lands exactly on ARR+1 (full on). */
	return ((h->arr + 1U) * duty_pct + 50U) / 100U;
}

void DRV8306_Init(DRV8306_HandleTypeDef *h)
{
	h->arr = (SystemCoreClock / DRV8306_PWM_FREQ_HZ) - 1U;

	/* ARR is timer-wide: both DRV8306 instances share TIM1 and the same
	 * frequency, so setting it from either Init is consistent. */
	__HAL_TIM_SET_AUTORELOAD(h->htim, h->arr);
	__HAL_TIM_SET_COMPARE(h->htim, h->pwm_ch, 0U);   /* 0 % duty: PWM held low */

	/* Start the PWM channel once and leave it running; duty 0 holds the pin
	 * low so the DRV8306 sees a valid (stopped) PWM input rather than a
	 * floating one. */
	HAL_TIM_PWM_Start(h->htim, h->pwm_ch);

	/* Safe idle state: not braking, direction CW, device asleep. */
	HAL_GPIO_WritePin(h->nbrake_port, h->nbrake_pin, GPIO_PIN_SET);   /* run mode  */
	HAL_GPIO_WritePin(h->dir_port, h->dir_pin, GPIO_PIN_RESET);       /* DIR = CW  */
	HAL_GPIO_WritePin(h->en_port, h->en_pin, GPIO_PIN_RESET);         /* sleep     */

	h->fg_edges = 0U;
	h->fg_last  = 0U;
	h->fault    = 0U;
	h->enabled  = 0U;
	h->meas_rpm = 0U;
}

void DRV8306_InitAll(void)
{
	DRV8306_Init(&drv8306_m1);
	DRV8306_Init(&drv8306_m2);
}

void DRV8306_Enable(DRV8306_HandleTypeDef *h)
{
	/* Rising edge on ENABLE wakes the device; tWAKE (~100 us) must pass before
	 * inputs are honoured. Callers drive duty on the next poll (>=1 ms later),
	 * so no explicit wait is needed here. */
	HAL_GPIO_WritePin(h->en_port, h->en_pin, GPIO_PIN_SET);
	h->enabled = 1U;
}

void DRV8306_Disable(DRV8306_HandleTypeDef *h)
{
	HAL_GPIO_WritePin(h->en_port, h->en_pin, GPIO_PIN_RESET);   /* enter sleep */
	h->enabled = 0U;
}

void DRV8306_SetDirection(DRV8306_HandleTypeDef *h, DRV8306_Direction dir)
{
	HAL_GPIO_WritePin(h->dir_port, h->dir_pin,
	                  (dir == DRV8306_DIR_CCW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void DRV8306_SetDuty(DRV8306_HandleTypeDef *h, uint8_t duty_pct)
{
	__HAL_TIM_SET_COMPARE(h->htim, h->pwm_ch, DRV8306_DutyTicks(h, duty_pct));
}

void DRV8306_Brake(DRV8306_HandleTypeDef *h)
{
	/* nBRAKE low short-brakes regardless of PWM; drop duty first so the phase
	 * PWM is not fighting the brake. */
	__HAL_TIM_SET_COMPARE(h->htim, h->pwm_ch, 0U);
	HAL_GPIO_WritePin(h->nbrake_port, h->nbrake_pin, GPIO_PIN_RESET);
}

void DRV8306_ReleaseBrake(DRV8306_HandleTypeDef *h)
{
	HAL_GPIO_WritePin(h->nbrake_port, h->nbrake_pin, GPIO_PIN_SET);
}

void DRV8306_Stop(DRV8306_HandleTypeDef *h)
{
	__HAL_TIM_SET_COMPARE(h->htim, h->pwm_ch, 0U);   /* 0 % duty (coast) */
	DRV8306_Disable(h);                              /* sleep            */
}

uint8_t DRV8306_RpmToDuty(uint16_t rpm)
{
	uint32_t duty;

	if (rpm <= DRV8306_MIN_RPM)
	{
		return (uint8_t)DRV8306_MIN_DUTY_PCT;
	}
	if (rpm >= DRV8306_MAX_RPM)
	{
		return 100U;
	}

	/* Linear map [MIN_RPM..MAX_RPM] -> [MIN_DUTY..100] %. */
	duty = DRV8306_MIN_DUTY_PCT
	     + (uint32_t)(rpm - DRV8306_MIN_RPM) * (100U - DRV8306_MIN_DUTY_PCT)
	       / (DRV8306_MAX_RPM - DRV8306_MIN_RPM);

	return (uint8_t)duty;
}

void DRV8306_SetSpeedRPM(DRV8306_HandleTypeDef *h, uint16_t rpm)
{
	DRV8306_SetDuty(h, DRV8306_RpmToDuty(rpm));
}

uint16_t DRV8306_MeasuredRPM(DRV8306_HandleTypeDef *h, uint32_t window_ms)
{
	uint32_t now;
	uint32_t delta;
	uint32_t divisor;

	if (window_ms == 0U)
	{
		return h->meas_rpm;
	}

	now   = h->fg_edges;                 /* volatile read; 32-bit is atomic here */
	delta = now - h->fg_last;            /* wraps correctly (unsigned)           */
	h->fg_last = now;

	/* mech RPM = edges * 60000 / (window_ms * FG_EDGES_PER_ELEC_REV * poles) */
	divisor = window_ms * DRV8306_FG_EDGES_PER_ELEC_REV
	        * (uint32_t)(h->pole_pairs ? h->pole_pairs : 1U);
	h->meas_rpm = (divisor != 0U)
	            ? (uint16_t)((delta * 60000U) / divisor)
	            : 0U;

	return h->meas_rpm;
}

/* ---- Closed-loop speed control (FGOUT PI) ------------------------------- */

static uint32_t DRV8306_DutyTicksPerMille(const DRV8306_HandleTypeDef *h,
                                          uint16_t duty_pm)
{
	if (duty_pm > 1000U)
	{
		duty_pm = 1000U;
	}
	/* Round to nearest tick so 1000 per-mille lands exactly on ARR+1 (full on). */
	return ((h->arr + 1U) * duty_pm + 500U) / 1000U;
}

void DRV8306_SetDutyPerMille(DRV8306_HandleTypeDef *h, uint16_t duty_pm)
{
	__HAL_TIM_SET_COMPARE(h->htim, h->pwm_ch,
	                      DRV8306_DutyTicksPerMille(h, duty_pm));
}

int16_t DRV8306_FeedForwardPerMille(const DRV8306_HandleTypeDef *h,
                                    uint16_t out_rpm)
{
	uint32_t gear   = (h->gear_ratio != 0U) ? h->gear_ratio : 1U;
	uint32_t noload = (h->noload_rpm != 0U) ? h->noload_rpm : DRV8306_MAX_RPM;
	uint32_t pm     = ((uint32_t)out_rpm * gear * 1000U) / noload;

	if (pm > 1000U)
	{
		pm = 1000U;
	}
	return (int16_t)pm;
}

uint16_t DRV8306_MeasuredOutputRPM(DRV8306_HandleTypeDef *h, uint32_t window_ms)
{
	uint16_t motor = DRV8306_MeasuredRPM(h, window_ms);
	uint16_t gear  = (h->gear_ratio != 0U) ? h->gear_ratio : 1U;
	return (uint16_t)(motor / gear);
}

void DRV8306_PI_Init(DRV8306_PI_t *pi,
                     int32_t kp_num, int32_t kp_den,
                     int32_t ki_num, int32_t ki_den,
                     int16_t out_min_pm, int16_t out_max_pm)
{
	pi->kp_num     = kp_num;
	pi->kp_den     = (kp_den != 0) ? kp_den : 1;
	pi->ki_num     = ki_num;
	pi->ki_den     = (ki_den != 0) ? ki_den : 1;
	pi->integ_pm   = 0;
	pi->out_min_pm = out_min_pm;
	pi->out_max_pm = out_max_pm;
	pi->duty_pm    = 0;
}

int16_t DRV8306_PI_Compute(DRV8306_PI_t *pi, int32_t err,
                           int16_t ff_pm, uint32_t dt_ms)
{
	int32_t p_pm  = (pi->kp_num * err) / pi->kp_den;
	int32_t di_pm = (pi->ki_num * err * (int32_t)dt_ms) / pi->ki_den;

	/* Tentative integrator + output, then clamp with conditional integration:
	 * the integral step is only committed if the output is inside the rails, or
	 * (on a rail) if the step unwinds toward the valid range -- classic
	 * anti-windup that stops the integrator ballooning while duty saturates. */
	int32_t integ = pi->integ_pm + di_pm;
	int32_t out   = (int32_t)ff_pm + p_pm + integ;

	if (out > pi->out_max_pm)
	{
		if (di_pm < 0)
		{
			pi->integ_pm = integ;    /* unwinding allowed */
		}
		out = pi->out_max_pm;
	}
	else if (out < pi->out_min_pm)
	{
		if (di_pm > 0)
		{
			pi->integ_pm = integ;    /* unwinding allowed */
		}
		out = pi->out_min_pm;
	}
	else
	{
		pi->integ_pm = integ;        /* not saturated: accumulate */
	}

	pi->duty_pm = (int16_t)out;
	return pi->duty_pm;
}

uint8_t DRV8306_IsFault(DRV8306_HandleTypeDef *h)
{
	return h->fault;
}

void DRV8306_ClearFault(DRV8306_HandleTypeDef *h)
{
	/* Cycling ENABLE low->high takes the device through sleep, which resets
	 * latched faults (datasheet 7.4.1.1). Preserve the prior enable state. */
	uint8_t was_enabled = h->enabled;

	HAL_GPIO_WritePin(h->en_port, h->en_pin, GPIO_PIN_RESET);
	h->fault = 0U;
	if (was_enabled)
	{
		HAL_GPIO_WritePin(h->en_port, h->en_pin, GPIO_PIN_SET);
	}
}

void DRV8306_OnEXTI(DRV8306_HandleTypeDef *h, uint16_t GPIO_Pin)
{
	if (GPIO_Pin == h->fgout_pin)
	{
		h->fg_edges++;                   /* tacho falling edge */
	}
	else if (GPIO_Pin == h->nfault_pin)
	{
		h->fault = 1U;                   /* latch fault (active-low, falling) */
	}
}

/* NOTE: the HAL_GPIO_EXTI_Callback override now lives in main.c (the single
 * central EXTI router for the whole build). It calls DRV8306_OnEXTI() for the
 * M1/M2 FGOUT and nFAULT lines, so tacho counting / fault latching are
 * unchanged; only the callback's location moved. DRV8306_OnEXTI + the
 * drv8306_m1/m2 instances are exported from drv8306.h for that purpose. */
