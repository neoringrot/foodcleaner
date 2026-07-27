#include "tb_drv8306.h"

#include "cmsis_os.h"
#include "drv8306.h"

/* Runtime switches -- set from the debugger while running. Default: disabled
 * and at the lowest speed (DRV8306_MIN_RPM), so no motor moves at power-up. */
volatile uint8_t  tb_m1_enable   = 0;
volatile uint16_t tb_m1_rpm      = DRV8306_MIN_RPM;
volatile uint8_t  tb_m1_reverse  = 0;
volatile uint16_t tb_m1_meas_rpm = 0;
volatile uint8_t  tb_m1_fault    = 0;

volatile uint8_t  tb_m2_enable   = 0;
volatile uint16_t tb_m2_rpm      = DRV8306_MIN_RPM;
volatile uint8_t  tb_m2_reverse  = 0;
volatile uint16_t tb_m2_meas_rpm = 0;
volatile uint8_t  tb_m2_fault    = 0;

/* Track previous enable state so ENABLE/DIR are set only on the start
 * transition, not re-written every poll. */
static uint8_t m1_running = 0;
static uint8_t m2_running = 0;

/* Measured-RPM window: recompute every ~200 ms using HAL_GetTick(), so the
 * result is independent of how often Poll() is called. */
#define TB_RPM_WINDOW_MS  200U
static uint32_t rpm_last_tick = 0;

static void TB_DRV8306_ServiceMotor(DRV8306_HandleTypeDef *h,
                                    volatile uint8_t  *enable,
                                    volatile uint16_t *rpm,
                                    volatile uint8_t  *reverse,
                                    volatile uint8_t  *fault_out,
                                    uint8_t           *running)
{
	if (*enable)
	{
		if (!*running)
		{
			DRV8306_ClearFault(h);          /* clear stale faults on (re)start */
			DRV8306_ReleaseBrake(h);
			DRV8306_SetDirection(h, (*reverse) ? DRV8306_DIR_CCW
			                                   : DRV8306_DIR_CW);
			DRV8306_Enable(h);              /* wake; PWM applied just below     */
			*running = 1;
		}
		/* Direction is applied on start; changing tb_mX_reverse while spinning
		 * takes effect on the next stop/start cycle (reversing a turning BLDC
		 * abruptly is intentionally avoided). */
		DRV8306_SetSpeedRPM(h, *rpm);
	}
	else if (*running)
	{
		DRV8306_Stop(h);                    /* coast (duty 0) + sleep           */
		*running = 0;
	}

	*fault_out = DRV8306_IsFault(h);
}

void TB_DRV8306_Init(void)
{
	DRV8306_InitAll();
	m1_running = 0;
	m2_running = 0;
	rpm_last_tick = HAL_GetTick();
}

void TB_DRV8306_Poll(void)
{
	uint32_t now = HAL_GetTick();

	TB_DRV8306_ServiceMotor(&drv8306_m1, &tb_m1_enable, &tb_m1_rpm,
	                        &tb_m1_reverse, &tb_m1_fault, &m1_running);
	TB_DRV8306_ServiceMotor(&drv8306_m2, &tb_m2_enable, &tb_m2_rpm,
	                        &tb_m2_reverse, &tb_m2_fault, &m2_running);

	/* Refresh measured RPM once per window. */
	if ((uint32_t)(now - rpm_last_tick) >= TB_RPM_WINDOW_MS)
	{
		uint32_t elapsed = now - rpm_last_tick;
		rpm_last_tick = now;
		tb_m1_meas_rpm = DRV8306_MeasuredRPM(&drv8306_m1, elapsed);
		tb_m2_meas_rpm = DRV8306_MeasuredRPM(&drv8306_m2, elapsed);
	}
}

void TB_DRV8306_Loop(void)
{
	for (;;)
	{
		TB_DRV8306_Poll();
		osDelay(10);
	}
}
