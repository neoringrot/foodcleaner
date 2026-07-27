#include "tb_stepmotor.h"

#include "cmsis_os.h"
#include "step_motor.h"

/* Runtime switches -- set from the debugger while running. Default off/safe.
 * Default 3 ms/step (~333 full-step/s) is a conservative bring-up rate for an
 * unknown motor; raise/lower period live once rotation is confirmed. */
volatile uint8_t  tb_step1_enable    = 0;
volatile uint8_t  tb_step1_dir       = 0;
volatile uint16_t tb_step1_period_ms = 3;
volatile uint8_t  tb_step1_hold      = 0;

volatile uint8_t  tb_step2_enable    = 0;
volatile uint8_t  tb_step2_dir       = 0;
volatile uint16_t tb_step2_period_ms = 3;
volatile uint8_t  tb_step2_hold      = 0;

static StepMotor_HandleTypeDef step1 =
{
	.port = { o_STEP1_M1_GPIO_Port, o_STEP1_M2_GPIO_Port,
	          o_STEP1_M3_GPIO_Port, o_STEP1_M4_GPIO_Port },
	.pin  = { o_STEP1_M1_Pin, o_STEP1_M2_Pin,
	          o_STEP1_M3_Pin, o_STEP1_M4_Pin },
};

static StepMotor_HandleTypeDef step2 =
{
	.port = { o_STEP2_M1_GPIO_Port, o_STEP2_M2_GPIO_Port,
	          o_STEP2_M3_GPIO_Port, o_STEP2_M4_GPIO_Port },
	.pin  = { o_STEP2_M1_Pin, o_STEP2_M2_Pin,
	          o_STEP2_M3_Pin, o_STEP2_M4_Pin },
};

/* Per-motor pacing state: last-step tick and whether we are currently running
 * (so the enable->disable transition releases/holds exactly once). */
static uint32_t step1_last, step2_last;
static uint8_t  step1_running, step2_running;

void TB_StepMotor_Init(void)
{
	StepMotor_Init(&step1);
	StepMotor_Init(&step2);
	step1_last = step2_last = 0;
	step1_running = step2_running = 0;
}

void TB_StepMotor_Poll(void)
{
	uint32_t now = HAL_GetTick();

	/* ---- STEP1 ---- */
	if (tb_step1_enable)
	{
		uint16_t period = tb_step1_period_ms ? tb_step1_period_ms : 1U;
		if (!step1_running || (uint32_t)(now - step1_last) >= period)
		{
			StepMotor_Step(&step1, tb_step1_dir ? -1 : 1);
			step1_last = now;
			step1_running = 1;
		}
	}
	else if (step1_running)
	{
		if (tb_step1_hold) { StepMotor_Hold(&step1); }
		else               { StepMotor_Release(&step1); }
		step1_running = 0;
	}

	/* ---- STEP2 ---- */
	if (tb_step2_enable)
	{
		uint16_t period = tb_step2_period_ms ? tb_step2_period_ms : 1U;
		if (!step2_running || (uint32_t)(now - step2_last) >= period)
		{
			StepMotor_Step(&step2, tb_step2_dir ? -1 : 1);
			step2_last = now;
			step2_running = 1;
		}
	}
	else if (step2_running)
	{
		if (tb_step2_hold) { StepMotor_Hold(&step2); }
		else               { StepMotor_Release(&step2); }
		step2_running = 0;
	}
}

void TB_StepMotor_Loop(void)
{
	for (;;)
	{
		TB_StepMotor_Poll();
		osDelay(1);
	}
}
