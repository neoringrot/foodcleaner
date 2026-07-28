#include "tb_lift.h"

#include "cmsis_os.h"
#include "lift_motor.h"

/* Runtime switches -- set from the debugger while running. Default off/safe. */
volatile uint8_t tb_lift_enable  = 0;
volatile uint8_t tb_lift_reverse = 0;

/* Track previous enable state so VM is switched only on transitions (avoids
 * re-toggling the enable GPIO every poll). */
static uint8_t lift_running = 0;

void TB_Lift_Init(void)
{
	/* Lift_Init() (VM off, coast) is performed in main.c USER CODE BEGIN 2,
	 * which always runs before the scheduler; here we only reset run-state. */
	lift_running = 0;
}

void TB_Lift_Poll(void)
{
	/* ---- U6 Lift ---- */
	if (tb_lift_enable)
	{
		if (!lift_running)
		{
			Lift_Enable();       /* VM on before driving */
			lift_running = 1;
		}
		if (tb_lift_reverse)
		{
			Lift_Down();
		}
		else
		{
			Lift_Up();
		}
	}
	else if (lift_running)
	{
		Lift_Stop();             /* coast */
		Lift_Disable();          /* VM off */
		lift_running = 0;
	}
}

void TB_Lift_Loop(void)
{
	for (;;)
	{
		TB_Lift_Poll();
		osDelay(10);
	}
}
