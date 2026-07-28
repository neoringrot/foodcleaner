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

/* Track previous enable state so VM is switched only on transitions (avoids
 * re-toggling the enable GPIO every poll). */
static uint8_t wdoor_running = 0;
static uint8_t tdoor_running = 0;

void TB_DRV8871_Init(void)
{
	WDoor_Init();
	TDoor_Init();
	wdoor_running = 0;
	tdoor_running = 0;
}

void TB_DRV8871_Poll(void)
{
	/* ---- U5 Water Door ---- */
	if (tb_wdoor_enable)
	{
		if (!wdoor_running)
		{
			WDoor_Enable();      /* VM on before driving */
			wdoor_running = 1;
		}
		if (tb_wdoor_reverse)
		{
			WDoor_Close(tb_wdoor_duty);
		}
		else
		{
			WDoor_Open(tb_wdoor_duty);
		}
	}
	else if (wdoor_running)
	{
		WDoor_Stop();            /* coast */
		WDoor_Disable();         /* VM off */
		wdoor_running = 0;
	}

	/* ---- U7 Trash Door ---- */
	if (tb_tdoor_enable)
	{
		if (!tdoor_running)
		{
			TDoor_Enable();
			tdoor_running = 1;
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
	else if (tdoor_running)
	{
		TDoor_Stop();
		TDoor_Disable();
		tdoor_running = 0;
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
