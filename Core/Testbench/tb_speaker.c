#include "tb_speaker.h"

#include "cmsis_os.h"
#include "lm4871.h"   /* board singleton lm4871, LM4871_Enable/Disable/BeepEx */

/* Runtime switches -- set from the debugger while running. Default off/safe.
 * tb_speaker_amp defaults LOW so the test tone is quiet, per the request. */
volatile uint8_t  tb_speaker_enable = 0;
volatile uint16_t tb_speaker_freq   = TB_SPEAKER_DEF_FREQ;
volatile uint16_t tb_speaker_amp    = TB_SPEAKER_LOW_AMP;
volatile uint16_t tb_speaker_on_ms  = TB_SPEAKER_DEF_ON;
volatile uint16_t tb_speaker_off_ms = TB_SPEAKER_DEF_OFF;

/* Track previous enable state so SPK-EN is toggled only on transitions (the amp
 * is un-muted once, not on every burst -- avoids the CB turn-on click per tone). */
static uint8_t  spk_running = 0;
static uint32_t next_burst_ms = 0;   /* HAL_GetTick() deadline for the next tone */

void TB_Speaker_Init(void)
{
	/* Idempotent: LM4871_BoardInit() already ran in main.c, but calling it again
	 * is safe and keeps this testbed self-contained. Leaves the amp muted. */
	LM4871_BoardInit();
	spk_running   = 0;
	next_burst_ms = 0;
}

void TB_Speaker_Poll(void)
{
	if (tb_speaker_enable)
	{
		if (!spk_running)
		{
			/* SPK-EN: drive the amplifier active (un-mute) once, before playing.
			 * settle_ms in the driver ramps CB to suppress the turn-on pop. */
			LM4871_Enable(&lm4871);
			spk_running   = 1;
			next_burst_ms = HAL_GetTick();   /* first burst immediately */
		}

		/* Space the bursts by on_ms + off_ms without a running blocking loop: emit
		 * one short tone when the deadline passes, then schedule the next. */
		if ((int32_t)(HAL_GetTick() - next_burst_ms) >= 0)
		{
			/* SPK-DAC: bit-bang the low-volume square wave. leave_enabled = 1 keeps
			 * SPK-EN active between bursts so we don't click on every repeat. */
			LM4871_BeepEx(&lm4871, tb_speaker_freq, tb_speaker_on_ms,
			              tb_speaker_amp, 1u);
			next_burst_ms = HAL_GetTick() + tb_speaker_off_ms;
		}
	}
	else if (spk_running)
	{
		/* SPK-EN: mute the amplifier (idle level) and park SPK-DAC at mid-scale. */
		LM4871_Disable(&lm4871);
		spk_running = 0;
	}
}

void TB_Speaker_Loop(void)
{
	for (;;)
	{
		TB_Speaker_Poll();
		osDelay(10);
	}
}
