#ifndef SRC_TB_SPEAKER_H_
#define SRC_TB_SPEAKER_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Testbed for the U15 LM4871 speaker path (SPK-DAC on PA4/DAC_OUT1, SPK-EN on
 * PA3/o_EN_SPK, active-low shutdown). It exercises both control pins end to end:
 * SPK-EN un-mutes the amplifier and SPK-DAC carries the test tone.
 *
 * Usage (e.g. from a task in freertos.c):
 *     TB_Speaker_Init();
 *     for(;;) { TB_Speaker_Poll(); osDelay(10); }
 * or just call the ready-made loop:
 *     TB_Speaker_Init();
 *     TB_Speaker_Loop();     // never returns
 *
 * tb_speaker_enable is the runtime switch: set it to 1 in the debugger (live
 * watch / expression) and a repeating low-volume test tone plays (SPK-EN driven
 * active, SPK-DAC bit-banged); set it back to 0 and the amplifier is muted
 * (SPK-EN idle, DAC parked at mid-scale). Frequency, amplitude and the on/off
 * burst timing are also tweakable live.
 *
 * Volume note: tb_speaker_amp is the DAC peak swing around mid-scale, deliberately
 * defaulted LOW (TB_SPEAKER_LOW_AMP) so the test tone is audible but quiet. The
 * driver clamps it so mid +/- amp always stays inside the 12-bit DAC range. */

#define TB_SPEAKER_LOW_AMP   300u   /* quiet default peak swing around DAC mid   */
#define TB_SPEAKER_DEF_FREQ 2000u   /* default test-tone frequency (Hz)          */
#define TB_SPEAKER_DEF_ON    150u   /* tone burst length (ms)                    */
#define TB_SPEAKER_DEF_OFF   350u   /* silence between bursts (ms)               */

extern volatile uint8_t  tb_speaker_enable;  /* 1 = play test tone, 0 = mute      */
extern volatile uint16_t tb_speaker_freq;    /* tone frequency (Hz)               */
extern volatile uint16_t tb_speaker_amp;     /* DAC peak swing around mid (quiet) */
extern volatile uint16_t tb_speaker_on_ms;   /* burst length (ms)                 */
extern volatile uint16_t tb_speaker_off_ms;  /* gap between bursts (ms)           */

void TB_Speaker_Init(void);
void TB_Speaker_Poll(void);   /* apply current enable state once (non-latching) */
void TB_Speaker_Loop(void);   /* Init already done; poll forever               */

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_SPEAKER_H_ */
