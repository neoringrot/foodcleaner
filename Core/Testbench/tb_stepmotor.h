#ifndef SRC_TB_STEPMOTOR_H_
#define SRC_TB_STEPMOTOR_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Testbed for the two 4-phase unipolar steppers (both 24 V, but DIFFERENT parts):
 *   STEP1 = 24BYJ48-895 : PD8..11  -> Q33/Q32/Q31/Q30 -> connector J31
 *   STEP2 = 35BYJ46-1014: PD12..15 -> Q41/Q40/Q39/Q38 -> connector J33
 * Phase M1..M4 = connector pins 2..5, pin1 = common +24V.
 *   STEP1 pins 1..5 = Red(common)/Orange/Yellow/Pink/Blue.
 *   STEP2 pins 1..5 = Yellow(common)/Red/Orange/Blue/Pink  (confirmed 2026-08-15;
 *                     STEP2 common is YELLOW, not red).
 * Both rotate with the natural M1..M4 order. See step_motor.h for the rationale.
 * Mirrors tb_drv8871's live-switch style.
 *
 * Usage (e.g. from StartDefaultTask in freertos.c):
 *     TB_StepMotor_Init();
 *     for(;;) { TB_StepMotor_Poll(); osDelay(1); }
 * or the ready-made loop:
 *     TB_StepMotor_Init();
 *     TB_StepMotor_Loop();     // never returns
 *
 * The tb_stepN_enable flags are the runtime switches: set one to 1 in the
 * debugger (live watch / expression) and that motor spins for a FIXED time,
 * then stops and the flag self-clears back to 0 (set it to 1 again to repeat).
 * Writing 0 early aborts the run. Direction, step period (speed) and
 * hold-on-stop are tweakable live.
 *
 * Run times (2026-08-25 bench request):
 *   STEP1 : 15 s regardless of direction  (tb_step1_run_ms)
 *   STEP2 : dir=1 (close) 2 s             (tb_step2_run_close_ms)
 *           dir=0 (open)  1 s             (tb_step2_run_open_ms)
 * Direction meaning on both motors: dir 0 = 열림(open), dir 1 = 닫힘(close).
 * The duration is latched at the moment enable goes 1, so changing dir mid-run
 * does not change the already-started run length.
 *
 * Pacing (the L6470's internal speed engine has no equivalent here) is done in
 * TB_StepMotor_Poll() off HAL_GetTick(): one full step is issued every
 * tb_stepN_period_ms. Poll must be called at least that often -- osDelay(1) is
 * fine. Smaller period = faster; too small for an unknown motor loses steps. */

extern volatile uint8_t  tb_step1_enable;    /* 1 = start a timed run, self-clears at the end */
extern volatile uint8_t  tb_step1_dir;       /* 0 = 열림(open), 1 = 닫힘(close) */
extern volatile uint16_t tb_step1_period_ms; /* ms per full step (>=1); speed knob */
extern volatile uint8_t  tb_step1_hold;      /* 0 = release coils on stop, 1 = hold torque */
extern volatile uint32_t tb_step1_run_ms;    /* run length, both directions (default 15000) */

extern volatile uint8_t  tb_step2_enable;    /* 1 = start a timed run, self-clears at the end */
extern volatile uint8_t  tb_step2_dir;       /* 0 = 열림(open), 1 = 닫힘(close) */
extern volatile uint16_t tb_step2_period_ms; /* ms per full step (>=1); speed knob */
extern volatile uint8_t  tb_step2_hold;      /* 0 = release coils on stop, 1 = hold torque */
extern volatile uint32_t tb_step2_run_open_ms;  /* dir=0 열림 run length (default 1000)  */
extern volatile uint32_t tb_step2_run_close_ms; /* dir=1 닫힘 run length (default 2000)  */

void TB_StepMotor_Init(void);
void TB_StepMotor_Poll(void);   /* apply current enable/dir/speed state; call frequently */
void TB_StepMotor_Loop(void);   /* Init already done; poll forever */

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_STEPMOTOR_H_ */
