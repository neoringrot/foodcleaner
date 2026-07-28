#ifndef SRC_TB_STEPMOTOR_H_
#define SRC_TB_STEPMOTOR_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Testbed for the two 4-phase unipolar steppers (STEP1 PD8..11 / STEP2 PD12..15,
 * driven through Q5..8 / Q9..12). Mirrors tb_drv8871's live-switch style.
 *
 * Usage (e.g. from StartDefaultTask in freertos.c):
 *     TB_StepMotor_Init();
 *     for(;;) { TB_StepMotor_Poll(); osDelay(1); }
 * or the ready-made loop:
 *     TB_StepMotor_Init();
 *     TB_StepMotor_Loop();     // never returns
 *
 * The tb_stepN_enable flags are the runtime switches: set one to 1 in the
 * debugger (live watch / expression) and that motor spins; set it back to 0 and
 * it stops. Direction, step period (speed) and hold-on-stop are tweakable live.
 *
 * Pacing (the L6470's internal speed engine has no equivalent here) is done in
 * TB_StepMotor_Poll() off HAL_GetTick(): one full step is issued every
 * tb_stepN_period_ms. Poll must be called at least that often -- osDelay(1) is
 * fine. Smaller period = faster; too small for an unknown motor loses steps. */

extern volatile uint8_t  tb_step1_enable;    /* 1 = run STEP1, 0 = stop           */
extern volatile uint8_t  tb_step1_dir;       /* 0 = forward, 1 = reverse          */
extern volatile uint16_t tb_step1_period_ms; /* ms per full step (>=1); speed knob */
extern volatile uint8_t  tb_step1_hold;      /* 0 = release coils on stop, 1 = hold torque */

extern volatile uint8_t  tb_step2_enable;    /* 1 = run STEP2, 0 = stop           */
extern volatile uint8_t  tb_step2_dir;       /* 0 = forward, 1 = reverse          */
extern volatile uint16_t tb_step2_period_ms; /* ms per full step (>=1); speed knob */
extern volatile uint8_t  tb_step2_hold;      /* 0 = release coils on stop, 1 = hold torque */

void TB_StepMotor_Init(void);
void TB_StepMotor_Poll(void);   /* apply current enable/dir/speed state; call frequently */
void TB_StepMotor_Loop(void);   /* Init already done; poll forever */

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_STEPMOTOR_H_ */
