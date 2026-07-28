#ifndef SRC_TB_LIFT_H_
#define SRC_TB_LIFT_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Testbed for the U6 Lift DRV8871 (schematic net "LIFT").
 *
 * Unlike the door testbed (tb_drv8871), the lift's IN1/IN2 are on PG3/PG4 with
 * no timer channel, so it is driven as plain GPIO -- full speed only, hence
 * there is no duty knob here.
 *
 * Usage (e.g. from StartMotorTask in freertos.c):
 *     TB_Lift_Init();
 *     for(;;) { TB_Lift_Poll(); osDelay(10); }
 * or just call the ready-made loop:
 *     TB_Lift_Init();
 *     TB_Lift_Loop();     // never returns
 *
 * Set tb_lift_enable to 1 in the debugger (live watch / expression) and the
 * lift runs; set it back to 0 and it stops. Direction is tweakable live via
 * tb_lift_reverse. */

extern volatile uint8_t tb_lift_enable;   /* 1 = run U6 lift, 0 = stop     */
extern volatile uint8_t tb_lift_reverse;  /* 0 = Up dir, 1 = Down dir      */

void TB_Lift_Init(void);
void TB_Lift_Poll(void);   /* apply current enable/direction state once */
void TB_Lift_Loop(void);   /* Init already done; poll forever          */

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_LIFT_H_ */
