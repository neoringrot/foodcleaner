#ifndef SRC_TB_LIFT_H_
#define SRC_TB_LIFT_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Testbed for the U6 Lift DRV8871 (schematic net "LIFT").
 *
 * The lift's IN1/IN2 are on PG3/PG4, which have no timer channel on the
 * STM32F103ZET6, so hardware PWM (as the TIM3 doors use) is impossible. Speed
 * is instead produced by SOFTWARE PWM in TB_Lift_Poll(): within each window of
 * tb_lift_pwm_period_ms the active input is driven for (tb_lift_duty %) of the
 * window and coasted for the rest (DRV8871 coast-decay control). Because the
 * pulse is quantised to the ~1 ms poll cadence, the period sets the trade-off:
 * 10 ms -> 100 Hz / ~10 % duty steps, 20 ms -> 50 Hz / ~5 % steps.
 *
 * Usage (e.g. from StartMotorTask in freertos.c), poll at ~1 ms so the software
 * PWM is accurate:
 *     TB_Lift_Init();
 *     for(;;) { TB_Lift_Poll(); osDelay(1); }
 * or just call the ready-made loop:
 *     TB_Lift_Init();
 *     TB_Lift_Loop();     // never returns
 *
 * Set tb_lift_enable to 1 in the debugger (live watch / expression) and the
 * lift runs; set it back to 0 and it stops. Direction (tb_lift_reverse), speed
 * (tb_lift_duty) and PWM period (tb_lift_pwm_period_ms) are all tweakable live.
 * NOTE: changing tb_lift_reverse while running reverses the motor immediately;
 * prefer to change direction only at duty 0 / while stopped. */

extern volatile uint8_t  tb_lift_enable;        /* 1 = run U6 lift, 0 = stop      */
extern volatile uint8_t  tb_lift_reverse;       /* 0 = Up dir, 1 = Down dir       */
extern volatile uint8_t  tb_lift_duty;          /* speed 0..100 % (0 coast)       */
extern volatile uint16_t tb_lift_pwm_period_ms; /* software-PWM period, >=1 ms    */

void TB_Lift_Init(void);
void TB_Lift_Poll(void);   /* apply current enable/direction/duty state once */
void TB_Lift_Loop(void);   /* Init already done; poll forever                */

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_LIFT_H_ */
