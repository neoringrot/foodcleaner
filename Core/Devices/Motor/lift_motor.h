#ifndef DEVICES_LIFT_H_
#define DEVICES_LIFT_H_

#include "gpio_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* U6 -- Lift DRV8871 (schematic net "LIFT"). Unlike U5 (wdoor) and U7 (tdoor),
 * whose IN1/IN2 sit on TIM3 channels (PC6/PC7, PC8/PC9) and are PWM-driven, the
 * lift's IN pins are on PG3/PG4, which have NO timer channel. So this driver
 * toggles IN1/IN2 as plain GPIO (gpio_ctrl, GPIO_MODE_OUTPUT_PP) rather than
 * PWM -- full-speed only, no duty/speed control.
 *
 *   IN1 = PG3  o_LIFT_IN1     (net LIFT-IN1)
 *   IN2 = PG4  o_LIFT_IN2     (net LIFT-IN2)
 *   EN  = PB12 o_MTR_DC_LIFT  (net MTR-DC-LIFT)
 *
 * DRV8871 IN1/IN2 truth table (datasheet Table 1):
 *   IN1  IN2  OUT1  OUT2  Mode
 *    0    0   Hi-Z  Hi-Z  Coast
 *    0    1    L     H    Reverse (Down)
 *    1    0    H     L    Forward (Up)
 *    1    1    L     L    Brake
 *
 * Enable: the DRV8871 has no logic enable pin; o_MTR_DC_LIFT gates the motor
 * supply onto VM through Q4 (NPN) -> Q2 (P-ch FET). It is active-high /
 * non-inverting: EN HIGH switches 24V onto VM, EN LOW removes it. Drive VM on
 * before commanding a direction and off after stopping.
 *
 * Up/Down direction is provisional -- swap the two bodies in lift.c if the lift
 * runs the wrong way on the bench. */

void Lift_Init(void);
void Lift_Enable(void);   /* switch 24V onto VM */
void Lift_Disable(void);  /* remove VM          */
void Lift_Up(void);
void Lift_Down(void);
void Lift_Brake(void);
void Lift_Stop(void);     /* coast */

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_LIFT_H_ */
