#ifndef DEVICES_LIFT_H_
#define DEVICES_LIFT_H_

#include "drv8871.h"

#ifdef __cplusplus
extern "C" {
#endif

/* U6 -- Lift DRV8871 (schematic net "LIFT").
 *
 * R1 change: the lift's IN1/IN2 moved from PG3/PG4 (which had NO timer channel,
 * so the old build drove them as plain GPIO -- full-speed only) to PB8/PB9 =
 * TIM4_CH3/CH4. The lift is now a TIM4 PWM driver, identical in structure to the
 * TIM3 doors (U5 wdoor, U7 tdoor), so it shares the DRV8871 20 kHz PWM engine
 * and gains open-loop speed control.
 *
 *   IN1 = PB8  tim4_LIFT_IN1  = TIM4_CH3   (net LIFT-IN1)
 *   IN2 = PB9  tim4_LIFT_IN2  = TIM4_CH4   (net LIFT-IN2)
 *   EN  = PB12 o_MTR_DC_LIFT              (net MTR-DC-LIFT)
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
 * The public API keeps the original no-argument shape (full-speed Up/Down); it
 * now routes through the TIM4 PWM channels at 100% duty, behaviourally identical
 * to the old GPIO drive. Use Lift_UpSpeed()/Lift_DownSpeed() for open-loop speed
 * control (0-100%).
 *
 * Up/Down direction is provisional -- swap the two bodies in lift_motor.c if the
 * lift runs the wrong way on the bench. */

void Lift_Init(void);
void Lift_Enable(void);   /* switch 24V onto VM */
void Lift_Disable(void);  /* remove VM          */
void Lift_Up(void);       /* forward, full speed */
void Lift_Down(void);     /* reverse, full speed */
void Lift_UpSpeed(uint8_t duty_pct);   /* forward at 0-100% */
void Lift_DownSpeed(uint8_t duty_pct); /* reverse at 0-100% */
void Lift_Brake(void);
void Lift_Stop(void);     /* coast */


/* ---- 모니터링 read-only (protocol_r0 0x26 OUTPUT) ---------------------
 * drive = drv8871_drive_t (0 코스트 / 1 정회전 / 2 역회전 / 3 제동),
 * duty  = 마지막으로 지령한 PWM [%]. 피드백이 아니라 "마지막 지령"이다. */
uint8_t Lift_GetDrive(void);
uint8_t Lift_GetDuty(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_LIFT_H_ */
