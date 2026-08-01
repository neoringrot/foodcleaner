#ifndef DEVICES_STEP_MOTOR_H_
#define DEVICES_STEP_MOTOR_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 4-phase unipolar stepper driver, direct GPIO phase sequencing.
 *
 * hasudo1 uses an L6470 (SPI dual full-bridge driver): the firmware sends
 * "Run/Move/GoTo speed-or-position" commands and the CHIP generates the coil
 * waveform internally. foodcleaner has NO driver chip -- it keeps only the
 * "output form": four transistor low-side switches per motor, driven straight
 * from GPIO, so the MCU itself must clock the phase sequence.
 *
 *   STEP1 : PD8..PD11 (o_STEP1_M1..M4) -> Q5..Q8
 *   STEP2 : PD12..PD15 (o_STEP2_M1..M4) -> Q9..Q12
 *
 * Each of the four pins energizes one motor phase (winding end pulled to ground
 * through a transistor; the classic 28BYJ-48 / ULN2003 unipolar arrangement).
 * gpio.c already configures all eight pins as push-pull outputs, LOW at boot
 * (all coils released -- safe).
 *
 * This driver is STATELESS with respect to time: StepMotor_Step() advances the
 * sequence by exactly one full step each call. The caller (a task/timer) sets
 * the step rate by how often it calls Step -- that is what replaces the L6470's
 * internal speed engine. Direction is the sign of the dir argument.
 *
 * Full-step, 2-phase-on sequence is used (two adjacent phases energized ->
 * maximum breakaway torque), mirroring the full-step choice L6470_Init() makes
 * for bring-up on hasudo1. */

/* Motor (STEP1 and STEP2): 24BYJ48, 24 VDC 4-phase unipolar PM stepper.
 * Datasheet performance parameters:
 *   Drive voltage        24 VDC
 *   Resistance           200 ohm / phase (25 C)  -> 24 V / 200 = 120 mA/phase
 *   Step angle (1-2 ph)  5.625 deg  (HALF-STEP, at the rotor)
 *   Pull-in torque       98 mN-m ;  detent (unpowered hold) 39.2 mN-m
 *   Operating freq       100 PPS (recommended self-start rate)
 *   Max pull-in freq     800 PPS (no-load, instant-start ceiling)
 *   Max pull-out freq    1000 PPS (no-load, sync-loss ceiling)
 *   Temp rise 40 K, weight 40 g
 *
 * 24 V direct-drive is safe on this board: each phase is 24 V / 200 ohm =
 * 120 mA, so the full-step 2-phase-on pattern pulls ~240 mA from the 24 V rail
 * (~5.8 W in the motor, within its 40 K rating). The FDN337N low-side switches
 * and LL4148 flyback diodes handle 120 mA/phase with margin. (Do NOT wire a
 * 5 V / ~25 ohm BYJ variant here: 24 V / 25 = ~1 A/phase would burn it out.)
 *
 * STEP-RATE LIMITS (open-loop, caller-enforced -- this driver does not gate
 * the rate). Start/stop at <= STEP_MOTOR_START_PPS_MAX and never exceed
 * STEP_MOTOR_RUN_PPS_MAX, or the motor loses sync (missed steps, no feedback
 * to detect it). One StepMotor_Step() call = one FULL step (11.25 deg at the
 * rotor -- twice the 5.625 deg half-step figure above).
 *
 * OUTPUT steps/rev: the datasheet omits the gearbox ratio. 24BYJ48 variants
 * ship as 1/32 or 1/64, giving 32 full-steps/rotor-rev * ratio = 1024 (1/32)
 * or 2048 (1/64) full-steps per OUTPUT revolution. Confirm the actual ratio
 * before using step counts for position / outflow-volume calibration. */
#define STEP_MOTOR_START_PPS_MAX  100U   /* <= self-start (pull-in) rate      */
#define STEP_MOTOR_RUN_PPS_MAX    1000U  /* <= pull-out rate (else stalls)    */

#define STEP_MOTOR_PHASES 4U

typedef struct
{
	GPIO_TypeDef *port[STEP_MOTOR_PHASES]; /* phase M1..M4 GPIO ports */
	uint16_t      pin[STEP_MOTOR_PHASES];  /* phase M1..M4 GPIO pins  */
	uint8_t       phase;                   /* current sequence index 0..3 (state) */
	uint8_t       energized;               /* 1 = a phase pattern is applied, 0 = released */
} StepMotor_HandleTypeDef;

/* Release all coils and reset the sequence index. Call once before stepping. */
void StepMotor_Init(StepMotor_HandleTypeDef *h);

/* Advance one full step: dir >= 0 forward, dir < 0 reverse. Energizes the coils. */
void StepMotor_Step(StepMotor_HandleTypeDef *h, int8_t dir);

/* Re-apply the current phase pattern (holding torque while stopped; draws
 * continuous coil current -> heat). */
void StepMotor_Hold(StepMotor_HandleTypeDef *h);

/* De-energize all four phases (stop with no holding current -- no heat). */
void StepMotor_Release(StepMotor_HandleTypeDef *h);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_STEP_MOTOR_H_ */
