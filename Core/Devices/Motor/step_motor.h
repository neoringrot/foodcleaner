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
 *   STEP1 : PD8..PD11 (o_STEP1_M1..M4) -> gate R123/R122/R121/R120
 *                                       -> Q33/Q32/Q31/Q30 (FDN337N) -> J31.2/3/4/5
 *   STEP2 : PD12..PD15 (o_STEP2_M1..M4) -> gate R131/R130/R129/R128
 *                                       -> Q41/Q40/Q39/Q38 (FDN337N) -> J33.2/3/4/5
 *   (verified against sch/R1 "FINAL NETLIST.NET", 2026-08.)
 *
 * The board CONNECTOR pinout is FIXED by the netlist and identical for J31/J33:
 *   pin1 = +24 V common (net 24P0V -- the motor's center tap MUST land here)
 *   pin2 = phase M1  (Q33 / Q41 drain, low-side switch)
 *   pin3 = phase M2  (Q32 / Q40 drain)
 *   pin4 = phase M3  (Q31 / Q39 drain)
 *   pin5 = phase M4  (Q30 / Q38 drain)
 * The handle's phase[0..3] = {M1,M2,M3,M4} is wired to pins {2,3,4,5}; which
 * wire COLOR lands on each pin depends on the motor and how its lead connector
 * is crimped (below). The two motors are DIFFERENT parts:
 *
 * -- STEP1 (J31) = 24BYJ48-895, 24 V. Datasheet lead order RED ORANGE YEL PINK
 *    BLUE already matches the board 1:1, so as crimped:
 *      pin1 RED / pin2 ORANGE(M1) / pin3 YELLOW(M2) / pin4 PINK(M3) / pin5 BLUE(M4)
 *    => phase[] = {Orange,Yellow,Pink,Blue}. The datasheet 2-2-phase table (CCW)
 *    Blue+Orange -> Orange+Yellow -> Yellow+Pink -> Pink+Blue means the stator
 *    ring IS Orange->Yellow->Pink->Blue; k_step_seq fires adjacent pairs
 *    (p0,p1)(p1,p2)(p2,p3)(p3,p0), reproducing it exactly. Natural M1..M4 order
 *    rotates -- do NOT reorder STEP1.
 *
 * -- STEP2 (J33) = 35BYJ46-1014 (35BYJ46 family; -30 class, 24 V; RED = common,
 *    phases Orange/Yellow/Blue/Pink). Its lead connector does NOT match the board
 *    as received -- the leads came ordered PINK BLUE ORANGE RED YELLOW on pins
 *    1..5, i.e. RED (common) sat on pin4 (a low-side switch) and PINK (a phase)
 *    on pin1 (+24 V). That cannot work and MUST be re-pinned so RED -> pin1.
 *    RECOMMENDED re-pin (matches the 35BYJ46 A->B->C->D = Orange->Yellow->Blue->
 *    Pink ring, so the SAME k_step_seq / natural M1..M4 order rotates):
 *      pin1 RED / pin2 ORANGE(M1) / pin3 YELLOW(M2) / pin4 BLUE(M3) / pin5 PINK(M4)
 *    Web sources disagree on the Blue/Pink pairing, so treat this as the bring-up
 *    starting point: if STEP2 only buzzes/vibrates instead of turning, swap the
 *    pin4/pin5 wires (BLUE<->PINK) -- no firmware change needed either way.
 *
 * Each of the four pins energizes one motor phase: GPIO HIGH -> FDN337N gate
 * high -> low-side N-MOSFET on -> that winding end pulled to GND while the common
 * (RED, pin1) sits at +24 V (classic 5-wire unipolar; LL4148 flyback diodes to
 * 24P0V catch each coil's turn-off spike). gpio.c already configures all eight
 * pins as push-pull outputs, LOW at boot (all coils released -- safe).
 *
 * This driver is STATELESS with respect to time: StepMotor_Step() advances the
 * sequence by exactly one full step each call. The caller (a task/timer) sets
 * the step rate by how often it calls Step -- that is what replaces the L6470's
 * internal speed engine. Direction is the sign of the dir argument.
 *
 * Full-step, 2-phase-on sequence is used (two adjacent phases energized ->
 * maximum breakaway torque), mirroring the full-step choice L6470_Init() makes
 * for bring-up on hasudo1. */

/* The two motors are DIFFERENT parts -- both 24 V, 4-phase unipolar, same driver
 * code, but different resistance / step angle / gear, so their step counts and
 * safe rates differ. Both share a common HAZARD: the connector common (pin1) is
 * hard-tied to +24 V, so NEVER fit a 5 V BYJ variant (24 V / ~25 ohm ~= 1 A/phase
 * burns it out). Confirm any replacement motor is a 24 V type.
 *
 * STEP1 = 24BYJ48-895 (Changzhou Jkongmotor;
 *         datasheet/"24BYJ48-895外形图 - English.pdf"):
 *   Drive voltage        24 VDC
 *   Resistance           150 ohm/phase +-7% (25 C) -> 160 mA/phase, ~320 mA/step
 *   Step angle / gear    11.25 deg (2-2 phase full step) , 1/48 gearbox
 *   OUTPUT steps/rev     360/11.25 * 48 = 1536 full-steps (0.234 deg/step out)
 *   Pull-in torque       >= 170 mN-m @ 330 PPS ; self-position >= 49 mN-m
 *   No-load pull-in/out  >= 400 / >= 600 Hz ; temp rise <= 75 K, noise <= 40 dB
 *
 * STEP2 = 35BYJ46-1014 (35BYJ46 family; "-1014" is a custom order code not in the
 *         public table -- specs taken from the 24 V/4-phase "-30" class, CONFIRM
 *         against the real label):
 *   Drive voltage        24 VDC
 *   Resistance           ~300 ohm/phase (25 C) -> ~80 mA/phase, ~160 mA/step
 *   Step angle / gear    7.5 deg (rotor) , ~1/85 gearbox
 *   OUTPUT steps/rev     360/7.5 * 85 ~= 4080 full-steps (~0.088 deg/step out)
 *   Torque               >= 127.4 mN-m ; self-lock >= 88.2 mN-m
 *   Start / operating    100 PPS start , >= 400 PPS operating ; noise < 40 dB
 *
 * STEP-RATE LIMITS (open-loop, caller-enforced -- this driver does not gate the
 * rate; one StepMotor_Step() = one FULL step). The macros below are a CONSERVATIVE
 * envelope safe for BOTH motors (bounded by 35BYJ46's 100 PPS start / ~400 PPS
 * run); STEP1 alone tolerates more (400 start / 600 run). Both are no-load figures
 * -- derate under mechanical load, or the motor loses sync (no feedback to catch
 * missed steps). The testbed's tb_stepN_period_ms sets the live rate. */
#define STEP_MOTOR_START_PPS_MAX  100U   /* self-start ceiling safe for both motors      */
#define STEP_MOTOR_RUN_PPS_MAX    400U   /* run ceiling safe for both (35BYJ46-bounded)  */

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
