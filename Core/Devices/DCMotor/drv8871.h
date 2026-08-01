#ifndef DEVICES_DRV8871_H_
#define DEVICES_DRV8871_H_

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TI DRV8871 brushed-DC motor driver (SLVSCY9B), instance-based driver.
 *
 * Unlike hasudo1 (single TIM1 driver), foodcleaner has multiple DRV8871s that
 * share TIM3, so each physical driver is described by a handle instead of being
 * hard-wired to one timer/channel pair:
 *
 *   U5  Water Door : TIM3_CH1/CH2 on PC6/PC7 (tim3_WDOOR_IN1 / tim3_WDDOR_IN2),
 *                    VM gated by o_EN_DOOR_WATER (PF10, net WATER-DOOR-EN)
 *   U7  Trash Door : TIM3_CH3/CH4 on PC8/PC9 (tim3_TDOOR_IN1 / tim3_TDOOR_IN2),
 *                    VM gated by o_EN_DOOR_TRASH (PF9,  net TRASH-DOOR-EN)
 *   U6  Lift       : IN1/IN2 as plain GPIO PG3/PG4 (LIFT-IN1 / LIFT-IN2, NOT
 *                    PWM -- full-speed bang-bang), VM gated by o_MTR_DC_LIFT
 *                    (PB12). Driven by lift_motor.c, not a handle below.
 *
 * Motor (all three U5/U6/U7): JKONGMOTOR JK30ZYT-5840-577, 24 VDC brushed PMDC
 * with a 5840 worm gearbox (body 58 x 40 x 36 mm). Envelope from the 5840-series
 * datasheet (DC 24 V): no-load current <=0.1..0.2 A, rated current <=0.6..1.6 A,
 * STALL current 1.4..4.4 A depending on winding, gear-limited output torque
 * ~70 kgf-cm (~6.9 N-m). The "-577" suffix is the (high) reduction variant, so
 * output speed is a few rpm -- slow, high-torque door/lift travel.
 * Two consequences for this driver:
 *   1. The worm gear is SELF-LOCKING (non-backdriveable): once VM is removed the
 *      door/lift holds position on its own. Coast (DRV8871_Coast / Lift idle)
 *      therefore holds; an active Brake is only needed to stop travel abruptly.
 *   2. Stall current can reach ~4.4 A on the high-power winding, above the
 *      DRV8871's ~3.6 A internal limit -- the chip will current-limit (not fault)
 *      at end-stops. Confirm the board's ILIM resistor suits the actual winding.
 * There is no encoder/tacho on these motors, so speed is pure open-loop PWM
 * duty (coast-decay); unlike the DRV8306/BLDC path there is no measured-RPM
 * feedback and nothing to RPM-calibrate here.
 *
 * IN1/IN2 are driven by TIM3 PWM; direction/braking follow the IN1/IN2 truth
 * table (datasheet Table 1):
 *
 *   IN1  IN2  OUT1  OUT2  Mode
 *    0    0   Hi-Z  Hi-Z  Coast (sleep entered after ~1ms if held)
 *    0    1    L     H    Reverse
 *    1    0    H     L    Forward
 *    1    1    L     L    Brake (low-side slow decay)
 *
 * Speed is set by PWM'ing the driven input while holding the other input low
 * (coast-decay speed control, DRV8871 datasheet section 7.4.2).
 *
 * Enable: the DRV8871 has no logic enable pin; a per-motor GPIO gates the motor
 * supply through Q(NPN DTC144EET1G) -> P-ch high-side FET onto VM. It is
 * active-high / non-inverting: EN HIGH switches 24V onto VM, EN LOW removes it.
 * Drive VM on before commanding a direction and off after coasting. */

/* Above audible range, well under the DRV8871 200kHz IN max. All DRV8871
 * instances sharing one timer MUST use the same period, so this is a single
 * shared #define (ARR is a timer-wide property, not per-channel). */
#define DRV8871_PWM_FREQ_HZ  20000U

typedef struct
{
	TIM_HandleTypeDef *htim;   /* PWM timer (e.g. &htim3)            */
	uint32_t           in1_ch; /* TIM_CHANNEL_x driving IN1         */
	uint32_t           in2_ch; /* TIM_CHANNEL_x driving IN2         */
	GPIO_TypeDef      *en_port;/* VM-enable GPIO port               */
	uint16_t           en_pin; /* VM-enable GPIO pin                */
	uint32_t           arr;    /* auto-reload (filled by Init)      */
} DRV8871_HandleTypeDef;

void DRV8871_Init(DRV8871_HandleTypeDef *h);
void DRV8871_Enable(DRV8871_HandleTypeDef *h);   /* EN HIGH: switch 24V onto VM */
void DRV8871_Disable(DRV8871_HandleTypeDef *h);  /* EN LOW:  remove VM          */
void DRV8871_Forward(DRV8871_HandleTypeDef *h, uint8_t duty_pct); /* 0-100 */
void DRV8871_Reverse(DRV8871_HandleTypeDef *h, uint8_t duty_pct); /* 0-100 */
void DRV8871_Brake(DRV8871_HandleTypeDef *h);
void DRV8871_Coast(DRV8871_HandleTypeDef *h);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_DRV8871_H_ */
