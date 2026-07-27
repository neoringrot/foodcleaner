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
