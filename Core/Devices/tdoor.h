#ifndef DEVICES_TDOOR_H_
#define DEVICES_TDOOR_H_

#include "drv8871.h"

#ifdef __cplusplus
extern "C" {
#endif

/* U7 -- Trash Door DRV8871 (schematic net "T-DOOR").
 *   IN1 = TIM3_CH3 / PC8 (tim3_TDOOR_IN1, net T-DOOR-IN1)
 *   IN2 = TIM3_CH4 / PC9 (tim3_TDOOR_IN2, net T-DOOR-IN2)
 *   EN  = PF9 o_EN_DOOR_TRASH (net TRASH-DOOR-EN)
 *
 * Open/Close map onto DRV8871 Forward/Reverse. The physical open vs. close
 * direction has not been verified on the bench yet -- swap the two calls in
 * tdoor.c if it turns out reversed. */

void TDoor_Init(void);
void TDoor_Enable(void);          /* switch 24V onto VM */
void TDoor_Disable(void);         /* remove VM          */
void TDoor_Open(uint8_t duty_pct);  /* 0-100 */
void TDoor_Close(uint8_t duty_pct); /* 0-100 */
void TDoor_Brake(void);
void TDoor_Stop(void);            /* coast */

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_TDOOR_H_ */
