#ifndef DEVICES_WDOOR_H_
#define DEVICES_WDOOR_H_

#include "drv8871.h"

#ifdef __cplusplus
extern "C" {
#endif

/* U5 -- Water Door DRV8871 (schematic net "W-DOOR").
 *   IN1 = TIM3_CH1 / PC6 (tim3_WDOOR_IN1, net W-DOOR-IN1)
 *   IN2 = TIM3_CH2 / PC7 (tim3_WDDOR_IN2, net W-DOOR-IN2)   [main.h spells it WDDOR]
 *   EN  = PF10 o_EN_DOOR_WATER (net WATER-DOOR-EN)
 *
 * Open/Close map onto DRV8871 Forward/Reverse. The physical open vs. close
 * direction has not been verified on the bench yet -- swap the two calls in
 * wdoor.c if it turns out reversed. */

void WDoor_Init(void);
void WDoor_Enable(void);          /* switch 24V onto VM */
void WDoor_Disable(void);         /* remove VM          */
void WDoor_Open(uint8_t duty_pct);  /* 0-100 */
void WDoor_Close(uint8_t duty_pct); /* 0-100 */
void WDoor_Brake(void);
void WDoor_Stop(void);            /* coast */

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_WDOOR_H_ */
