#ifndef SRC_TB_DRV8871_H_
#define SRC_TB_DRV8871_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Testbed for the DRV8871 door motors (U5 Water Door, U7 Trash Door on TIM3).
 *
 * Usage (e.g. from StartDefaultTask in freertos.c):
 *     TB_DRV8871_Init();
 *     for(;;) { TB_DRV8871_Poll(); osDelay(10); }
 * or just call the ready-made loop:
 *     TB_DRV8871_Init();
 *     TB_DRV8871_Loop();     // never returns
 *
 * The tb_*_enable flags are the runtime switches: set one to 1 in the debugger
 * (live watch / expression) and that motor spins; set it back to 0 and it
 * stops. Duty and direction are also tweakable live. */

extern volatile uint8_t tb_wdoor_enable;   /* 1 = run U5 water door, 0 = stop */
extern volatile uint8_t tb_wdoor_duty;     /* 0-100 % PWM                     */
extern volatile uint8_t tb_wdoor_reverse;  /* 0 = Open dir, 1 = Close dir     */

extern volatile uint8_t tb_tdoor_enable;   /* 1 = run U7 trash door, 0 = stop */
extern volatile uint8_t tb_tdoor_duty;     /* 0-100 % PWM                     */
extern volatile uint8_t tb_tdoor_reverse;  /* 0 = Open dir, 1 = Close dir     */

void TB_DRV8871_Init(void);
void TB_DRV8871_Poll(void);   /* apply current enable/duty state once */
void TB_DRV8871_Loop(void);   /* Init already done; poll forever      */

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_DRV8871_H_ */
