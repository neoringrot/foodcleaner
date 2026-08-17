#ifndef DEVICES_WDOOR_H_
#define DEVICES_WDOOR_H_

#include "drv8871.h"

#ifdef __cplusplus
extern "C" {
#endif

/* U5 -- Water Door DRV8871 (schematic net "W-DOOR").
 *   IN1 = TIM3_CH1 / PC6 (tim3_WDOOR_IN1, net W-DOOR-IN1)
 *   IN2 = TIM3_CH2 / PC7 (tim3_WDDOR_IN2, net W-DOOR-IN2)   [main.h spells it WDDOR]
 *   EN  = PE4 o_EN_DOOR_WATER (net WATER-DOOR-EN)
 *
 * Open/Close map onto DRV8871 Forward/Reverse. The physical open vs. close
 * direction has not been verified on the bench yet -- swap the two calls in
 * wdoor.c if it turns out reversed.
 *
 * Limit sensors (R1): two Hall switches report the door end positions, wired
 * straight to the MCU as EXTI inputs:
 *   W-HALL-OPEN  = PF4 exti4_WHALL_OPEN  (GPIO_EXTI_WHALL_OPEN)  -> fully open
 *   W-HALL-CLOSE = PF3 exti3_WHALL_CLOSE (GPIO_EXTI_WHALL_CLOSE) -> fully closed
 * Polarity ACTIVE-LOW (confirmed from the R1 net-list): 3-wire open-drain Hall
 * (connector VCC/OUT/GND) with a 10k pull-up to 3V3 -- pin HIGH when clear,
 * LOW when magnet present. EXTI is IT_FALLING (edge fires at the limit).
 * (Note: W-HALL-CLOSE also feeds HW NOR gate U13 with Bimetal-80 -- a separate
 * interlock, independent of this MCU read.)
 * WDoor_AtOpen()/AtClose() return 1 when the door has reached that limit. A
 * motion controller should stop/brake the motor once the target limit reads 1. */

/* Asserted (magnet-present) pin level for the door-limit Halls. 0 = active-low
 * (default). Keep this equal to tdoor.h's definition. */
#ifndef DOOR_LIMIT_ACTIVE_HIGH
#define DOOR_LIMIT_ACTIVE_HIGH 0U
#endif

void WDoor_Init(void);
void WDoor_Enable(void);          /* switch 24V onto VM */
void WDoor_Disable(void);         /* remove VM          */
void WDoor_Open(uint8_t duty_pct);  /* 0-100 */
void WDoor_Close(uint8_t duty_pct); /* 0-100 */
void WDoor_Brake(void);
void WDoor_Stop(void);            /* coast */

/* Limit-switch reads: 1 = door is at that end position (Hall magnet present). */
uint8_t WDoor_AtOpen(void);
uint8_t WDoor_AtClose(void);


/* ---- 모니터링 read-only (protocol_r0 0x26 OUTPUT) ---------------------
 * drive = drv8871_drive_t (0 코스트 / 1 정회전 / 2 역회전 / 3 제동),
 * duty  = 마지막으로 지령한 PWM [%]. 피드백이 아니라 "마지막 지령"이다. */
uint8_t WDoor_GetDrive(void);
uint8_t WDoor_GetDuty(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_WDOOR_H_ */
