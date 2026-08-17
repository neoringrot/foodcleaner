#ifndef DEVICES_TDOOR_H_
#define DEVICES_TDOOR_H_

#include "drv8871.h"

#ifdef __cplusplus
extern "C" {
#endif

/* U7 -- Trash Door DRV8871 (schematic net "T-DOOR").
 *   IN1 = TIM3_CH3 / PC8 (tim3_TDOOR_IN1, net T-DOOR-IN1)
 *   IN2 = TIM3_CH4 / PC9 (tim3_TDOOR_IN2, net T-DOOR-IN2)
 *   EN  = PE3 o_EN_DOOR_TRASH (net TRASH-DOOR-EN)
 *
 * Open/Close map onto DRV8871 Forward/Reverse. The physical open vs. close
 * direction has not been verified on the bench yet -- swap the two calls in
 * tdoor.c if it turns out reversed.
 *
 * Limit sensors (R1): two Hall switches report the door end positions, wired
 * straight to the MCU as EXTI inputs:
 *   T-HALL-OPEN  = PF5 exti5_THALL_OPEN  (GPIO_EXTI_THALL_OPEN)  -> fully open
 *   T-HALL-CLOSE = PF2 exti2_THALL_CLOSE (GPIO_EXTI_THALL_CLOSE) -> fully closed
 * Polarity ACTIVE-LOW (confirmed from the R1 net-list): each is a 3-wire
 * open-drain Hall (connector VCC/OUT/GND) with a 10k pull-up to 3V3, so the pin
 * is HIGH when clear and pulls LOW when the magnet is present. EXTI is
 * IT_FALLING, i.e. the edge fires exactly when the door reaches that limit.
 * (Note: T-HALL-CLOSE also feeds a HW NOR gate U13 with Bimetal-80 -- a separate
 * interlock path, independent of this MCU read.)
 * TDoor_AtOpen()/AtClose() return 1 when the door has reached that limit. A
 * motion controller should stop/brake the motor once the target limit reads 1. */

/* Asserted (magnet-present) pin level for the door-limit Halls. 0 = active-low
 * (default, matches the 10k-to-3V3 open-drain wiring). Set to 1 only if a
 * sensor variant drives the line HIGH when active. Keep tdoor.h / wdoor.h equal. */
#ifndef DOOR_LIMIT_ACTIVE_HIGH
#define DOOR_LIMIT_ACTIVE_HIGH 0U
#endif

void TDoor_Init(void);
void TDoor_Enable(void);          /* switch 24V onto VM */
void TDoor_Disable(void);         /* remove VM          */
void TDoor_Open(uint8_t duty_pct);  /* 0-100 */
void TDoor_Close(uint8_t duty_pct); /* 0-100 */
void TDoor_Brake(void);
void TDoor_Stop(void);            /* coast */

/* Limit-switch reads: 1 = door is at that end position (Hall magnet present). */
uint8_t TDoor_AtOpen(void);
uint8_t TDoor_AtClose(void);


/* ---- 모니터링 read-only (protocol_r0 0x26 OUTPUT) ---------------------
 * drive = drv8871_drive_t (0 코스트 / 1 정회전 / 2 역회전 / 3 제동),
 * duty  = 마지막으로 지령한 PWM [%]. 피드백이 아니라 "마지막 지령"이다. */
uint8_t TDoor_GetDrive(void);
uint8_t TDoor_GetDuty(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_TDOOR_H_ */
