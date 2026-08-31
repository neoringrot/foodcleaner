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
 * Open/Close map onto DRV8871 Forward/Reverse. CONFIRMED on the bench
 * (2026-08-25): TDoor_Open() = Forward = 열림, TDoor_Close() = Reverse = 닫힘
 * (testbench tb_tdoor_reverse 0 = 열림, 1 = 닫힘). NOTE the water door is wired
 * the other way round -- see wdoor.h, tb_wdoor_reverse 0 = 닫힘.
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
 * motion controller should stop/brake the motor once the target limit reads 1.
 *
 * 도달 판정은 wdoor.h와 동일한 arm + latched-edge 관용구를 쓴다: 이동 시작 시
 * TDoor_LimitArm(), 판정은 TDoor_ReachedOpen/Close() ("arm 이후의 EXTI 하강엣지"
 * OR 현재 레벨). 근거/주의는 wdoor.h 주석 참조. */

/* Asserted (magnet-present) pin level for the door-limit Halls. 0 = active-low
 * (default, matches the 10k-to-3V3 open-drain wiring). Set to 1 only if a
 * sensor variant drives the line HIGH when active. Keep tdoor.h / wdoor.h equal. */
#ifndef DOOR_LIMIT_ACTIVE_HIGH
#define DOOR_LIMIT_ACTIVE_HIGH 0U
#endif

/* 1 = Reached*()가 EXTI 하강엣지 래치도 함께 본다(기본). wdoor.h와 동일 정의. */
#ifndef DOOR_LIMIT_USE_EDGE
#define DOOR_LIMIT_USE_EDGE 1U
#endif

/* ---- 도어 구동 프로파일 (2026-08-25 벤치 확정) ------------------------
 * 배수문(wdoor.h)과 같은 취지 -- 테스트벤치와 시나리오가 같은 값·같은 규칙으로
 * 배출문을 움직이도록 여기 한 곳에만 둔다. 배수문과 달리 열림/닫힘 동작이 같다:
 *
 *   TDOOR_DUTY(80 %) 고정으로 구동하고, 구동 방향의 THALL이 인식되어도 즉시
 *   서지 않고 TDOOR_OVERRUN_MS(1 s)만큼 **더 돌아 자석을 지난 뒤** 정지한다.
 *   THALL이 끝내 인식되지 않을 때를 위한 상한이 방향별로 있다
 *   (TDOOR_OPEN_MAX_MS / TDOOR_CLOSE_MAX_MS, 둘 다 14.3 s). 상한은 구동 전체의
 *   하드 상한이라, 상한 직전에 인식되면 추가회전이 상한에서 잘린다.
 *
 * 경과시간은 그 방향으로 구동을 시작한 시점부터 잰다. */
#ifndef TDOOR_DUTY
#define TDOOR_DUTY            80U
#endif
#ifndef TDOOR_OVERRUN_MS
#define TDOOR_OVERRUN_MS      1000U
#endif
#ifndef TDOOR_OPEN_MAX_MS
#define TDOOR_OPEN_MAX_MS     14300U
#endif
#ifndef TDOOR_CLOSE_MAX_MS
#define TDOOR_CLOSE_MAX_MS    14300U
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

/* Drop any latched THALL falling edge. Call once when a door move STARTS (not
 * on every re-drive / pulse restart) so the latch only reports this move. */
void TDoor_LimitArm(void);

/* Arrival test used by the scenarios: latched falling edge since the last
 * TDoor_LimitArm() OR the limit level right now. */
uint8_t TDoor_ReachedOpen(void);
uint8_t TDoor_ReachedClose(void);


/* ---- 모니터링 read-only (protocol_r0 0x26 OUTPUT) ---------------------
 * drive = drv8871_drive_t (0 코스트 / 1 정회전 / 2 역회전 / 3 제동),
 * duty  = 마지막으로 지령한 PWM [%]. 피드백이 아니라 "마지막 지령"이다. */
uint8_t TDoor_GetDrive(void);
uint8_t TDoor_GetDuty(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_TDOOR_H_ */
